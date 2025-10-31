from machine import I2C, Pin, SPI, ADC
from utime import ticks_ms, ticks_diff, sleep_ms
from gc import collect as gc_collect
import uos, struct, sdcard
from bno08x import *

# -------- CONFIG --------
RATE_HZ     = 100
BIAS_RATE   = 25
BIAS_SECS   = 4
BLOCK_RECS  = 512              # ~18 KB per flush
I2C_FREQ    = 400_000
SPI_BAUD    = 12_000_000
CS_PIN      = 13
LED_PIN     = 28              	# external LED (active-high)
BTN_PIN     = 14               # button to GND (active-low)
STATUS_MS   = 2000
# ------------------------

DT_MS    = int(1000 // RATE_HZ)
REC_FMT  = "<Ifffffff"         # [t_ms][lx ly lz qi qj qk qr]
REC_SIZE = struct.calcsize(REC_FMT)

# States
S_BOOT, S_READY, S_REC, S_STOPPING, S_ERR = range(5)

# --- Battery Monitoring (LiPo SHIM) ---
# VSYS monitoring via ADC29 (internal ÷3 divider)
# On battery: VSYS ≈ battery voltage
# On USB: VSYS ≈ 5V (via power-mux diode)
VBUS_SENSE = Pin(24, Pin.IN)  # USB present signal
VSYS_ADC = ADC(29)  # ADC3, internal divider: VSYS/3
VREF = 3.3
DIV = 3.0  # internal VSYS→ADC ÷3 divider

def read_vsys_voltage():
    """Read VSYS voltage using ADC29 with internal ÷3 divider."""
    try:
        raw = VSYS_ADC.read_u16()
        # Convert: (raw/65535) * 3.3V * 3 (divider) = VSYS voltage
        vsys_v = (raw / 65535.0) * VREF * DIV
        return vsys_v, raw
    except Exception as e:
        return None, None

def read_battery_status():
    """Read battery/VSYS status with USB detection."""
    try:
        usb_present = bool(VBUS_SENSE.value())
        vsys_v, adc_raw = read_vsys_voltage()
        
        if vsys_v is None:
            return None
        
        if usb_present:
            # USB is present; VSYS ≈ 5V (not battery voltage)
            return {
                "usb": True,
                "vsys_v": vsys_v,
                "battery_v": None,
                "percent": None,
                "adc_raw": adc_raw
            }
        
        # Battery mode: VSYS ≈ battery voltage
        # Piecewise LiPo percentage (open-circuit, approximate)
        # 4.20=100%, 4.00≈85, 3.90≈70, 3.80≈55, 3.70≈40, 3.60≈25, 3.50≈10, 3.40≈5, <3.30 critical
        edges = [4.20, 4.00, 3.90, 3.80, 3.70, 3.60, 3.50, 3.40, 3.30]
        perc = [100, 85, 70, 55, 40, 25, 10, 5, 0]
        percentage = next((perc[i] for i, e in enumerate(edges) if vsys_v >= e), 0)
        
        return {
            "usb": False,
            "vsys_v": vsys_v,
            "battery_v": vsys_v,  # On battery, VSYS ≈ battery voltage
            "percent": percentage,
            "adc_raw": adc_raw
        }
    except Exception as e:
        return None

def print_battery_status():
    """Print battery/VSYS status at startup."""
    status = read_battery_status()
    print("=" * 40)
    if status is None:
        print("Battery Status: Unable to read (check ADC/GPIO)")
        print("=" * 40)
        return
    
    if status["usb"]:
        print("Power Status (USB Connected):")
        print(f"  VSYS: {status['vsys_v']:.2f} V")
        print(f"  ADC Raw: {status['adc_raw']}")
        print(f"  Note: Battery voltage not available on USB")
    else:
        print("Battery Status (LiPo SHIM):")
        print(f"  Battery Voltage: {status['battery_v']:.2f} V")
        print(f"  VSYS: {status['vsys_v']:.2f} V")
        print(f"  Level: {status['percent']}%")
        print(f"  ADC Raw: {status['adc_raw']}")
        # Status indicator
        if status['percent'] >= 80:
            status_str = "✓ Good"
        elif status['percent'] >= 50:
            status_str = "○ OK"
        elif status['percent'] >= 20:
            status_str = "! Low"
        else:
            status_str = "⚠ Critical"
        print(f"  Status: {status_str}")
    print("=" * 40)

# --- LED / Button ---
led = Pin(LED_PIN, Pin.OUT, value=0)
btn = Pin(BTN_PIN, Pin.IN, Pin.PULL_UP)

def btn_pressed():
    return btn.value() == 0

def blink_pattern(now_ms, state):
    if state == S_BOOT:       # slow blink: 200ms ON per 1s
        return (now_ms % 1000) < 200
    if state == S_READY:      # double-blink every 2s
        p = now_ms % 2000
        return (0 <= p < 120) or (240 <= p < 360)
    if state == S_REC:        # solid ON (flush flicker handled elsewhere)
        return True
    if state == S_STOPPING:   # triple-blink (1s window)
        p = now_ms % 1000
        return (0 <= p < 80) or (200 <= p < 280) or (400 <= p < 480)
    if state == S_ERR:        # fast blink
        return (now_ms % 200) < 100
    return False

# --- Filesystem helpers ---
def next_name(prefix="bno", ext="bin"):
    n = 0
    while True:
        name = f"/sd/{prefix}_{n:03d}.{ext}"
        try:
            uos.stat(name); n += 1
        except OSError:
            return name

def mount_sd():
    """Mount SD card with error diagnostics."""
    try:
        print("Mounting SD card...")
        # Check power status before mounting
        status = read_battery_status()
        if status and not status["usb"]:
            print(f"  Running on battery (VSYS: {status['vsys_v']:.2f}V)")
            print("  NOTE: Ensure SD breakout is powered from 3V3(OUT) or VSYS, NOT VBUS!")
        
        # Initialize SPI at lower speed first
        spi = SPI(1, baudrate=1_000_000, polarity=0, phase=0,
                  sck=Pin(10), mosi=Pin(11), miso=Pin(12))
        sd = sdcard.SDCard(spi, Pin(CS_PIN, Pin.OUT, value=1))
        uos.mount(sd, "/sd")
        # Switch to full speed after mount
        spi.init(baudrate=SPI_BAUD)
        print("SD card mounted successfully")
        return sd, spi
    except Exception as e:
        error_msg = str(e)
        print(f"SD card mount FAILED: {error_msg}")
        if "OSError" in str(type(e)) or "ENODEV" in error_msg:
            print("  DIAGNOSIS: SD card not detected or not powered")
            print("  CHECK:")
            print("    1. SD breakout VCC → 3V3(OUT) (pin 36) or VSYS (pin 39)")
            print("    2. SD breakout VCC NOT connected to VBUS (pin 40)")
            print("    3. SD card inserted properly")
            print("    4. All SPI connections: CS(13), SCK(10), MOSI(11), MISO(12)")
            print("    5. Common GND connection")
        raise

def umount_sd():
    try: uos.umount("/sd")
    except: pass

# --- BNO bring-up (will be initialized in main) ---
i2c = None
bno = None

def still_bias(seconds=BIAS_SECS):
    n = int(seconds * BIAS_RATE)
    sx = sy = sz = 0.0
    dt_bias = int(1000 // BIAS_RATE)
    for k in range(n):
        ax, ay, az = bno.acc
        gx, gy, gz = bno.gravity
        sx += ax - gx; sy += ay - gy; sz += az - gz
        if (k & 7) == 0: gc_collect()
        sleep_ms(dt_bias)
    return sx/n, sy/n, sz/n

# switch to logging features/rates
def enable_logging_features():
    bno.enable_feature(BNO_REPORT_ACCELEROMETER, RATE_HZ)
    bno.enable_feature(BNO_REPORT_GRAVITY,      RATE_HZ)
    bno.enable_feature(BNO_REPORT_GAME_ROTATION_VECTOR, RATE_HZ)
    bno.set_quaternion_euler_vector(BNO_REPORT_GAME_ROTATION_VECTOR)

# --- Recording loop (preallocated buffer + pack_into) ---
def record_session(bias_xyz, fname):
    bx, by, bz = bias_xyz
    f = open(fname, "wb")
    # header: magic(4) ver(1) rate(2) reserved(1)
    f.write(struct.pack("<IBHB", 0x424E4F31, 1, RATE_HZ, 0))  # "BNO1"

    buf  = bytearray(REC_SIZE * BLOCK_RECS)
    pack_into = struct.pack_into
    widx = 0
    total = 0
    t0 = ticks_ms()
    t_last = t0

    print("Recording ->", fname, "| rate =", RATE_HZ, "Hz")
    while True:
        now = ticks_ms()
        # Solid LED during recording
        led.value(1)

        # stop on button press (debounced)
        if btn_pressed():
            sleep_ms(40)
            if btn_pressed():
                break

        t_ms = ticks_diff(now, t0)
        ax, ay, az = bno.acc
        gx, gy, gz = bno.gravity
        lx, ly, lz = ax - gx - bx, ay - gy - by, az - gz - bz
        qi, qj, qk, qr = bno.quaternion

        pack_into(REC_FMT, buf, widx * REC_SIZE, t_ms, lx, ly, lz, qi, qj, qk, qr)
        widx += 1; total += 1

        if widx == BLOCK_RECS:
            t_w0 = ticks_ms()
            f.write(buf); f.flush()
            # brief flush flicker
            led.value(0); sleep_ms(40); led.value(1)
            print("[FLUSH] samples=%d wrote=%d bytes in %d ms" %
                  (total, len(buf), ticks_diff(ticks_ms(), t_w0)))
            widx = 0
            gc_collect()

        if ticks_diff(now, t_last) >= STATUS_MS:
            print("[STAT] t=%.1fs count=%d (lx,ly,lz)=(%.3f,%.3f,%.3f)" %
                  (ticks_diff(now, t0)/1000, total, lx, ly, lz))
            t_last = now

        # pace
        elapsed = ticks_diff(ticks_ms(), now)
        rem = DT_MS - elapsed
        if rem > 0:
            sleep_ms(rem)
        if (total & 511) == 0:
            gc_collect()

    # graceful close
    if widx:
        f.write(memoryview(buf)[:widx*REC_SIZE])
    try:
        f.flush(); f.close()
    except:
        pass
    print("Recording stopped. File closed:", fname)

# -------- MAIN FSM --------
try:
    state = S_BOOT
    
    # Startup delay for battery stability (critical for battery-only operation)
    sleep_ms(1000)  # Longer delay for battery power stability
    
    # Start LED immediately to show script is running (blink pattern)
    for _ in range(2):
        led.value(1)
        sleep_ms(50)
        led.value(0)
        sleep_ms(50)
    
    # Display battery status at startup (when connected via USB/Thonny)
    print_battery_status()
    
    # Initialize I2C with retry logic
    print("Initializing I2C...")
    led.value(1)  # LED on during I2C init
    i2c = None
    for attempt in range(3):
        try:
            i2c = I2C(0, sda=Pin(4), scl=Pin(5), freq=I2C_FREQ)
            sleep_ms(200)  # Delay after I2C init
            # Test I2C by scanning
            devices = i2c.scan()
            if devices:
                print(f"I2C scan found devices: {devices}")
                led.value(0)
                sleep_ms(100)
                led.value(1)  # Flash success
                sleep_ms(50)
                led.value(0)
                break
            else:
                print(f"I2C scan attempt {attempt+1} found no devices, retrying...")
                # Blink for retry
                led.value(0)
                sleep_ms(100)
                led.value(1)
                sleep_ms(300)
        except Exception as e:
            print(f"I2C init attempt {attempt+1} failed: {e}")
            led.value(0)
            if attempt < 2:
                sleep_ms(300)
                led.value(1)
            else:
                led.value(0)
                raise
    
    if i2c is None:
        led.value(0)
        raise Exception("Failed to initialize I2C after 3 attempts")
    
    # Initialize BNO085 with retry logic
    print("Initializing BNO085...")
    led.value(1)  # LED on during BNO init
    bno = None
    for attempt in range(3):
        try:
            sleep_ms(300)  # Extra delay before BNO init
            bno = BNO08X(i2c, debug=False)
            sleep_ms(500)  # Allow BNO085 to fully initialize
            # Object creation successful means I2C communication works
            print("BNO085 initialized successfully")
            led.value(0)
            sleep_ms(100)
            led.value(1)  # Flash success
            sleep_ms(50)
            led.value(0)
            break
        except Exception as e:
            print(f"BNO085 init attempt {attempt+1} failed: {e}")
            led.value(0)
            if attempt < 2:
                # Double blink for retry
                for _ in range(2):
                    led.value(1)
                    sleep_ms(100)
                    led.value(0)
                    sleep_ms(100)
                sleep_ms(500)
                led.value(1)
            else:
                led.value(0)
                raise
    
    if bno is None:
        led.value(0)
        raise Exception("Failed to initialize BNO085 after 3 attempts")
    
    # Enable features and verify by reading a value
    print("Enabling BNO085 features...")
    led.value(1)  # LED on during feature enable
    for attempt in range(3):
        try:
            bno.enable_feature(BNO_REPORT_ACCELEROMETER, BIAS_RATE)
            sleep_ms(200)  # Allow time for feature to enable
            bno.enable_feature(BNO_REPORT_GRAVITY,      BIAS_RATE)
            sleep_ms(300)  # Allow features to enable
            # Now verify by reading accel (features are enabled)
            _ = bno.acc  # This verifies communication is working
            print("BNO085 features enabled and verified")
            led.value(0)
            sleep_ms(100)
            led.value(1)  # Flash success
            sleep_ms(50)
            led.value(0)
            break
        except Exception as e:
            print(f"Feature enable attempt {attempt+1} failed: {e}")
            led.value(0)
            if attempt < 2:
                # Triple blink for retry
                for _ in range(3):
                    led.value(1)
                    sleep_ms(80)
                    led.value(0)
                    sleep_ms(80)
                sleep_ms(500)
                led.value(1)
            else:
                led.value(0)
                raise
    
    print("Initializing bias… keep the unit still.")
    # show bias pattern during acquisition
    t_bias0 = ticks_ms()
    while ticks_diff(ticks_ms(), t_bias0) < (BIAS_SECS * 1000):
        led.value(1 if blink_pattern(ticks_ms(), state) else 0)
        sleep_ms(10)
    bx, by, bz = still_bias()
    print("Bias (lin-acc) ~ bx=%.4f by=%.4f bz=%.4f" % (bx, by, bz))

    # enable full logging features
    enable_logging_features()
    gc_collect()

    # Check battery status before SD mount (if on battery)
    status = read_battery_status()
    if status and not status["usb"]:
        if status["percent"] is not None and status["percent"] < 10:
            raise Exception(f"Battery too low ({status['percent']}%). Charge before use. VSYS: {status['vsys_v']:.2f}V")
        if status["vsys_v"] < 3.5:
            raise Exception(f"VSYS too low ({status['vsys_v']:.2f}V). Charge before use.")

    # mount SD and go READY
    try:
        sd, spi = mount_sd()
    except Exception as e:
        error_msg = str(e)
        if "mount" in error_msg.lower() or "sdcard" in error_msg.lower() or "spi" in error_msg.lower():
            raise Exception(f"SD card mount failed. Check wiring: SD breakout VCC must be on 3V3(OUT) or VSYS, NOT VBUS. Original error: {error_msg}")
        raise
    state = S_READY
    print("Ready. Press button to START recording.")
    while not btn_pressed():
        led.value(1 if blink_pattern(ticks_ms(), state) else 0)
        sleep_ms(20)
    while btn_pressed(): sleep_ms(10)  # release

    # start recording
    state = S_REC
    led.value(1)
    fname = next_name()
    record_session((bx, by, bz), fname)

    # STOPPING
    state = S_STOPPING
    umount_sd()
    t1 = ticks_ms()
    while ticks_diff(ticks_ms(), t1) < 1000:
        led.value(1 if blink_pattern(ticks_ms(), state) else 0)
        sleep_ms(10)
    led.value(0)
    print("SD unmounted. Safe to power off.")

except Exception as e:
    state = S_ERR
    led.value(0)
    # Triple flash before error pattern to make it visible
    for _ in range(3):
        led.value(1)
        sleep_ms(100)
        led.value(0)
        sleep_ms(100)
    sleep_ms(200)
    print("ERROR:", e)
    # Fast blink error pattern (indefinitely)
    while True:
        led.value(1 if blink_pattern(ticks_ms(), state) else 0)
        sleep_ms(20)

