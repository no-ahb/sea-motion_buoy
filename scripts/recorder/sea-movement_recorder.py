from machine import I2C, Pin, SPI, ADC
from utime import ticks_ms, ticks_diff, sleep_ms
from gc import collect as gc_collect
import uos, struct, sdcard
from bno08x import *

# -------- CONFIG --------
RATE_HZ       = 200
BIAS_RATE     = 25
BIAS_SECS     = 10
BLOCK_RECS    = 512                # 512 * 32B = 16 KiB blocks
FLUSH_EVERY_N = 1                  # flush every block (debug visibility)
I2C_FREQ      = 400_000
SPI_BAUD      = 12_000_000
CS_PIN        = 13
LED_PIN       = 28                 # use GP16 for LED (safe choice)
BTN_PIN       = 14                 # button to GND (internal pull-up)
STATUS_MS     = 2000
# ------------------------

DT_MS    = int(1000 // RATE_HZ)
REC_FMT  = "<Ifffffff"             # [t_ms][lx ly lz qi qj qk qr] = 32 B
REC_SIZE = struct.calcsize(REC_FMT)

# States
S_BOOT, S_READY, S_REC, S_STOPPING, S_ERR = range(5)

# --- Power sense (optional; harmless if unused) ---
VBUS_SENSE = Pin(24, Pin.IN)  # USB present
VSYS_ADC   = ADC(29)          # VSYS/3
VREF, DIV  = 3.3, 3.0
def read_vsys_voltage():
    try:
        raw = VSYS_ADC.read_u16()
        return (raw / 65535.0) * VREF * DIV, raw
    except:
        return None, None

def print_battery_status():
    vsys_v, adc_raw = read_vsys_voltage()
    print("=" * 40)
    if vsys_v is None:
        print("Battery Status: Unable to read"); print("=" * 40); return
    if VBUS_SENSE.value():
        print("Power Status (USB Connected):")
        print("  VSYS: %.2f V" % vsys_v); print("  ADC Raw:", adc_raw)
        print("  Note: Battery voltage not available on USB")
    else:
        print("Battery Status (LiPo SHIM):")
        print("  VSYS: %.2f V" % vsys_v); print("  ADC Raw:", adc_raw)
    print("=" * 40)

# --- LED / Button with IRQ-latched stop ---
led = Pin(LED_PIN, Pin.OUT, value=0)
btn = Pin(BTN_PIN, Pin.IN, Pin.PULL_UP)

stop_flag = False
last_press_ms = 0
def _btn_irq(pin):
    # latch a stop request on falling edge with simple debounce
    global stop_flag, last_press_ms
    now = ticks_ms()
    if ticks_diff(now, last_press_ms) > 200 and pin.value() == 0:
        stop_flag = True
        last_press_ms = now

btn.irq(trigger=Pin.IRQ_FALLING, handler=_btn_irq)

def btn_pressed():
    return btn.value() == 0

def blink_pattern(now_ms, state):
    if state == S_BOOT:    return (now_ms % 1000) < 200
    if state == S_READY:
        p = now_ms % 2000; return (0 <= p < 120) or (240 <= p < 360)
    if state == S_REC:     return True
    if state == S_STOPPING:
        p = now_ms % 1000; return (0 <= p < 80) or (200 <= p < 280) or (400 <= p < 480)
    if state == S_ERR:     return (now_ms % 200) < 100
    return False

# --- Filesystem helpers ---
def next_name(prefix="bno", ext="bin"):
    n = 0
    while True:
        name = f"/sd/{prefix}_{n:03d}.{ext}"
        try: uos.stat(name); n += 1
        except OSError: return name

def mount_sd():
    print("Mounting SD card…")
    spi = SPI(1, baudrate=1_000_000, polarity=0, phase=0,
              sck=Pin(10), mosi=Pin(11), miso=Pin(12))
    sd  = sdcard.SDCard(spi, Pin(CS_PIN, Pin.OUT, value=1))
    uos.mount(sd, "/sd")
    spi.init(baudrate=SPI_BAUD)
    print("SD card mounted.")
    return sd, spi

def umount_sd():
    try: uos.umount("/sd")
    except: pass

# --- BNO setup ---
i2c = I2C(0, sda=Pin(4), scl=Pin(5), freq=I2C_FREQ)
bno = BNO08X(i2c, debug=False)
bno.enable_feature(BNO_REPORT_ACCELEROMETER, BIAS_RATE)
bno.enable_feature(BNO_REPORT_GRAVITY,      BIAS_RATE)

def still_bias(seconds=BIAS_SECS):
    n = int(seconds * BIAS_RATE); sx=sy=sz=0.0; dt=int(1000//BIAS_RATE)
    for k in range(n):
        ax,ay,az = bno.acc; gx,gy,gz = bno.gravity
        sx += ax-gx; sy += ay-gy; sz += az-gz
        if (k & 7)==0: gc_collect()
        sleep_ms(dt)
    return sx/n, sy/n, sz/n

def enable_logging_features():
    bno.enable_feature(BNO_REPORT_ACCELEROMETER, RATE_HZ)
    bno.enable_feature(BNO_REPORT_GRAVITY,      RATE_HZ)
    bno.enable_feature(BNO_REPORT_GAME_ROTATION_VECTOR, RATE_HZ)
    bno.set_quaternion_euler_vector(BNO_REPORT_GAME_ROTATION_VECTOR)

# --- Recording loop (double-buffered) ---
def record_session(bias_xyz, fname):
    global stop_flag
    stop_flag = False  # clear any prior latch

    bx,by,bz = bias_xyz
    f = open(fname, "wb")
    f.write(struct.pack("<IBHB", 0x424E4F31, 1, RATE_HZ, 0))  # "BNO1"

    buf_a = bytearray(REC_SIZE * BLOCK_RECS)
    buf_b = bytearray(REC_SIZE * BLOCK_RECS)
    active = buf_a
    pack_into = struct.pack_into
    widx = 0; total=0; blocks=0
    t0 = ticks_ms(); t_last = t0
    t_first = None; t_last_samp = None

    print("Recording ->", fname, "| rate =", RATE_HZ, "Hz")
    led.value(1)  # solid while recording

    try:
        while True:
            # honor stop as soon as possible (latched by IRQ)
            if stop_flag:
                print("Stop requested (button).")
                break

            now = ticks_ms()
            if t_first is None: t_first = now
            t_last_samp = now
            t_ms = ticks_diff(now, t0)

            # read sensors
            ax,ay,az = bno.acc
            gx,gy,gz = bno.gravity
            lx,ly,lz = ax-gx-bx, ay-gy-by, az-gz-bz
            qi,qj,qk,qr = bno.quaternion

            pack_into(REC_FMT, active, widx*REC_SIZE, t_ms, lx,ly,lz, qi,qj,qk,qr)
            widx += 1; total += 1

            if widx == BLOCK_RECS:
                # swap buffers and write the full one
                to_write = active
                active = buf_b if active is buf_a else buf_a
                widx = 0

                t_w0 = ticks_ms()
                f.write(to_write); blocks += 1
                # visible activity each block
                led.value(0); sleep_ms(8); led.value(1)

                if FLUSH_EVERY_N and (blocks % FLUSH_EVERY_N == 0):
                    f.flush()
                    print("[FLUSH] blocks=%d samples=%d (%.1f KiB)" %
                          (blocks, total, (REC_SIZE*BLOCK_RECS)/1024))
                else:
                    print("[BLOCK] blocks=%d samples=%d wrote=%d bytes in %d ms" %
                          (blocks, total, len(to_write), ticks_diff(ticks_ms(), t_w0)))
                gc_collect()

            # periodic status
            if ticks_diff(now, t_last) >= STATUS_MS:
                print("[STAT] t=%.1fs count=%d (lx,ly,lz)=(%.3f,%.3f,%.3f)" %
                      (ticks_diff(now, t0)/1000, total, lx, ly, lz))
                t_last = now

            # pacing
            rem = DT_MS - ticks_diff(ticks_ms(), now)
            if rem > 0: sleep_ms(rem)
            if (total & 511) == 0: gc_collect()

    finally:
        # write any remainder
        if widx:
            f.write(memoryview(active)[:widx*REC_SIZE]); blocks += 1
        try:
            f.flush(); led.value(0); sleep_ms(20); led.value(1); f.close()
        except: pass
        # sidecar meta
        if t_first and t_last_samp:
            dur_ms = ticks_diff(t_last_samp, t_first)
            eff = (total*1000.0/dur_ms) if dur_ms>0 else 0.0
            meta = fname.replace(".bin", "_meta.txt")
            try:
                with open(meta, "w") as mf:
                    mf.write("Recording: %s\n" % fname)
                    mf.write("Target rate: %d Hz\n" % RATE_HZ)
                    mf.write("Effective rate: %.2f Hz\n" % eff)
                    mf.write("Total samples: %d\n" % total)
                    mf.write("Duration: %.2f s\n" % (dur_ms/1000.0))
                    mf.write("Blocks: %d\n" % blocks)
                    mf.write("Block size: %d records (%d bytes)\n" %
                             (BLOCK_RECS, REC_SIZE*BLOCK_RECS))
                print("Effective rate: %.2f Hz" % eff)
                print("Metadata:", meta)
            except Exception as e:
                print("Meta write failed:", e)
        print("Recording stopped. File closed:", fname)

# -------- MAIN --------
try:
    # boot blink
    for _ in range(2): led.value(1); sleep_ms(60); led.value(0); sleep_ms(60)
    print_battery_status()

    print("Initializing I2C/BNO…")
    # (i2c and bno already created above)

    print("Biasing… keep still.")
    t0 = ticks_ms()
    while ticks_diff(ticks_ms(), t0) < (BIAS_SECS*1000):
        led.value(1 if blink_pattern(ticks_ms(), S_BOOT) else 0); sleep_ms(10)
    bx,by,bz = still_bias()
    print("Bias ~ bx=%.4f by=%.4f bz=%.4f" % (bx,by,bz))

    enable_logging_features(); gc_collect()

    # SD mount and READY
    sd, spi = mount_sd()
    print("Ready. Press button to START.")
    while not btn_pressed():
        led.value(1 if blink_pattern(ticks_ms(), S_READY) else 0); sleep_ms(20)
    # wait for release (debounce)
    while btn_pressed(): sleep_ms(10)

    # RECORD
    led.value(1)
    fname = next_name()
    record_session((bx,by,bz), fname)

    # STOP / UNMOUNT
    print("Unmounting SD…")
    umount_sd()
    t1 = ticks_ms()
    while ticks_diff(ticks_ms(), t1) < 800:
        led.value(1 if blink_pattern(ticks_ms(), S_STOPPING) else 0); sleep_ms(10)
    led.value(0)
    print("Done. Safe to power off.")

except Exception as e:
    print("ERROR:", e)
    while True:
        led.value(1 if blink_pattern(ticks_ms(), S_ERR) else 0); sleep_ms(20)
