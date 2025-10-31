from machine import I2C, Pin, SPI
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
LED_PIN     = 16               # external LED (active-high)
BTN_PIN     = 14               # button to GND (active-low)
STATUS_MS   = 2000
# ------------------------

DT_MS    = int(1000 // RATE_HZ)
REC_FMT  = "<Ifffffff"         # [t_ms][lx ly lz qi qj qk qr]
REC_SIZE = struct.calcsize(REC_FMT)

# States
S_BOOT, S_READY, S_REC, S_STOPPING, S_ERR = range(5)

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
    spi = SPI(1, baudrate=1_000_000, polarity=0, phase=0,
              sck=Pin(10), mosi=Pin(11), miso=Pin(12))
    sd  = sdcard.SDCard(spi, Pin(CS_PIN, Pin.OUT, value=1))
    uos.mount(sd, "/sd")
    spi.init(baudrate=SPI_BAUD)
    return sd, spi

def umount_sd():
    try: uos.umount("/sd")
    except: pass

# --- BNO bring-up ---
i2c = I2C(0, sda=Pin(4), scl=Pin(5), freq=I2C_FREQ)
bno = BNO08X(i2c, debug=False)

# bias at low rate (acc & gravity only)
bno.enable_feature(BNO_REPORT_ACCELEROMETER, BIAS_RATE)
bno.enable_feature(BNO_REPORT_GRAVITY,      BIAS_RATE)

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

    # mount SD and go READY
    sd, spi = mount_sd()
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
    print("ERROR:", e)
    while True:
        led.value(1 if blink_pattern(ticks_ms(), state) else 0)
        sleep_ms(20)
