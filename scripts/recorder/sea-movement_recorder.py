from machine import I2C, Pin, SPI, ADC
from utime import ticks_ms, ticks_diff, sleep_ms
from gc import collect as gc_collect
import uos, struct, sdcard
from bno08x import *

# -------- CONFIG --------
RATE_HZ       = 100            # start at 100 Hz (stable); try 200 after verifying
BIAS_RATE     = 25
BIAS_SECS     = 10
BLOCK_RECS    = 1024            # 16 KiB blocks (512 * 32 B)
FLUSH_EVERY_N = 0              # visible flushing while you debug
I2C_FREQ      = 400_000
SPI_BAUD      = 12_000_000
CS_PIN        = 13
LED_PIN       = 28
BTN_PIN       = 14
STATUS_MS     = 2000
# ------------------------

DT_MS    = int(1000 // RATE_HZ)
REC_FMT  = "<Ifffffff"         # t_ms, lx,ly,lz, qi,qj,qk,qr
REC_SIZE = struct.calcsize(REC_FMT)

# --- Power (optional) ---
VBUS_SENSE = Pin(24, Pin.IN)
VSYS_ADC   = ADC(29); VREF=3.3; DIV=3.0
def print_battery_status():
    raw = VSYS_ADC.read_u16()
    vsys = (raw/65535.0)*VREF*DIV
    print("="*40)
    if VBUS_SENSE.value():
        print("Power: USB, VSYS=%.2f V (ADC=%d)" % (vsys, raw))
    else:
        print("Power: Battery, VSYS=%.2f V (ADC=%d)" % (vsys, raw))
    print("="*40)

# --- LED / Button (IRQ-latched stop) ---
led = Pin(LED_PIN, Pin.OUT, value=0)
btn = Pin(BTN_PIN, Pin.IN, Pin.PULL_UP)

stop_flag=False; _last_ms=0
def _btn_irq(p):
    global stop_flag,_last_ms
    now=ticks_ms()
    if p.value()==0 and ticks_diff(now,_last_ms)>200:
        stop_flag=True; _last_ms=now
btn.irq(trigger=Pin.IRQ_FALLING, handler=_btn_irq)

def blink_pattern(now,state):
    S_BOOT,S_READY,S_REC,S_STOP,S_ERR=0,1,2,3,4
    if state==S_BOOT:  return (now%1000)<200
    if state==S_READY: p=now%2000; return (0<=p<120) or (240<=p<360)
    if state==S_REC:   return True
    if state==S_STOP:  p=now%1000; return (0<=p<80) or (200<=p<280) or (400<=p<480)
    if state==S_ERR:   return (now%200)<100
    return False

# --- Filesystem ---
def next_name(prefix="bno", ext="bin"):
    n=0
    while True:
        name=f"/sd/{prefix}_{n:03d}.{ext}"
        try: uos.stat(name); n+=1
        except OSError: return name

def mount_sd():
    print("Mounting SD…")
    spi=SPI(1, baudrate=1_000_000, polarity=0, phase=0,
            sck=Pin(10), mosi=Pin(11), miso=Pin(12))
    sd=sdcard.SDCard(spi, Pin(CS_PIN, Pin.OUT, value=1))
    uos.mount(sd,"/sd"); spi.init(baudrate=SPI_BAUD)
    print("SD mounted."); return sd,spi
def umount_sd():
    try: uos.umount("/sd")
    except: pass

# --- BNO ---
i2c=I2C(0, sda=Pin(4), scl=Pin(5), freq=I2C_FREQ)
bno=BNO08X(i2c, debug=False)

# enable low-rate for bias
bno.enable_feature(BNO_REPORT_ACCELEROMETER, BIAS_RATE)
bno.enable_feature(BNO_REPORT_GRAVITY,      BIAS_RATE)

def still_bias(sec=BIAS_SECS):
    n=int(sec*BIAS_RATE); sx=sy=sz=0.0; dt=int(1000//BIAS_RATE)
    for k in range(n):
        ax,ay,az=bno.acc; gx,gy,gz=bno.gravity
        sx+=ax-gx; sy+=ay-gy; sz+=az-gz
        if (k&7)==0: gc_collect()
        sleep_ms(dt)
    return sx/n, sy/n, sz/n

def enable_logging_features():
    # Use BNO’s own linear acceleration to avoid extra I²C churn
    bno.enable_feature(BNO_REPORT_LINEAR_ACCELERATION, RATE_HZ)
    bno.enable_feature(BNO_REPORT_GAME_ROTATION_VECTOR, RATE_HZ)
    bno.set_quaternion_euler_vector(BNO_REPORT_GAME_ROTATION_VECTOR)

# --- Recording (reads ONLY 2 reports per sample) ---
def record_session(bias_xyz, fname):
    global stop_flag
    stop_flag=False
    bx,by,bz=bias_xyz

    f=open(fname,"wb")
    f.write(struct.pack("<IBHB", 0x424E4F31,1,RATE_HZ,0))

    buf_a=bytearray(REC_SIZE*BLOCK_RECS); buf_b=bytearray(REC_SIZE*BLOCK_RECS)
    active=buf_a; widx=0; total=0; blocks=0
    pack_into=struct.pack_into
    t0=ticks_ms(); t_last=t0; t_first=None; t_last_s=None

    print("Recording ->",fname,"| rate =",RATE_HZ,"Hz")
    led.value(1)

    try:
        while True:
            if stop_flag:
                print("Stop requested."); break

            now=ticks_ms()
            if t_first is None: t_first=now
            t_last_s=now
            t_ms=ticks_diff(now,t0)

            # ONE read of lin-acc + ONE read of quaternion
            lx,ly,lz=bno.acc_linear      # direct from sensor
            # subtract the small still-bias (helps DC offsets)
            lx-=bx; ly-=by; lz-=bz
            qi,qj,qk,qr=bno.quaternion

            pack_into(REC_FMT, active, widx*REC_SIZE, t_ms, lx,ly,lz, qi,qj,qk,qr)
            widx+=1; total+=1

            if widx==BLOCK_RECS:
                to_write=active
                active=buf_b if active is buf_a else buf_a
                widx=0
                t_w0=ticks_ms()
                f.write(to_write); blocks+=1
                led.value(0); sleep_ms(8); led.value(1)
                if FLUSH_EVERY_N and (blocks%FLUSH_EVERY_N==0):
                    f.flush(); print("[FLUSH] blocks=%d samples=%d" % (blocks,total))
                else:
                    print("[BLOCK] blocks=%d samples=%d wrote=%d bytes in %d ms"
                          % (blocks,total,len(to_write),ticks_diff(ticks_ms(),t_w0)))
                gc_collect()

#             if ticks_diff(now,t_last)>=STATUS_MS:
#                 print("[STAT] t=%.1fs count=%d (lx,ly,lz)=(%.3f,%.3f,%.3f)"
#                       % (ticks_diff(now,t0)/1000,total,lx,ly,lz))
#                 t_last=now

            rem=DT_MS - ticks_diff(ticks_ms(), now)
            if rem>0: sleep_ms(rem)
            if (total & 511)==0: gc_collect()

    finally:
        if widx: f.write(memoryview(active)[:widx*REC_SIZE]); blocks+=1
        try: f.flush(); led.value(0); sleep_ms(20); led.value(1); f.close()
        except: pass
        if t_first and t_last_s:
            dur=ticks_diff(t_last_s,t_first)
            eff=(total*1000.0/dur) if dur>0 else 0.0
            meta=fname.replace(".bin","_meta.txt")
            try:
                with open(meta,"w") as mf:
                    mf.write("Recording: %s\n"%fname)
                    mf.write("Target rate: %d Hz\n"%RATE_HZ)
                    mf.write("Effective rate: %.2f Hz\n"%eff)
                    mf.write("Total samples: %d\n"%total)
                    mf.write("Duration: %.2f s\n"%(dur/1000))
                    mf.write("Blocks: %d\n"%blocks)
                    mf.write("Block size: %d records (%d bytes)\n"%(BLOCK_RECS,REC_SIZE*BLOCK_RECS))
                print("Effective rate: %.2f Hz"%eff); print("Metadata:",meta)
            except Exception as e: print("Meta write failed:",e)
        print("Recording stopped. File closed:",fname)

# -------- MAIN --------
try:
    for _ in range(2): led.value(1); sleep_ms(60); led.value(0); sleep_ms(60)
    print_battery_status()

    print("Biasing… keep still.")
    t0=ticks_ms()
    while ticks_diff(ticks_ms(),t0)<(BIAS_SECS*1000):
        led.value(1 if blink_pattern(ticks_ms(),0) else 0); sleep_ms(10)
    bx,by,bz=still_bias()
    print("Bias ~ bx=%.4f by=%.4f bz=%.4f"%(bx,by,bz))

    enable_logging_features(); gc_collect()

    sd,spi=mount_sd()
    print("Ready. Press button to START.")
    while btn.value()==1:
        led.value(1 if blink_pattern(ticks_ms(),1) else 0); sleep_ms(20)
    while btn.value()==0: sleep_ms(10)  # release

    led.value(1)
    fname=next_name()
    record_session((bx,by,bz), fname)

    print("Unmounting SD…")
    umount_sd()
    t1=ticks_ms()
    while ticks_diff(ticks_ms(),t1)<800:
        led.value(1 if blink_pattern(ticks_ms(),3) else 0); sleep_ms(10)
    led.value(0); print("Done. Safe to power off.")

except Exception as e:
    print("ERROR:",e)
    while True:
        led.value(1 if blink_pattern(ticks_ms(),4) else 0); sleep_ms(20)