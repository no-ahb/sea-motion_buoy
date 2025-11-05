from machine import I2C, Pin, SPI, ADC
from utime import ticks_ms, ticks_diff, sleep_ms, ticks_add, time
from array import array
from gc import collect as gc_collect
import gc
import sys
import uos, struct, sdcard
import ubinascii
from math import sqrt
from bno08x import *

try:
    from _thread import start_new_thread, allocate_lock
except ImportError:
    start_new_thread = None
    allocate_lock = None

# -------- CONFIG --------
RATE_HZ       = 50              # sampling rate (stable target)
BIAS_RATE     = 25
BIAS_SECS     = 30
BLOCK_RECS    = 1024            # 32 KiB blocks (1024 * 32 B)
FLUSH_EVERY_N = 1               # checkpoint every N full blocks (0 = only at stop)
I2C_FREQ      = 400_000
SPI_BAUD      = 24_000_000
CS_PIN        = 13
LED_PIN       = 28
BTN_PIN       = 14
INT_PIN       = None
STATUS_MS     = 2000
# ------------------------

DT_MS    = int(1000 // RATE_HZ)
REC_FMT  = "<Ifffffff"         # t_ms, lx,ly,lz, qi,qj,qk,qr
REC_SIZE = struct.calcsize(REC_FMT)

HEADER_MAGIC   = b"BNO2"
HEADER_VERSION = 2
HEADER_FMT     = "<4sBBHHIIIfffII"
HEADER_SIZE    = struct.calcsize(HEADER_FMT)
HEADER_FLAGS_LINEAR   = 0x01
HEADER_FLAGS_GAME_RV  = 0x02
HEADER_FLAGS_COMBINED = 0x04
DEVICE_ID      = 0x42554F59  # 'BUOY'

QA_SAMPLE_INTERVAL_S   = 5
QA_SAMPLE_INTERVAL_REC = max(1, int(RATE_HZ * QA_SAMPLE_INTERVAL_S))
QA_MAX_SAMPLES         = 1024
QA_FEATURE_RATE        = min(25, max(5, RATE_HZ))

# --- Power (optional) ---
VBUS_SENSE = Pin(24, Pin.IN)
VSYS_ADC   = ADC(29); VREF=3.3; DIV=3.0

def read_vsys_voltage():
    raw = VSYS_ADC.read_u16()
    return (raw / 65535.0) * VREF * DIV, raw

def print_battery_status():
    vsys, raw = read_vsys_voltage()
    src = "USB" if VBUS_SENSE.value() else "Battery"
    print("=" * 40)
    print("Power:", src, "VSYS=%.2f V (ADC=%d)" % (vsys, raw))
    print("=" * 40)

# --- LED / Button (IRQ-latched stop) ---
led = Pin(LED_PIN, Pin.OUT, value=0)
btn = Pin(BTN_PIN, Pin.IN, Pin.PULL_UP)
int_pin = Pin(INT_PIN, Pin.IN, Pin.PULL_UP) if INT_PIN is not None else None

stop_flag=False; _last_ms=0
def _btn_irq(p):
    global stop_flag,_last_ms
    now=ticks_ms()
    if p.value()==0 and ticks_diff(now,_last_ms)>200:
        stop_flag=True; _last_ms=now
btn.irq(trigger=Pin.IRQ_FALLING, handler=_btn_irq)

data_ready_flag = False
if int_pin is not None:
    def _int_irq(p):
        global data_ready_flag
        data_ready_flag = True
    int_pin.irq(trigger=Pin.IRQ_FALLING, handler=_int_irq)

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
i2c = None
bno = None

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
    bno.enable_feature(BNO_REPORT_ACCELEROMETER, QA_FEATURE_RATE)
    bno.enable_feature(BNO_REPORT_GYROSCOPE, QA_FEATURE_RATE)
    bno.set_quaternion_euler_vector(BNO_REPORT_GAME_ROTATION_VECTOR)

# --- Recording (reads ONLY 2 reports per sample) ---
def record_session(bias_xyz, fname):
    global stop_flag, data_ready_flag
    stop_flag=False
    data_ready_flag=False
    bx,by,bz=bias_xyz

    f=open(fname,"wb")
    session_epoch = int(time())
    session_ticks = ticks_ms()
    header_flags = HEADER_FLAGS_LINEAR | HEADER_FLAGS_GAME_RV | HEADER_FLAGS_COMBINED
    header_values = [HEADER_MAGIC, HEADER_VERSION, HEADER_SIZE, RATE_HZ, header_flags,
                     session_epoch, session_ticks, DEVICE_ID, bx, by, bz, 0, 0]
    f.write(struct.pack(HEADER_FMT, *header_values))
    qa_path = fname.replace(".bin", "_qa.csv")
    qa_file = None
    try:
        qa_file = open(qa_path, "w")
        qa_file.write("t_ms,ax,ay,az,gx,gy,gz,qnorm\n")
    except Exception:
        qa_file = None

    block_bytes = REC_SIZE * BLOCK_RECS
    have_thread = bool(start_new_thread and allocate_lock)
    buf_count = 6 if have_thread else 2
    queue_capacity = (buf_count - 1) if have_thread and buf_count > 1 else buf_count
    buffers = [bytearray(block_bytes) for _ in range(buf_count)]
    active_idx = 0
    active_mv = memoryview(buffers[active_idx])
    widx = 0; total = 0; blocks = 0

    free_indices = [i for i in range(1, buf_count)]
    q = [None] * buf_count if have_thread else None
    q_head = q_tail = q_count = 0
    q_lock = allocate_lock() if have_thread else None
    free_lock = allocate_lock() if have_thread else None
    writer_run = have_thread
    writer_done = not have_thread
    writer_error = None

    total_write_bytes = 0
    max_write_ms = 0
    q_full_hits = 0
    q_high_water = 0
    checkpoint_count = 0
    prev_t_ms = None
    late20 = 0
    late50 = 0
    missing_slots = 0
    # last-known-good values for resilient sampling on transient driver hiccups
    last_lx = last_ly = last_lz = 0.0
    last_qi = last_qj = last_qk = 0.0
    last_qr = 1.0
    # combined-read scratch buffers (avoid two driver calls per tick)
    acc3 = array('f', [0.0, 0.0, 0.0])
    quat4 = array('f', [0.0, 0.0, 0.0, 1.0])
    crc32_accum = 0
    qa_sample_count = 0
    qa_counter = QA_SAMPLE_INTERVAL_REC

    def release_buffer(idx):
        if have_thread:
            free_lock.acquire()
            free_indices.append(idx)
            free_lock.release()
        else:
            free_indices.append(idx)

    def take_buffer():
        while True:
            if have_thread:
                free_lock.acquire()
                if free_indices:
                    idx = free_indices.pop()
                    free_lock.release()
                    return idx
                free_lock.release()
                if stop_flag:
                    return None
                sleep_ms(1)
            else:
                if free_indices:
                    return free_indices.pop()
                buf = bytearray(block_bytes)
                buffers.append(buf)
                return len(buffers) - 1

    def enqueue_block(idx, length, flush_flag):
        nonlocal q_tail, q_count, q_full_hits, q_high_water, total_write_bytes, max_write_ms
        if length <= 0:
            return
        if writer_error:
            raise writer_error
        if have_thread:
            while True:
                q_lock.acquire()
                if q_count < queue_capacity:
                    queue_depth = q_count + 1
                    queue_item = (idx, length, flush_flag)
                    q[q_tail] = queue_item
                    q_tail = (q_tail + 1) % buf_count
                    q_count = queue_depth
                    if queue_depth > q_high_water:
                        q_high_water = queue_depth
                    q_lock.release()
                    break
                q_lock.release()
                q_full_hits += 1
                if stop_flag:
                    return
                sleep_ms(1)
        else:
            mv = memoryview(buffers[idx])[:length]
            t0w = ticks_ms()
            f.write(mv)
            write_ms = ticks_diff(ticks_ms(), t0w)
            if write_ms > max_write_ms:
                max_write_ms = write_ms
            total_write_bytes += length
            if flush_flag:
                try:
                    f.flush()
                    try:
                        uos.sync()
                    except Exception:
                        pass
                    checkpoint_count += 1
                except Exception:
                    pass
            release_buffer(idx)

    def writer_loop():
        nonlocal q_head, q_count, writer_run, writer_done, total_write_bytes, max_write_ms, writer_error, checkpoint_count
        # NOTE: this closure uses the outer 'f'; never rebind 'f' inside.
        try:
            while True:
                entry = None
                q_lock.acquire()
                if q_count:
                    entry = q[q_head]
                    q[q_head] = None
                    q_head = (q_head + 1) % buf_count
                    q_count -= 1
                running = writer_run or (q_count > 0)
                q_lock.release()
                if entry:
                    buf_idx, length, flush_flag = entry
                    mv = memoryview(buffers[buf_idx])[:length]
                    t0w = ticks_ms()
                    f.write(mv)
                    write_ms = ticks_diff(ticks_ms(), t0w)
                    if write_ms > max_write_ms:
                        max_write_ms = write_ms
                    total_write_bytes += length
                    if flush_flag:
                        try:
                            f.flush()
                            try:
                                uos.sync()
                            except Exception:
                                pass
                            checkpoint_count += 1
                        except Exception:
                            pass
                    release_buffer(buf_idx)
                    continue
                if not running:
                    break
                sleep_ms(1)
        except Exception as err:
            writer_error = err
        finally:
            writer_done = True

    if have_thread:
        start_new_thread(writer_loop, ())

    pack_into = struct.pack_into
    t0 = ticks_ms(); t_first = None; t_last_s = None
    # Low-voltage failsafe (debounced)
    LV_THRESH = 3.40
    LV_CHECK_MS = 3000
    lv_count = 0
    min_vsys, _ = read_vsys_voltage()
    low_voltage_stop = 0
    next_lv_check = ticks_add(t0, LV_CHECK_MS)

    print("Recording ->",fname,"| rate =",RATE_HZ,"Hz")
    led.value(1)

    try:
        # Use combined driver read (bounded/non-alloc in driver) per tick if available
        combined_read = hasattr(bno, "read_combined_into")
        next_t = ticks_add(ticks_ms(), DT_MS)
        while True:
            if stop_flag:
                print("Stop requested."); break

            if int_pin is not None and data_ready_flag:
                data_ready_flag = False

            if writer_error:
                raise writer_error

            now = ticks_ms()
            if t_first is None:
                t_first = now
            t_last_s = now
            t_ms = ticks_diff(now, t0)

            # Bounded, resilient reads; reuse last values on transient MemoryError
            read_ok = True
            try:
                if combined_read:
                    bno.read_combined_into(acc3, quat4)
                    lx, ly, lz = acc3[0], acc3[1], acc3[2]
                    qi, qj, qk, qr = quat4[0], quat4[1], quat4[2], quat4[3]
                else:
                    lx, ly, lz = bno.acc_linear
                    qi, qj, qk, qr = bno.quaternion
            except (MemoryError, RuntimeError):
                read_ok = False

            if not read_ok:
                missing_slots += 1
                lx, ly, lz = last_lx, last_ly, last_lz
                qi, qj, qk, qr = last_qi, last_qj, last_qk, last_qr
            else:
                last_lx, last_ly, last_lz = lx, ly, lz
                last_qi, last_qj, last_qk, last_qr = qi, qj, qk, qr

            lx -= bx; ly -= by; lz -= bz

            if prev_t_ms is not None:
                dt = t_ms - prev_t_ms
                if dt > DT_MS:
                    missing_slots += max(0, (dt // DT_MS) - 1)
                if dt > 20:
                    late20 += 1
                if dt > 50:
                    late50 += 1
            prev_t_ms = t_ms

            # Low-voltage periodic check (debounced)
            if ticks_diff(now, next_lv_check) >= 0:
                vsys, _raw = read_vsys_voltage()
                if vsys < min_vsys:
                    min_vsys = vsys
                if vsys < LV_THRESH:
                    lv_count += 1
                else:
                    lv_count = 0
                if lv_count >= 3 and not low_voltage_stop:
                    low_voltage_stop = 1
                    stop_flag = True
                next_lv_check = ticks_add(now, LV_CHECK_MS)

            offset = widx * REC_SIZE
            pack_into(REC_FMT, buffers[active_idx], offset, t_ms, lx, ly, lz, qi, qj, qk, qr)
            crc32_accum = ubinascii.crc32(active_mv[offset:offset + REC_SIZE], crc32_accum) & 0xFFFFFFFF
            widx += 1; total += 1

            qa_counter -= 1
            if qa_counter <= 0:
                qa_counter = QA_SAMPLE_INTERVAL_REC
                try:
                    ax_q, ay_q, az_q = bno.acc
                except Exception:
                    ax_q = ay_q = az_q = 0.0
                try:
                    gx_q, gy_q, gz_q = bno.gyro
                except Exception:
                    gx_q = gy_q = gz_q = 0.0
                qnorm = sqrt((qi * qi) + (qj * qj) + (qk * qk) + (qr * qr))
                qa_sample_count += 1
                if qa_file:
                    try:
                        qa_file.write("%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n" % (t_ms, ax_q, ay_q, az_q, gx_q, gy_q, gz_q, qnorm))
                        qa_file.flush()
                    except Exception:
                        pass

            if widx == BLOCK_RECS:
                blocks += 1
                flush_flag = bool(FLUSH_EVERY_N) and ((blocks % FLUSH_EVERY_N) == 0)
                enqueue_block(active_idx, block_bytes, flush_flag)
                try:
                    led.toggle()
                except AttributeError:
                    led.value(0 if led.value() else 1)
                next_idx = take_buffer()
                if next_idx is None:
                    print("Buffer exhaustion; stopping.")
                    break
                active_idx = next_idx
                active_mv = memoryview(buffers[active_idx])
                widx = 0
                gc_collect()

            next_t = ticks_add(next_t, DT_MS)
            d = ticks_diff(next_t, ticks_ms())
            if d > 0:
                sleep_ms(d)
            if (total & 511) == 0:
                gc_collect()

    finally:
        try:
            if widx:
                blocks += 1
                enqueue_block(active_idx, widx * REC_SIZE, True)
            if have_thread:
                writer_run = False
                while not writer_done:
                    sleep_ms(1)
                if writer_error:
                    raise writer_error
            else:
                # ensure final flush if running synchronously
                pass
        except Exception as e:
            print("Writer cleanup error:", e)
        gc_collect()
        try:
            f.flush(); led.value(0); sleep_ms(20); led.value(1); f.close()
        except Exception:
            pass
        try:
            qa_file.flush(); qa_file.close()
        except Exception:
            pass
        if qa_sample_count:
            print("QA samples:", qa_path, "count:", qa_sample_count)
        else:
            print("QA samples: none logged")
        try:
            header_values[-2] = total & 0xFFFFFFFF
            header_values[-1] = crc32_accum & 0xFFFFFFFF
            with open(fname, "r+b") as hf:
                hf.seek(0)
                hf.write(struct.pack(HEADER_FMT, *header_values))
        except Exception as e:
            print("Header rewrite failed:", e)
        if t_first and t_last_s:
            dur = ticks_diff(t_last_s, t_first)
            eff = ((total - 1) * 1000.0 / dur) if (dur > 0 and total > 1) else 0.0
            meta = fname.replace(".bin","_meta.txt")
            try:
                with open(meta,"w") as mf:
                    mf.write("Recording: %s\n" % fname)
                    mf.write("Target rate: %d Hz\n" % RATE_HZ)
                    mf.write("Header version: %d\n" % HEADER_VERSION)
                    mf.write("Start epoch (s): %d\n" % session_epoch)
                    mf.write("Start ticks (ms): %d\n" % session_ticks)
                    mf.write("Device ID: 0x%08X\n" % DEVICE_ID)
                    mf.write("CRC32 (records): 0x%08X\n" % (crc32_accum & 0xFFFFFFFF))
                    mf.write("Bias offsets (m/s^2): %.6f, %.6f, %.6f\n" % (bx, by, bz))
                    mf.write("Effective rate: %.6f Hz\n" % eff)
                    mf.write("Total samples: %d\n" % total)
                    mf.write("Duration: %.2f s\n" % (dur/1000))
                    mf.write("Blocks: %d\n" % blocks)
                    mf.write("Block size: %d records (%d bytes)\n" % (BLOCK_RECS, REC_SIZE*BLOCK_RECS))
                    mf.write("Late >20 ms gaps: %d\n" % late20)
                    mf.write("Late >50 ms gaps: %d\n" % late50)
                    mf.write("Missed slots (DT_MS=%d ms): %d\n" % (DT_MS, missing_slots))
                    mf.write("Queue high-water: %d\n" % q_high_water)
                    mf.write("Queue full hits: %d\n" % q_full_hits)
                    mf.write("Writer max write ms: %d\n" % max_write_ms)
                    mf.write("Writer total bytes: %d\n" % total_write_bytes)
                    mf.write("Checkpoints (flush+sync): %d\n" % checkpoint_count)
                    mf.write("Low voltage stop: %d (thresh=%.2f V, min_vsys=%.2f V)\n" % (low_voltage_stop, LV_THRESH, min_vsys))
                    mf.write("QA sample count: %d (every %d samples)\n" % (qa_sample_count, QA_SAMPLE_INTERVAL_REC))
                    if qa_sample_count:
                        mf.write("QA CSV: %s\n" % qa_path)
                print("Effective rate: %.6f Hz" % eff); print("Metadata:", meta)
            except Exception as e:
                print("Meta write failed:", e)
        print("Recording stopped. File closed:", fname)

# -------- MAIN --------
try:
    for _ in range(2): led.value(1); sleep_ms(60); led.value(0); sleep_ms(60)
    print_battery_status()

    print("Initializing I2C…")
    if i2c is None:
        i2c = I2C(0, sda=Pin(4), scl=Pin(5), freq=I2C_FREQ)

    print("Initializing BNO085…")
    bno = BNO08X(i2c, debug=False)
    bno.enable_feature(BNO_REPORT_ACCELEROMETER, BIAS_RATE)
    bno.enable_feature(BNO_REPORT_GRAVITY,      BIAS_RATE)

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
    sys.print_exception(e)
    print("ERROR:",repr(e))
    while True:
        led.value(1 if blink_pattern(ticks_ms(),4) else 0); sleep_ms(20)
