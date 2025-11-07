try:
    from machine import I2C, Pin, SPI, ADC, PWM
except ImportError:
    from machine import I2C, Pin, SPI, ADC
    PWM = None
from utime import ticks_ms, ticks_diff, sleep_ms, ticks_add, time
from array import array
from gc import collect as gc_collect
import gc
import sys
import uos, struct, sdcard
import ubinascii
from math import sqrt, cos, pi
from bno08x import *

try:
    from _thread import start_new_thread, allocate_lock
except ImportError:
    start_new_thread = None
    allocate_lock = None

# Force synchronous writer to isolate issues (disable threading temporarily)
start_new_thread = None

# -------- CONFIG --------
RATE_HZ       = 50              # sampling rate (stable target)
BIAS_RATE     = 25
BIAS_SECS     = 30
BLOCK_RECS    = 1024            # 32 KiB blocks (1024 * 32 B)
FLUSH_EVERY_N = 4               # checkpoint every N full blocks (0 = only at stop)
I2C_FREQ      = 400_000
SPI_BAUD      = 12_000_000      # reduced to add margin for flaky media/sockets
CS_PIN        = 13
LED_PIN       = 28
BTN_PIN       = 14
INT_PIN       = None
STATUS_MS     = 2000
# ------------------------

# --- Stop reasons (meta tags) ---
REASON_NONE     = 0
REASON_I2C_OS   = 1
REASON_SD_WRITE = 2
REASON_OTHER    = 3
REASON_ENOSPC   = 4  # No space left on device

# --- Robustness knobs ---
MAX_CONSEC_READ_ERRORS = 100   # ~2.0 s at 50 Hz before controlled stop
I2C_RETRY_SLEEP_MS     = 2     # brief backoff after transient I2C faults
MAX_BUFFERS            = 6     # cap on live buffers when running synchronously
QA_FLUSH_INTERVAL      = 6     # number of QA lines to batch before flushing
INT_WAIT_TIMEOUT_MS    = 8     # max wait for data-ready IRQ before falling back

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

# --- SD write resilience ---
SD_WRITE_RETRIES     = 3
SD_WRITE_BACKOFF_MS  = 10

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
S_BOOT, S_READY, S_REC, S_STOP, S_ERR = 0, 1, 2, 3, 4

_led_pin = Pin(LED_PIN, Pin.OUT, value=0)
if PWM is not None:
    try:
        led_pwm = PWM(_led_pin)
        led_pwm.freq(1000)
        HAVE_PWM_LED = True
    except Exception:
        led_pwm = None
        HAVE_PWM_LED = False
else:
    led_pwm = None
    HAVE_PWM_LED = False

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
    if state==S_BOOT:  return (now%1000)<200
    if state==S_READY: p=now%2000; return (0<=p<120) or (240<=p<360)
    if state==S_REC:   return True
    if state==S_STOP:  p=now%1000; return (0<=p<80) or (200<=p<280) or (400<=p<480)
    if state==S_ERR:   return (now%200)<100
    return False

def led_set(state, now_ms=None):
    if now_ms is None:
        now_ms = ticks_ms()
    if not HAVE_PWM_LED:
        _led_pin.value(1 if blink_pattern(now_ms, state) else 0)
        return
    if state == S_REC:
        period = 2400
        phase = (now_ms % period) / period
        amp = (1 - cos(2 * pi * phase)) * 0.5
        lo, hi = 0.08, 0.80
        level = lo + (hi - lo) * amp
        led_pwm.duty_u16(int(level * 65535))
        return
    if state == S_BOOT:
        level = 0.5 if (now_ms % 1000) < 200 else 0.02
    elif state == S_READY:
        p = now_ms % 2000
        level = 0.6 if (0 <= p < 120 or 240 <= p < 360) else 0.02
    elif state == S_STOP:
        p = now_ms % 1000
        level = 0.7 if (0 <= p < 80 or 200 <= p < 280 or 400 <= p < 480) else 0.02
    else:  # S_ERR or fallback
        level = 0.85 if (now_ms % 200) < 100 else 0.02
    led_pwm.duty_u16(int(level * 65535))

def led_off():
    if HAVE_PWM_LED:
        led_pwm.duty_u16(0)
    else:
        _led_pin.value(0)

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

# Write helper with small, bounded retries on transient SD errors
def _sd_write_with_retries(fh, mv):
    for _ in range(SD_WRITE_RETRIES):
        try:
            return fh.write(mv)
        except OSError as e:
            code = e.args[0] if getattr(e, "args", None) else None
            if code in (5, 110):  # EIO / ETIMEDOUT
                sleep_ms(SD_WRITE_BACKOFF_MS)
                continue
            raise
    # last try, let any exception propagate to caller
    return fh.write(mv)

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
        qa_file.write("t_ms,ax,ay,az,gx,gy,gz,qnorm,vsys_v\n")
    except Exception:
        qa_file = None
    qa_pending = []
    def flush_qa_buffer(force=False):
        nonlocal qa_pending, qa_file
        if not qa_pending:
            return
        if not qa_file:
            qa_pending.clear()
            return
        try:
            # MicroPython streams lack writelines(), so write line-by-line
            for line in qa_pending:
                qa_file.write(line)
            if force or len(qa_pending) >= QA_FLUSH_INTERVAL:
                qa_file.flush()
        except Exception:
            pass
        finally:
            qa_pending.clear()
    # SD free space snapshot for metadata
    sd_bs = sd_blocks = sd_bfree = sd_bavail = 0
    sd_total_bytes = sd_free_bytes = 0
    try:
        vfs = uos.statvfs("/sd")
        sd_bs = vfs[0] if len(vfs) > 0 and vfs[0] else 512
        sd_blocks = vfs[2] if len(vfs) > 2 else 0
        sd_bfree = vfs[3] if len(vfs) > 3 else 0
        sd_bavail = vfs[4] if len(vfs) > 4 else sd_bfree
        sd_total_bytes = sd_bs * sd_blocks
        sd_free_bytes = sd_bs * sd_bavail
    except Exception:
        pass

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


    # Recovery helper: try to remount SD and reopen current file in append mode
    def sd_recover_and_reopen():
        nonlocal f, fname, writer_error, error_reason, err_info, qa_file, recover_attempted, recover_success, recover_tail_mod
        # sd/spi live at module scope
        global sd, spi
        try:
            try:
                f.flush()
            except Exception:
                pass
            try:
                f.close()
            except Exception:
                pass
            # Close QA file before unmount/remount
            try:
                flush_qa_buffer()
                if qa_file:
                    qa_file.close()
            except Exception:
                pass
            try:
                umount_sd()
            except Exception:
                pass
            sleep_ms(200)
            try:
                sd, spi = mount_sd()
            except Exception:
                pass
            # Before appending, ensure file tail is record-aligned
            try:
                st = uos.stat(fname)
                tail = (st[6] - HEADER_SIZE) % REC_SIZE
            except Exception:
                tail = -1
            # track recovery context for .err details
            recover_attempted = 1
            recover_tail_mod = tail
            if tail != 0:
                # Unsafe to append to a dirty tail; bail out cleanly
                err_info = err_info or ("recover: dirty tail (bytes=%r), abort append" % (tail,))
                recover_success = 0
                return False
            try:
                f = open(fname, "ab")
                # Reopen QA CSV in append mode as well
                try:
                    qa_file = open(qa_path, "a")
                except Exception:
                    qa_file = None
                recover_success = 1
                return True
            except Exception:
                recover_success = 0
                return False
        except Exception:
            return False

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
    consec_read_errors = 0
    max_consec_read_errors = 0
    error_reason = REASON_NONE
    err_info = None
    # Recovery state context for .err diagnostics
    recover_attempted = 0
    recover_success = 0
    recover_tail_mod = -1
    buf_inflight_max = 0

    def update_buf_inflight():
        nonlocal buf_inflight_max
        in_use = len(buffers) - len(free_indices)
        if in_use > buf_inflight_max:
            buf_inflight_max = in_use
    update_buf_inflight()

    def release_buffer(idx):
        if have_thread:
            free_lock.acquire()
            free_indices.append(idx)
            free_lock.release()
        else:
            free_indices.append(idx)

    def take_buffer():
        global stop_flag
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
                    idx = free_indices.pop()
                    update_buf_inflight()
                    return idx
                # No free buffers; cap allocations to avoid runaway growth
                if len(buffers) >= MAX_BUFFERS:
                    for _ in range(20):
                        if free_indices or stop_flag:
                            break
                        sleep_ms(1)
                    if free_indices:
                        idx = free_indices.pop()
                        update_buf_inflight()
                        return idx
                    print("Buffer cap reached; stopping.")
                    stop_flag = True
                    return None
                buf = bytearray(block_bytes)
                buffers.append(buf)
                update_buf_inflight()
                return len(buffers) - 1

    def enqueue_block(idx, length, flush_flag):
        nonlocal q_tail, q_count, q_full_hits, q_high_water, total_write_bytes, max_write_ms, checkpoint_count
        nonlocal writer_error, error_reason, err_info, f
        global stop_flag
        if length <= 0:
            return
        if writer_error:
            if not stop_flag:
                stop_flag = True
            release_buffer(idx)
            return
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
                    release_buffer(idx)
                    return
                sleep_ms(1)
        else:
            mv = memoryview(buffers[idx])[:length]
            t0w = ticks_ms()
            try:
                _sd_write_with_retries(f, mv)
            except OSError as err:
                code = err.args[0] if getattr(err, "args", None) else None
                if code in (5, 110):
                    # Attempt a basic recovery: remount SD and reopen current file, then retry once
                    if sd_recover_and_reopen():
                        t0w = ticks_ms()
                        try:
                            _sd_write_with_retries(f, mv)
                        except Exception as err2:
                            writer_error = err2
                            if error_reason == REASON_NONE:
                                error_reason = REASON_SD_WRITE
                            err_info = err_info or "enqueue_block write (after recover): %r" % (err2,)
                            stop_flag = True
                            release_buffer(idx)
                            return
                    else:
                        writer_error = err
                        if error_reason == REASON_NONE:
                            error_reason = REASON_SD_WRITE
                        err_info = err_info or "enqueue_block write: %r" % (err,)
                        stop_flag = True
                        release_buffer(idx)
                        return
                elif code == 28:  # ENOSPC
                    writer_error = err
                    if error_reason == REASON_NONE:
                        error_reason = REASON_ENOSPC
                    err_info = err_info or "enqueue_block write: ENOSPC"
                    stop_flag = True
                    release_buffer(idx)
                    return
                else:
                    writer_error = err
                    if error_reason == REASON_NONE:
                        error_reason = REASON_SD_WRITE
                    err_info = err_info or "enqueue_block write: %r" % (err,)
                    stop_flag = True
                    release_buffer(idx)
                    return
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
                except Exception as err:
                    writer_error = err
                    code = err.args[0] if getattr(err, "args", None) else None
                    if error_reason == REASON_NONE:
                        error_reason = REASON_ENOSPC if code == 28 else REASON_SD_WRITE
                    err_info = err_info or ("enqueue_block flush: %s" % ("ENOSPC" if code == 28 else repr(err)))
                    stop_flag = True
                    release_buffer(idx)
                    return
            release_buffer(idx)

    def writer_loop():
        nonlocal q_head, q_count, writer_run, writer_done, total_write_bytes, max_write_ms, writer_error, checkpoint_count
        nonlocal error_reason, err_info, f
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
                    _sd_write_with_retries(f, mv)
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
            code = None
            try:
                code = err.args[0] if getattr(err, "args", None) else None
            except Exception:
                code = None
            if error_reason == REASON_NONE:
                error_reason = REASON_ENOSPC if code == 28 else REASON_SD_WRITE
            err_info = err_info or ("writer_loop: %s" % ("ENOSPC" if code == 28 else repr(err)))
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
    led_set(S_REC, ticks_ms())

    try:
        # Use combined driver read (bounded/non-alloc in driver) per tick if available
        combined_read = hasattr(bno, "read_combined_into")
        next_t = ticks_add(ticks_ms(), DT_MS)
        while True:
            if stop_flag:
                print("Stop requested."); break

            if int_pin is not None:
                if not data_ready_flag:
                    wait_deadline = ticks_add(ticks_ms(), INT_WAIT_TIMEOUT_MS)
                    while (not data_ready_flag) and (ticks_diff(wait_deadline, ticks_ms()) > 0) and not stop_flag:
                        sleep_ms(1)
                if data_ready_flag:
                    data_ready_flag = False

            if writer_error:
                print("Writer error; stopping safely.")
                if error_reason == REASON_NONE:
                    error_reason = REASON_SD_WRITE
                stop_flag = True
                continue

            now = ticks_ms()
            led_set(S_REC, now)
            if t_first is None:
                t_first = now
            t_last_s = now
            t_ms = ticks_diff(now, t0)

            # Bounded, resilient reads; reuse last values on transient MemoryError
            read_ok = True
            last_exception_was_os = False
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
            except OSError:
                read_ok = False
                last_exception_was_os = True
                consec_read_errors += 1
                if consec_read_errors > max_consec_read_errors:
                    max_consec_read_errors = consec_read_errors
                sleep_ms(I2C_RETRY_SLEEP_MS)
            else:
                consec_read_errors = 0

            if not read_ok:
                missing_slots += 1
                lx, ly, lz = last_lx, last_ly, last_lz
                qi, qj, qk, qr = last_qi, last_qj, last_qk, last_qr
                if last_exception_was_os and consec_read_errors >= MAX_CONSEC_READ_ERRORS:
                    print("I2C error persisted; stopping safely.")
                    if error_reason == REASON_NONE:
                        error_reason = REASON_I2C_OS
                    stop_flag = True
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
                vsys_now, _ = read_vsys_voltage()
                qa_sample_count += 1
                if qa_file:
                    line = "%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.3f\n" % (t_ms, ax_q, ay_q, az_q, gx_q, gy_q, gz_q, qnorm, vsys_now)
                qa_pending.append(line)
                if len(qa_pending) >= QA_FLUSH_INTERVAL:
                    flush_qa_buffer()

            if widx == BLOCK_RECS:
                blocks += 1
                flush_flag = bool(FLUSH_EVERY_N) and ((blocks % FLUSH_EVERY_N) == 0)
                enqueue_block(active_idx, block_bytes, flush_flag)
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

    except Exception as e:
        if error_reason == REASON_NONE:
            error_reason = REASON_OTHER
        try:
            err_code = e.args[0] if getattr(e, "args", None) else None
        except Exception:
            err_code = None
        err_info = err_info or "Exception: %r errno=%r" % (e, err_code)
        stop_flag = True
    finally:
        try:
            if widx:
                blocks += 1
                enqueue_block(active_idx, widx * REC_SIZE, True)
            if have_thread:
                writer_run = False
                for _ in range(3000):  # ~3 s grace
                    if writer_done:
                        break
                    sleep_ms(1)
                if not writer_done:
                    print("Writer thread join timeout.")
            else:
                # ensure final flush if running synchronously
                pass
        except Exception as e:
            print("Writer cleanup error:", e)
        gc_collect()
        try:
            f.flush(); led_off(); sleep_ms(20); led_set(S_REC, ticks_ms()); f.close()
        except Exception:
            pass
        try:
            flush_qa_buffer(force=True)
            if qa_file:
                qa_file.close()
        except Exception:
            pass
        if qa_sample_count:
            print("QA samples:", qa_path, "count:", qa_sample_count)
        else:
            print("QA samples: none logged")
        try:
            if writer_error is not None or err_info is not None:
                err_path = fname.replace(".bin", ".err")
                with open(err_path, "w") as ef:
                    ef.write("blocks=%d total_write_bytes=%d\n" % (blocks, total_write_bytes))
                    if writer_error is not None:
                        ef.write("Writer error (SD path):\n")
                        try:
                            sys.print_exception(writer_error, ef)
                        except Exception:
                            ef.write(repr(writer_error) + "\n")
                        code = writer_error.args[0] if getattr(writer_error, "args", None) else None
                        ef.write("errno=%r\n" % (code,))
                    if err_info:
                        ef.write(err_info + "\n")
                    # Recovery/tail diagnostic context
                    try:
                        st_now = uos.stat(fname)
                        cur_tail = (st_now[6] - HEADER_SIZE) % REC_SIZE
                    except Exception:
                        cur_tail = -1
                    try:
                        ef.write("recovery_attempted=%d recovery_success=%d recovery_tail_mod=%d current_tail_mod=%d\n" % (
                            recover_attempted, recover_success, recover_tail_mod, cur_tail))
                    except Exception:
                        pass
        except Exception:
            pass
        recs_from_size = 0
        try:
            st = uos.stat(fname)
            recs_from_size = max(0, (st[6] - HEADER_SIZE) // REC_SIZE)
        except Exception:
            pass
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
                    mf.write("Buffer in-flight max: %d\n" % buf_inflight_max)
                    mf.write("Writer max write ms: %d\n" % max_write_ms)
                    mf.write("Writer total bytes: %d\n" % total_write_bytes)
                    mf.write("Checkpoints (flush+sync): %d\n" % checkpoint_count)
                    mf.write("SD block size: %d\n" % sd_bs)
                    mf.write("SD total blocks: %d\n" % sd_blocks)
                    mf.write("SD free blocks (start): %d\n" % sd_bfree)
                    mf.write("SD avail blocks (start): %d\n" % sd_bavail)
                    mf.write("SD total bytes: %d\n" % sd_total_bytes)
                    mf.write("SD free bytes (start): %d\n" % sd_free_bytes)
                    # Capture end-of-session SD space as well
                    try:
                        vfs_end = uos.statvfs("/sd")
                        sd_end_bfree = vfs_end[3] if len(vfs_end) > 3 else 0
                        sd_end_bavail = vfs_end[4] if len(vfs_end) > 4 else sd_end_bfree
                        sd_end_free_bytes = sd_bs * sd_end_bavail
                        mf.write("SD free blocks (end): %d\n" % sd_end_bfree)
                        mf.write("SD avail blocks (end): %d\n" % sd_end_bavail)
                        mf.write("SD free bytes (end): %d\n" % sd_end_free_bytes)
                    except Exception:
                        pass
                    reason_text = {
                        REASON_NONE: "none",
                        REASON_I2C_OS: "I2C_OSError",
                        REASON_SD_WRITE: "SD_write_error",
                        REASON_ENOSPC: "ENOSPC",
                        REASON_OTHER: "other",
                    }.get(error_reason, "other")
                    mf.write("Stop reason: %d (%s)\n" % (error_reason, reason_text))
                    mf.write("Max consecutive read errors: %d\n" % max_consec_read_errors)
                    mf.write("Samples by file size (may include partial last record): %d\n" % recs_from_size)
                    if total and recs_from_size and (recs_from_size != total):
                        mf.write("Header vs size mismatch: header=%d size=%d\n" % (total, recs_from_size))
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
    for _ in range(2):
        led_set(S_BOOT, ticks_ms()); sleep_ms(60)
        led_off(); sleep_ms(60)
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
        led_set(S_BOOT, ticks_ms()); sleep_ms(10)
    bx,by,bz=still_bias()
    print("Bias ~ bx=%.4f by=%.4f bz=%.4f"%(bx,by,bz))

    enable_logging_features(); gc_collect()

    sd,spi=mount_sd()
    print("Ready. Press button to START.")
    while btn.value()==1:
        led_set(S_READY, ticks_ms()); sleep_ms(20)
    while btn.value()==0: sleep_ms(10)  # release

    led_set(S_REC, ticks_ms())
    fname=next_name()
    record_session((bx,by,bz), fname)

    print("Unmounting SD…")
    umount_sd()
    t1=ticks_ms()
    while ticks_diff(ticks_ms(),t1)<800:
        led_set(S_STOP, ticks_ms()); sleep_ms(10)
    led_off(); print("Done. Safe to power off.")

except Exception as e:
    sys.print_exception(e)
    print("ERROR:",repr(e))
    while True:
        led_set(S_ERR, ticks_ms()); sleep_ms(20)
