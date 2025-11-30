#!/usr/bin/env python3
import os
import time
import json
import queue
import threading
from datetime import datetime

import sounddevice as sd
from vosk import Model, KaldiRecognizer

import RPi.GPIO as GPIO
import serial
import pynmea2

# ---------------- CONFIG ----------------
MODEL_PATH = "/home/raspberrypi/models/vosk-model-small-en-us-0.15"
SAMPLE_RATE = 16000
CHANNELS = 1
AUDIO_BLOCKSIZE = 4000

# GPIO pins (BCM numbering)
SERVO_PIN = 18
KILL_PIN  = 17
LED_PIN   = 27

# Servo / Throttle mapping
SERVO_FREQ = 50.0
SERVO_MIN_DUTY = 3.0
SERVO_MAX_DUTY = 11.0
THROTTLE_MIN = 0.0
THROTTLE_MAX = 100.0

THROTTLE_STEP = 10.0
SAFE_THROTTLE = 0.0
WATCHDOG_TIMEOUT = 30.0

# Accepted commands
GRAMMAR_PHRASES = [
    "increase", "decrease", "stop",
    "speed one", "speed 1", "speed two", "speed 2", "speed to", 
    "speed three", "speed 3",
    "cruise one", "cruise two", "cruise three",
    "mode one", "mode two", "mode three",
    "kill", "brake"
]

# Mode → demo speeds
MODE_SPEEDS = {"one": 30.0, "two": 50.0, "three": 70.0}
THROTTLE_PER_KMPH = 1.2

# GPS
GPS_SERIAL = "/dev/serial0"
GPS_BAUD = 9600
GPS_LOG_DIR = "/home/raspberrypi/gps_logs"
GPS_LOG_INTERVAL = 30.0

# External throttle (demo pattern)
EXTERNAL_UPDATE_INTERVAL = 4.0
EXTERNAL_THROTTLE_PATTERN = [
    36.0, 36.0, 42.5, 49.0, 55.5, 62.0, 68.5, 75.0,
    75.0, 75.0, 65.8, 56.7, 47.5, 38.3, 29.2, 20.0
]

# ---------------- GLOBALS ----------------
q = queue.Queue()
running = True
kill_triggered = threading.Event()

current_throttle = 0.0
throttle_lock = threading.Lock()

gps_fix = None
gps_lock = threading.Lock()
gps_log_file = None
gps_log_lock = threading.Lock()

last_command_time = time.time()

external_throttle = None
external_lock = threading.Lock()
external_index = 0

voice_override_until = 0.0
voice_override_lock = threading.Lock()

# ---------------- GPIO setup ----------------
GPIO.setmode(GPIO.BCM)
GPIO.setup(SERVO_PIN, GPIO.OUT)
GPIO.setup(KILL_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(LED_PIN, GPIO.OUT)

pwm = GPIO.PWM(SERVO_PIN, SERVO_FREQ)
pwm.start(0.0)

def duty_from_throttle(percent):
    p = max(THROTTLE_MIN, min(THROTTLE_MAX, float(percent)))
    return SERVO_MIN_DUTY + (p / 100.0) * (SERVO_MAX_DUTY - SERVO_MIN_DUTY)

def set_throttle(percent):
    global current_throttle
    with throttle_lock:
        current_throttle = max(THROTTLE_MIN, min(THROTTLE_MAX, float(percent)))
        duty = duty_from_throttle(current_throttle)
        try:
            pwm.ChangeDutyCycle(duty)
        except Exception as e:
            print("[ACT] PWM error:", e)
    print(f"[ACT] throttle {current_throttle:.1f}% -> duty {duty:.2f}")

# ---------------- Kill switch ----------------
def kill_callback(channel):
    if not kill_triggered.is_set():
        print("[KILL] callback detected")
        kill_triggered.set()

GPIO.add_event_detect(KILL_PIN, GPIO.FALLING, callback=kill_callback, bouncetime=200)

def kill_watchdog():
    global running
    while running and not kill_triggered.is_set():
        try:
            if GPIO.input(KILL_PIN) == GPIO.LOW:
                print("[KILL] watchdog triggered")
                set_throttle(SAFE_THROTTLE)
                GPIO.output(LED_PIN, GPIO.HIGH)
                kill_triggered.set()
                running = False
                break
        except Exception as e:
            print("[KILL] watchdog read error:", e)
        time.sleep(0.1)
    print("[KILL] watchdog exiting")

# ---------------- Speech Recognition ----------------
def audio_callback(indata, frames, time_info, status):
    if status:
        print("[AUDIO] status", status)
    q.put(bytes(indata))

def start_stt():
    if not os.path.exists(MODEL_PATH):
        print("[STT] model missing:", MODEL_PATH)
        return None
    model = Model(MODEL_PATH)
    rec = KaldiRecognizer(model, SAMPLE_RATE, json.dumps(GRAMMAR_PHRASES))
    rec.SetWords(False)
    print("[STT] recognizer ready")

    def stt_loop():
        global last_command_time, running
        while running and not kill_triggered.is_set():
            try:
                data = q.get(timeout=1)
            except queue.Empty:
                continue
            if rec.AcceptWaveform(data):
                j = json.loads(rec.Result())
                text = j.get("text", "").strip().lower()
                if text:
                    print("[STT] ->", text)
                    last_command_time = time.time()
                    handle_text(text)
        print("[STT] exiting")

    t = threading.Thread(target=stt_loop, daemon=True)
    t.start()
    return t

# ---------------- Command Handling ----------------
def normalize_number_word(w):
    if w in ("to","too","2"): return "two"
    if w in ("1","one"): return "one"
    if w in ("3","three"): return "three"
    return w

def find_mode_from_text(text):
    for token in text.split():
        t = normalize_number_word(token)
        if t in MODE_SPEEDS:
            return t
    if "speed two" in text: return "two"
    if "speed one" in text: return "one"
    if "speed three" in text: return "three"
    return None

def handle_text(text):
    global voice_override_until, last_command_time

    text = text.replace("cruz", "cruise").replace("more", "mode")
    text = text.replace("speed to", "speed two")
    text = text.lower()

    if any(k in text for k in ("stop","halt","brake")):
        print("[CMD] stop")
        set_throttle(0.0)
        with voice_override_lock:
            voice_override_until = time.time() + EXTERNAL_UPDATE_INTERVAL
        last_command_time = time.time()
        return

    if any(k in text for k in ("increase","rise","faster","up")):
        print("[CMD] increase")
        set_throttle(current_throttle + THROTTLE_STEP)
        with voice_override_lock:
            voice_override_until = time.time() + EXTERNAL_UPDATE_INTERVAL
        last_command_time = time.time()
        return

    if any(k in text for k in ("decrease","slow","down","slower")):
        print("[CMD] decrease")
        set_throttle(current_throttle - THROTTLE_STEP)
        with voice_override_lock:
            voice_override_until = time.time() + EXTERNAL_UPDATE_INTERVAL
        last_command_time = time.time()
        return

    if any(k in text for k in ("speed","mode","cruise")):
        mode = find_mode_from_text(text)
        if mode:
            target_speed = MODE_SPEEDS[mode]
            desired_throttle = min(THROTTLE_MAX, target_speed * THROTTLE_PER_KMPH)
            print(f"[CMD] mode {mode} -> throttle {desired_throttle:.1f}%")
            set_throttle(desired_throttle)
            with voice_override_lock:
                voice_override_until = time.time() + EXTERNAL_UPDATE_INTERVAL
            last_command_time = time.time()
            return

    print("[CMD] no match:", text)

# ---------------- GPS ----------------
def gps_thread_func():
    global gps_fix, gps_log_file, running
    os.makedirs(GPS_LOG_DIR, exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    fname = os.path.join(GPS_LOG_DIR, f"run_{ts}.txt")

    try:
        gps_log_file = open(fname, "a", buffering=1)
        gps_log_file.write("# timestamp_iso, lat, lon, gps_speed_kmph\n")
    except Exception as e:
        print("[GPS] cannot open log file:", e)
        gps_log_file = None

    print(f"[GPS] logging to {fname}")

    try:
        ser = serial.Serial(GPS_SERIAL, GPS_BAUD, timeout=1)
    except Exception as e:
        print("[GPS] serial open failed:", e)
        return

    last_log_time = 0.0
    while running and not kill_triggered.is_set():
        try:
            line = ser.readline().decode(errors='ignore').strip()
            if not line or not line.startswith('$'):
                continue

            try:
                msg = pynmea2.parse(line)
            except pynmea2.ParseError:
                continue

            if msg.sentence_type == "RMC":
                lat = msg.latitude
                lon = msg.longitude
                spd_knots = float(msg.spd_over_grnd) if msg.spd_over_grnd else 0.0
                spd_kmph = spd_knots * 1.852

                with gps_lock:
                    gps_fix = {
                        "time": datetime.utcnow().isoformat(),
                        "lat": lat,
                        "lon": lon,
                        "speed_kmph": spd_kmph
                    }

                now_ts = time.time()
                if (now_ts - last_log_time) >= GPS_LOG_INTERVAL:
                    last_log_time = now_ts
                    if gps_log_file:
                        with gps_log_lock:
                            gps_log_file.write(
                                f"{datetime.utcnow().isoformat()}, {lat}, {lon}, {spd_kmph:.2f}\n"
                            )
                            gps_log_file.flush()

            elif msg.sentence_type == "GGA":
                with gps_lock:
                    gps_fix = {
                        "time": datetime.utcnow().isoformat(),
                        "lat": msg.latitude,
                        "lon": msg.longitude,
                        "speed_kmph": 0.0
                    }

        except Exception as e:
            print("[GPS] read exception:", e)
            time.sleep(0.1)

    try: ser.close()
    except: pass

    if gps_log_file:
        gps_log_file.close()
    print("[GPS] thread exiting")

# ---------------- External throttle update ----------------
def external_throttle_thread():
    global external_throttle, external_index, running
    pattern = EXTERNAL_THROTTLE_PATTERN
    idx = 0
    while running and not kill_triggered.is_set():
        val = pattern[idx % len(pattern)]
        idx += 1
        with external_lock:
            external_throttle = val

        with voice_override_lock:
            vo_until = voice_override_until
        now = time.time()

        if now >= vo_until:
            print(f"[SIM] external -> {val}%")
            set_throttle(val)
        else:
            remain = vo_until - now
            print(f"[SIM] skipped (voice override {remain:.2f}s)")

        sleep_time = EXTERNAL_UPDATE_INTERVAL
        slept = 0.0
        while slept < sleep_time and running and not kill_triggered.is_set():
            time.sleep(0.1)
            slept += 0.1

    print("[SIM] external throttle thread exiting")

# ---------------- Watchdog ----------------
def watchdog_thread():
    global last_command_time, running
    while running and not kill_triggered.is_set():
        time.sleep(1.0)
        if (time.time() - last_command_time) > WATCHDOG_TIMEOUT:
            if current_throttle != SAFE_THROTTLE:
                print("[WATCHDOG] inactivity -> safe throttle")
                set_throttle(SAFE_THROTTLE)
    print("[WATCHDOG] exiting")

# ---------------- Status Printer ----------------
def status_printer():
    while running and not kill_triggered.is_set():
        with gps_lock:
            gf = gps_fix.copy() if gps_fix else None
        with external_lock:
            ext = external_throttle
        with voice_override_lock:
            vo_until = voice_override_until

        now = time.time()
        vo_remaining = max(0.0, vo_until - now) if vo_until > now else 0.0

        if gf:
            print(f"[INFO] gps_speed={gf['speed_kmph']:.2f} ext_thr={ext} override={vo_remaining:.2f}s lat={gf['lat']} lon={gf['lon']}")
        else:
            print(f"[INFO] gps=NO_FIX ext_thr={ext} override={vo_remaining:.2f}s")

        time.sleep(5.0)

    print("[STATUS] printer exiting")

# ---------------- MAIN ----------------
def main():
    global running

    stt_t = start_stt()
    threading.Thread(target=kill_watchdog, daemon=True).start()
    threading.Thread(target=gps_thread_func, daemon=True).start()
    threading.Thread(target=external_throttle_thread, daemon=True).start()
    threading.Thread(target=watchdog_thread, daemon=True).start()
    threading.Thread(target=status_printer, daemon=True).start()

    try:
        with sd.RawInputStream(
            samplerate=SAMPLE_RATE,
            blocksize=AUDIO_BLOCKSIZE,
            dtype='int16',
            channels=CHANNELS,
            callback=audio_callback):

            print("[MAIN] Listening for commands.")
            set_throttle(0.0)

            while not kill_triggered.is_set():
                time.sleep(0.2)

            print("[MAIN] Kill triggered -> safe-hold")
            set_throttle(SAFE_THROTTLE)
            GPIO.output(LED_PIN, GPIO.HIGH)

            while True:
                time.sleep(1.0)

    except KeyboardInterrupt:
        print("[MAIN] KeyboardInterrupt")

    except Exception as e:
        print("[MAIN] exception:", e)

    finally:
        running = False
        time.sleep(0.2)
        print("[MAIN] cleanup -> SAFE_THROTTLE")
        set_throttle(SAFE_THROTTLE)
        time.sleep(0.2)

        try: pwm.ChangeDutyCycle(0)
        except: pass

        pwm.stop()

        try: GPIO.cleanup()
        except: pass

        print("[MAIN] exited cleanly")

if __name__ == "__main__":
    main()
