# -*- coding: utf-8 -*-
# Sensor logger for iPhone (Pythonista)
# Logs: gravity, userAcceleration (linear), rotationRate (gyro), magnetic field,
# attitude quaternion (rotation vector), pressure, mic level (dB), location.
# Saves CSV + optional audio recording.

# q256809221848 v1.0.0 chatgpt

import os, time, csv, math, threading, queue, datetime
from pathlib import Path

# ---------- USER SETTINGS ----------
SAMPLE_HZ = 25.0                # samples per second (try 25–60)
CSV_BASENAME = "sensor_log"     # will append timestamp
RECORD_AUDIO = True             # set False to skip audio file recording
AUDIO_BASENAME = "sensor_audio" # .m4a file
LOCATION_POLL_SEC = 1.0         # how often to refresh location
# ----------------------------------

# Pythonista modules
import location

# iOS frameworks via objc_util
from objc_util import ObjCClass, ObjCInstance, ns, c_void_p, ObjCBlock

# Core Motion
CMMotionManager = ObjCClass('CMMotionManager')
NSOperationQueue = ObjCClass('NSOperationQueue')

# Altimeter (pressure)
try:
    CMAltimeter = ObjCClass('CMAltimeter')
    HAS_ALTIMETER = True
except Exception:
    HAS_ALTIMETER = False

# AVFoundation for mic metering
AVAudioSession = ObjCClass('AVAudioSession')
AVAudioRecorder = ObjCClass('AVAudioRecorder')
NSURL = ObjCClass('NSURL')
NSDictionary = ObjCClass('NSDictionary')
NSNumber = ObjCClass('NSNumber')

# Utilities
def documents_path():
    # Pythonista’s Documents
    return Path(os.path.expanduser("~/Documents"))

ts_str = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
csv_path = documents_path() / f"{CSV_BASENAME}_{ts_str}.csv"
audio_path = documents_path() / f"{AUDIO_BASENAME}_{ts_str}.m4a"

print(f"CSV will be saved to: {csv_path}")
if RECORD_AUDIO:
    print(f"Audio (if enabled) will be saved to: {audio_path}")

# -------------- Microphone (metering / optional recording) --------------
class MicMeter:
    def __init__(self, record_path: Path = None):
        self.recorder = None
        self.level_db = None
        self._setup(record_path)

    def _setup(self, record_path: Path):
        session = AVAudioSession.sharedInstance()
        # category: PlayAndRecord gives mic access while allowing device audio
        ok, err = session.setCategory_error_(ns('AVAudioSessionCategoryPlayAndRecord'), None)
        ok, err = session.setActive_error_(True, None)

        settings = {
            # AAC, mono, 44.1k
            'AVFormatIDKey': 1633772320,   # kAudioFormatMPEG4AAC
            'AVSampleRateKey': 44100.0,
            'AVNumberOfChannelsKey': 1,
            'AVEncoderAudioQualityKey': 0x7F  # max
        }
        nss = NSDictionary.dictionaryWithDictionary_(ns(settings))

        url = None
        if record_path is not None:
            url = NSURL.fileURLWithPath_(ns(str(record_path)))
        else:
            # use a throwaway tmp file even if not keeping audio
            url = NSURL.fileURLWithPath_(ns(str(documents_path() / f"tmp_{ts_str}.m4a")))

        self.recorder = AVAudioRecorder.alloc().initWithURL_settings_error_(url, nss, None)
        if not self.recorder:
            print("Failed to init AVAudioRecorder")
            return

        self.recorder.setMeteringEnabled_(True)
        self.recorder.prepareToRecord()
        self.recorder.record()  # start recording (required for metering updates)

    def read_db(self):
        if not self.recorder:
            return None
        self.recorder.updateMeters()
        # averagePowerForChannel: 0 => dBFS (-160..0)
        return float(self.recorder.averagePowerForChannel_(0))

    def stop(self, keep_file=True):
        if self.recorder:
            self.recorder.stop()
            if not keep_file:
                # best-effort cleanup — iOS may still keep it briefly
                pass
            self.recorder = None

# -------------- Core Motion (device motion + magnetometer) --------------
class MotionSampler:
    def __init__(self, hz=50.0):
        self.mm = CMMotionManager.alloc().init()
        self.queue = NSOperationQueue.alloc().init()
        self.mm.deviceMotionUpdateInterval = 1.0 / hz
        self.mm.magnetometerUpdateInterval = 1.0 / hz
        self.latest = {}  # last sample dict
        self._lock = threading.Lock()

    def start(self):
        # Start DeviceMotion (gives gravity, userAcceleration, rotationRate, attitude, magnetic field)
        if self.mm.isDeviceMotionAvailable():
            self.mm.startDeviceMotionUpdatesToQueue_withHandler_(self.queue, self._make_dm_handler())
        else:
            print("DeviceMotion not available")

        # If standalone magnetometer needed (DeviceMotion usually provides), start it too:
        if self.mm.isMagnetometerAvailable():
            self.mm.startMagnetometerUpdatesToQueue_withHandler_(self.queue, self._make_mag_handler())
        else:
            print("Magnetometer not available")

    def _make_dm_handler(self):
        def handler(_cmd, data, error):
            if error:
                return
            dm = ObjCInstance(data)
            grav = dm.gravity()
            ua = dm.userAcceleration()
            rr = dm.rotationRate()
            att = dm.attitude()
            quat = att.quaternion()
            # magnetic field (calibrated) is via dm.magneticField().field
            mf = dm.magneticField().field()

            sample = {
                'gravity_x': float(grav.x), 'gravity_y': float(grav.y), 'gravity_z': float(grav.z),
                'linacc_x': float(ua.x), 'linacc_y': float(ua.y), 'linacc_z': float(ua.z),
                'gyro_x': float(rr.x), 'gyro_y': float(rr.y), 'gyro_z': float(rr.z),
                'quat_x': float(quat.x), 'quat_y': float(quat.y), 'quat_z': float(quat.z), 'quat_w': float(quat.w),
                'mag_x': float(mf.x), 'mag_y': float(mf.y), 'mag_z': float(mf.z)
            }
            with self._lock:
                self.latest.update(sample)
        return ObjCBlock(handler, restype=None, argtypes=[c_void_p, c_void_p, c_void_p])

    def _make_mag_handler(self):
        def handler(_cmd, data, error):
            # Not strictly needed (DeviceMotion’s calibrated field is usually enough)
            pass
        return ObjCBlock(handler, restype=None, argtypes=[c_void_p, c_void_p, c_void_p])

    def snapshot(self):
        with self._lock:
            return dict(self.latest)

    def stop(self):
        try:
            self.mm.stopDeviceMotionUpdates()
            self.mm.stopMagnetometerUpdates()
        except Exception:
            pass

# -------------- Altimeter (pressure) --------------
class PressureSampler:
    def __init__(self):
        self.altimeter = CMAltimeter.alloc().init() if HAS_ALTIMETER else None
        self.pressure_kpa = None
        self._lock = threading.Lock()
        self._running = False

    def start(self):
        if not (HAS_ALTIMETER and CMAltimeter.isRelativeAltitudeAvailable()):
            print("Altimeter/pressure not available on this device.")
            return
        self._running = True

        def handler(_cmd, data, error):
            if error or not data:
                return
            obj = ObjCInstance(data)
            # pressure in kPa as NSNumber
            p = float(obj.pressure())  # kPa
            with self._lock:
                self.pressure_kpa = p

        self.altimeter.startRelativeAltitudeUpdatesToQueue_withHandler_(NSOperationQueue.mainQueue(), ObjCBlock(handler, restype=None, argtypes=[c_void_p, c_void_p, c_void_p]))

    def read(self):
        with self._lock:
            return self.pressure_kpa

    def stop(self):
        if self.altimeter and self._running:
            self.altimeter.stopRelativeAltitudeUpdates()
            self._running = False

# -------------- Location (polling via Pythonista) --------------
class LocationSampler:
    def __init__(self, poll_sec=1.0):
        self.poll_sec = poll_sec
        self.latest = {}
        self._stop = threading.Event()
        self._t = None

    def _loop(self):
        while not self._stop.is_set():
            try:
                loc = location.get_location()  # dict with lat, lon, altitude, speed, course, horizontal_accuracy, vertical_accuracy, timestamp
                if loc:
                    self.latest = {
                        'lat': loc.get('latitude'),
                        'lon': loc.get('longitude'),
                        'alt': loc.get('altitude'),
                        'hacc': loc.get('horizontal_accuracy'),
                        'vacc': loc.get('vertical_accuracy'),
                        'speed': loc.get('speed'),
                        'course': loc.get('course')
                    }
            except Exception as e:
                pass
            self._stop.wait(self.poll_sec)

    def start(self):
        try:
            location.start_updates()
        except Exception as e:
            print("Location start failed:", e)
        self._t = threading.Thread(target=self._loop, daemon=True)
        self._t.start()

    def snapshot(self):
        return dict(self.latest)

    def stop(self):
        self._stop.set()
        try:
            location.stop_updates()
        except Exception:
            pass
        if self._t:
            self._t.join(timeout=1.0)

# -------------- CSV Writer --------------
FIELDNAMES = [
    't_iso', 't_epoch',
    # Motion
    'gravity_x','gravity_y','gravity_z',
    'linacc_x','linacc_y','linacc_z',
    'gyro_x','gyro_y','gyro_z',
    'quat_x','quat_y','quat_z','quat_w',
    'mag_x','mag_y','mag_z',
    # Pressure
    'pressure_kpa',
    # Mic
    'mic_db',
    # Location
    'lat','lon','alt','hacc','vacc','speed','course'
]

def open_csv(path: Path):
    f = open(path, 'w', newline='')
    w = csv.DictWriter(f, FIELDNAMES)
    w.writeheader()
    return f, w

# -------------- Main control --------------
def main():
    # Prepare CSV
    f, writer = open_csv(csv_path)

    # Start sensors
    ms = MotionSampler(hz=SAMPLE_HZ)
    ms.start()

    ps = PressureSampler()
    ps.start()

    ls = LocationSampler(poll_sec=LOCATION_POLL_SEC)
    ls.start()

    mic = MicMeter(record_path=(audio_path if RECORD_AUDIO else None))

    print("Logging… tap the (x) stop button in Pythonista or Ctrl-C in console to end.")
    period = 1.0 / SAMPLE_HZ
    last_loc = time.time()

    try:
        while True:
            now = time.time()
            row = {k: '' for k in FIELDNAMES}
            row['t_epoch'] = f"{now:.6f}"
            row['t_iso'] = datetime.datetime.utcfromtimestamp(now).isoformat() + 'Z'

            # Motion snapshot
            row.update(ms.snapshot())

            # Pressure
            p = ps.read()
            if p is not None:
                row['pressure_kpa'] = f"{p:.5f}"

            # Mic level
            db = mic.read_db()
            if db is not None:
                row['mic_db'] = f"{db:.2f}"

            # Location
            row.update(ls.snapshot())

            writer.writerow(row)
            f.flush()
            time.sleep(max(0.0, period))
    except KeyboardInterrupt:
        print("\nStopping…")
    finally:
        ms.stop()
        ps.stop()
        ls.stop()
        mic.stop(keep_file=RECORD_AUDIO)
        f.close()
        print(f"Saved CSV to: {csv_path}")
        if RECORD_AUDIO:
            print(f"Saved audio to: {audio_path}")

if __name__ == '__main__':
    main()
