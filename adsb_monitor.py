"""
ADS-B Overhead Monitor - Cube Orange+
-------------------------------------
Auto-connects to the Cube Orange+ (ADS-B carrier) over USB, shows live flight data
(attitude, altitude, battery, GPS) with an artificial horizon and a 3D orientation view,
lists ADS-B aircraft (airplanes, helicopters, drones, balloons) and sounds an alarm when
one is detected. Also shows:
  * aircraft from free online ADS-B networks (adsb.fi, adsb.lol) the local antenna misses,
  * satellites above you (ISS, Tiangong, Hubble, Starlink, weather, GPS/Galileo/GLONASS/BeiDou),
    predicted from CelesTrak orbits, with pass predictions and an alarm,
  * the Cube's GPS status next to the navigation satellites that are overhead.

Connection (automatic):
  * The app opens the Cube's USB port itself and re-shares it to Mission Planner on
    UDP 14550 - in Mission Planner pick "UDP", click Connect, port 14550.
  * If Mission Planner already holds the USB port, the app reads Mission Planner's live
    telemetry log (and its MAVLink Mirror on UDP 14550 if enabled).

Requires: pip install pymavlink pyserial numpy sgp4
"""

import csv
import glob
import json
import math
import os
import queue
import re
import socket
import threading
import time
import tkinter as tk
import urllib.error
import urllib.request
from tkinter import ttk

os.environ.setdefault("MAVLINK20", "1")
from pymavlink import mavutil  # noqa: E402

try:
    import serial.tools.list_ports as list_ports
except ImportError:
    list_ports = None

try:
    import winsound
except ImportError:
    winsound = None

try:
    import sounddevice as sd  # plays alarms on any speaker, incl. Bluetooth
except Exception:  # noqa: BLE001 - not installed / no audio: fall back to winsound
    sd = None

from online_feed import CATEGORY_TO_EMITTER, OnlineFeed  # noqa: E402

try:
    from sat_tracker import PASS_TARGETS, SatTracker
except ImportError:  # numpy / sgp4 not installed - satellites disabled
    SatTracker, PASS_TARGETS = None, {}

SETTINGS_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "adsb_monitor_settings.json")
LOG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "adsb_logs")
SOUND_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "sounds")

MODE_AUTO = "Auto (recommended)"
MODE_COM = "Direct COM port only"
MODE_UDP = "Mission Planner mirror (UDP) only"

DEFAULT_SETTINGS = {
    "connection": MODE_AUTO,
    "udp_port": 14550,
    "com_port": "",
    "baud": 115200,
    "share_with_mp": True,
    "home_lat": 29.8543,     # Roorkee
    "home_lon": 77.8880,
    "home_alt_m": 268.0,
    "alert_km": 0.0,         # 0 = unlimited: alarm for every aircraft the receiver hears
    "online_lookup": True,
    "alarm": True,
    "special_alarm": True,   # distinct alarm for helicopters / drones / balloons
    "net_feed": True,        # online ADS-B networks (adsb.fi, adsb.lol)
    "net_radius_km": 150.0,
    "net_alarm_km": 10.0,    # alarm for online-only aircraft within this distance (0 = never)
    "satellites": True,
    "sat_starlink": True,
    "sat_alarm": True,
    "sat_alarm_el": 10.0,    # alarm when ISS / Tiangong / Hubble climbs above this elevation
    "show_plane": True, "show_heli": True, "show_drone": True, "show_balloon": True, "show_ground": True,
    "show_net": True,
    "sat_show_starlink": False,
    "sat_show_gnss": True,
    "sound_device": "",      # "" = Windows default output (your Bluetooth speaker when connected)
    "volume": 100,
    "latch_aircraft": True,  # aircraft alarms keep sounding until STOP is pressed
    "latch_sats": False,
    # which detections play their tune
    "tune_plane": True, "tune_heli": True, "tune_drone": True, "tune_balloon": True, "tune_ground": True,
    "tune_net": True, "tune_emergency": True,
    "tune_iss": True, "tune_tiangong": True, "tune_hubble": True, "tune_bright": True,
    "tune_weather": True, "tune_starlink": False, "tune_gnss": False,
}

# ---- alarm tunes: list of (frequency Hz, duration ms); frequency 0 = rest
TUNES = {
    # aircraft
    "plane": ("Airplane", [(880, 220), (1320, 220)] * 3),
    "heli": ("Helicopter", [(330, 70), (0, 40)] * 8 + [(660, 250)]),
    "drone": ("Drone", [(1600, 90), (1000, 90)] * 6),
    "balloon": ("Balloon / glider", [(523, 140), (659, 140), (784, 140), (1047, 300)]),
    "ground": ("Ground vehicle", [(300, 300), (0, 120), (300, 300)]),
    "net": ("Online-feed aircraft", [(1000, 180), (750, 260)]),
    "emergency": ("Emergency squawk", [(2000, 120), (1400, 120)] * 6),
    # satellites
    "iss": ("ISS", [(523, 150), (659, 150), (784, 150), (1047, 400)]),
    "tiangong": ("Tiangong", [(784, 160), (659, 160), (523, 160), (659, 160), (784, 380)]),
    "hubble": ("Hubble", [(440, 180), (554, 180), (659, 180), (880, 420)]),
    "bright": ("Bright satellite", [(1319, 120), (1568, 120), (1319, 200)]),
    "weather": ("Weather satellite", [(880, 140), (0, 80), (880, 140), (0, 80), (1175, 260)]),
    "starlink": ("Starlink", [(2093, 60), (0, 50), (2093, 60)]),
    "gnss": ("GPS / GNSS satellite", [(1047, 110), (1319, 160)]),
}
AIRCRAFT_TUNES = ("plane", "heli", "drone", "balloon", "ground", "net", "emergency")
SAT_TUNES = ("iss", "tiangong", "hubble", "bright", "weather", "starlink", "gnss")
TUNE_PRIORITY = ["emergency", "drone", "heli", "balloon", "plane", "ground", "net",
                 "iss", "tiangong", "hubble", "weather", "bright", "gnss", "starlink"]

NEW_S = 30         # an aircraft counts as "just detected" for this many seconds

STALE_S = 20       # grey out after this many seconds without an update
REMOVE_S = 90      # drop from the list after this many seconds

ADSB_FLAG_VALID_COORDS = 1
ADSB_FLAG_VALID_ALTITUDE = 2
ADSB_FLAG_VALID_HEADING = 4
ADSB_FLAG_VALID_VELOCITY = 8
ADSB_FLAG_VALID_CALLSIGN = 16
ADSB_FLAG_VALID_SQUAWK = 32

EMITTER_TYPES = {
    0: "No info", 1: "Light", 2: "Small", 3: "Large", 4: "High-vortex large",
    5: "Heavy", 6: "Highly manoeuvrable", 7: "Rotorcraft", 9: "Glider",
    10: "Balloon", 11: "Parachute", 12: "Ultralight", 14: "UAV", 15: "Space",
    17: "Emergency vehicle", 18: "Service vehicle", 19: "Obstacle",
}
SPECIAL_SQUAWKS = {"7500": "HIJACK", "7600": "RADIO FAILURE", "7700": "EMERGENCY"}
GPS_FIX = {0: "No GPS", 1: "No fix", 2: "2D fix", 3: "3D fix", 4: "DGPS", 5: "RTK float", 6: "RTK fixed"}

TELEM_TYPES = {"HEARTBEAT", "ATTITUDE", "VFR_HUD", "GLOBAL_POSITION_INT", "SYS_STATUS",
               "GPS_RAW_INT", "GPS2_RAW", "SCALED_PRESSURE", "STATUSTEXT", "ADSB_VEHICLE"}

# --- colours
BG = "#0e1319"
PANEL = "#151c24"
CARD = "#1a232d"
BORDER = "#283442"
TEXT = "#e6edf3"
MUTED = "#8b98a5"
ACCENT = "#3fb6ff"
GREEN = "#3fd17f"
ORANGE = "#f0a43a"
RED = "#ff5c5c"
CUBE_ORANGE = "#ff8a1f"
PURPLE = "#b07cff"
FONT = "Segoe UI"

# what kind of thing an ADS-B report is: key -> (label, icon, colour)
KINDS = {
    "plane": ("Planes", "✈", GREEN),
    "heli": ("Helicopters", "🚁", "#c77dff"),
    "drone": ("Drones", "◆", "#ff6bd6"),
    "balloon": ("Balloons", "🎈", "#ffd43b"),
    "ground": ("Ground", "▣", MUTED),
}
KIND_NAMES = {"plane": "Airplane", "heli": "Helicopter", "drone": "Drone", "balloon": "Balloon/glider",
              "ground": "Ground vehicle"}
SPECIAL_KINDS = ("heli", "drone", "balloon")

# satellite colours on the sky view
SAT_COLORS = {"station": "#ffd43b", "bright": "#ffffff", "weather": "#4dd4ff", "starlink": "#5d7891",
              "GPS": "#3fd17f", "Galileo": "#3f8cff", "GLONASS": "#ff5c5c", "BeiDou": "#f0a43a"}


def blend(c1, c2, t):
    """Mix two #rrggbb colours: t=0 -> c1, t=1 -> c2 (Tk has no transparency)."""
    a = [int(c1[i:i + 2], 16) for i in (1, 3, 5)]
    b = [int(c2[i:i + 2], 16) for i in (1, 3, 5)]
    return "#" + "".join(f"{round(x + (y - x) * t):02x}" for x, y in zip(a, b))


def kind_of(emitter):
    if emitter == 7:
        return "heli"
    if emitter == 14:
        return "drone"
    if emitter in (9, 10, 11, 12):
        return "balloon"
    if emitter in (17, 18, 19):
        return "ground"
    return "plane"


# --------------------------------------------------------------------------- helpers

def load_settings():
    settings = dict(DEFAULT_SETTINGS)
    try:
        with open(SETTINGS_FILE, "r", encoding="utf-8") as f:
            settings.update(json.load(f))
    except (OSError, ValueError):
        pass
    if settings["home_lat"] == "" or settings["home_lon"] == "":
        for k in ("home_lat", "home_lon", "home_alt_m"):
            settings[k] = DEFAULT_SETTINGS[k]
    if settings["connection"] not in (MODE_AUTO, MODE_COM, MODE_UDP):
        settings["connection"] = MODE_AUTO
    return settings


def save_settings(settings):
    try:
        with open(SETTINGS_FILE, "w", encoding="utf-8") as f:
            json.dump({k: settings[k] for k in DEFAULT_SETTINGS}, f, indent=2)
    except OSError:
        pass


def log_event(text):
    """Append a line to today's event log (connection changes, detections)."""
    try:
        os.makedirs(LOG_DIR, exist_ok=True)
        with open(os.path.join(LOG_DIR, f"events_{time.strftime('%Y-%m-%d')}.log"), "a", encoding="utf-8") as f:
            f.write(f"{time.strftime('%H:%M:%S')}  {text}\n")
    except OSError:
        pass


def log_adsb(a):
    """Append one received ADS-B report to today's CSV."""
    try:
        os.makedirs(LOG_DIR, exist_ok=True)
        path = os.path.join(LOG_DIR, f"aircraft_{time.strftime('%Y-%m-%d')}.csv")
        new = not os.path.exists(path)
        with open(path, "a", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            if new:
                w.writerow(["time", "icao", "callsign", "lat", "lon", "alt_m", "dist_km", "bearing",
                            "heading", "speed_kmh", "squawk"])
            w.writerow([time.strftime("%H:%M:%S"), a.icao_hex, a.callsign,
                        f"{a.lat:.5f}" if a.lat is not None else "", f"{a.lon:.5f}" if a.lon is not None else "",
                        f"{a.alt_m:.0f}" if a.alt_m is not None else "",
                        f"{a.dist_m / 1000:.1f}" if a.dist_m is not None else "",
                        f"{a.bearing:.0f}" if a.bearing is not None else "",
                        f"{a.heading:.0f}" if a.heading is not None else "",
                        f"{a.speed_ms * 3.6:.0f}" if a.speed_ms is not None else "", a.squawk])
    except OSError:
        pass


def distance_bearing(lat1, lon1, lat2, lon2):
    """Great-circle distance (m) and initial bearing (deg) from point 1 to point 2."""
    r = 6371000.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp, dl = p2 - p1, math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    dist = 2 * r * math.asin(math.sqrt(a))
    y = math.sin(dl) * math.cos(p2)
    x = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dl)
    return dist, (math.degrees(math.atan2(y, x)) + 360) % 360


def compass(deg):
    names = ["N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE",
             "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"]
    return names[int((deg + 11.25) // 22.5) % 16]


def list_com_ports():
    if list_ports is None:
        return []
    return [f"{p.device} - {p.description}" for p in list_ports.comports()]


def find_cube_port():
    """Return the Cube's MAVLink COM port (CubePilot USB VID 0x2DAE), or None."""
    if list_ports is None:
        return None
    candidates = []
    for p in list_ports.comports():
        desc = (p.description or "").lower()
        if p.vid == 0x2DAE or "cube" in desc:
            if "slcan" in desc:
                continue
            candidates.append((0 if "mavlink" in desc else 1, p.device))
    return sorted(candidates)[0][1] if candidates else None


# --------------------------------------------------------------------------- data

class Aircraft:
    def __init__(self, icao):
        self.icao = icao
        self.callsign = ""
        self.lat = self.lon = None
        self.alt_m = None
        self.alt_type = ""
        self.heading = None
        self.speed_ms = None
        self.vspeed_ms = None
        self.squawk = ""
        self.emitter = 0
        self.first_seen = time.time()
        self.last_seen = time.time()
        self.msgs = 0
        self.dist_m = None
        self.bearing = None
        self.elevation = None
        self.in_range = False
        self.alerted = False
        self.detected_at = None  # when the alarm fired for this aircraft
        self.min_dist_m = None
        self.info = {}          # online lookup result
        self.last_cube = None   # last report from the Cube's own receiver
        self.last_net = None    # last report from the online feed
        self.on_ground = False

    @property
    def icao_hex(self):
        return f"{self.icao:06X}"

    @property
    def kind(self):
        return kind_of(self.emitter)

    def cube_recent(self, now=None):
        return self.last_cube is not None and (now or time.time()) - self.last_cube < STALE_S

    def net_recent(self, now=None):
        return self.last_net is not None and (now or time.time()) - self.last_net < STALE_S

    @property
    def source(self):
        now = time.time()
        c, n = self.cube_recent(now), self.net_recent(now)
        if c and n:
            return "Cube+Net"
        if c or (self.last_cube and not n):
            return "Cube"
        return "Net"

    def update_from_net(self, rec, fetched_at):
        """Merge a report from the online feed (adsb.fi / adsb.lol format)."""
        t = fetched_at - float(rec.get("seen_pos", rec.get("seen", 0)) or 0)
        self.last_net = max(self.last_net or 0.0, t)
        self.last_seen = max(self.last_seen, t)
        if rec.get("r"):
            self.info.setdefault("Registration", rec["r"])
        if rec.get("t"):
            self.info.setdefault("ICAO type", rec["t"])
        if rec.get("desc"):
            self.info.setdefault("Type", rec["desc"].title())
        flight = (rec.get("flight") or "").strip()
        if self.cube_recent():
            if flight and not self.callsign:
                self.callsign = flight
            return  # the Cube's own data is fresher - keep it
        self.lat, self.lon = float(rec["lat"]), float(rec["lon"])
        alt = rec.get("alt_baro", rec.get("alt_geom"))
        if alt == "ground":
            self.alt_m, self.on_ground = 0.0, True
        elif isinstance(alt, (int, float)):
            self.alt_m, self.alt_type, self.on_ground = alt * 0.3048, "Baro", False
        if rec.get("track") is not None:
            self.heading = float(rec["track"])
        if rec.get("gs") is not None:
            self.speed_ms = float(rec["gs"]) * 0.514444
        if rec.get("baro_rate") is not None:
            self.vspeed_ms = float(rec["baro_rate"]) * 0.00508
        if rec.get("squawk"):
            self.squawk = str(rec["squawk"])
        if flight:
            self.callsign = flight
        if rec.get("category") in CATEGORY_TO_EMITTER:
            self.emitter = CATEGORY_TO_EMITTER[rec["category"]]

    def update(self, msg):
        f = msg.flags
        self.msgs += 1
        self.last_seen = self.last_cube = time.time()
        if f & ADSB_FLAG_VALID_COORDS and (msg.lat or msg.lon):
            self.lat, self.lon = msg.lat / 1e7, msg.lon / 1e7
        if f & ADSB_FLAG_VALID_ALTITUDE:
            self.alt_m = msg.altitude / 1000.0
            self.alt_type = "GNSS" if msg.altitude_type == 1 else "Baro"
        if f & ADSB_FLAG_VALID_HEADING:
            self.heading = msg.heading / 100.0
        if f & ADSB_FLAG_VALID_VELOCITY:
            self.speed_ms = msg.hor_velocity / 100.0
            self.vspeed_ms = msg.ver_velocity / 100.0
        if f & ADSB_FLAG_VALID_CALLSIGN:
            cs = msg.callsign
            if isinstance(cs, bytes):
                cs = cs.decode("ascii", "ignore")
            self.callsign = cs.strip("\x00 ").strip()
        if f & ADSB_FLAG_VALID_SQUAWK:
            self.squawk = f"{msg.squawk:04d}"
        self.emitter = msg.emitter_type

    def compute_geometry(self, home):
        if home is None or self.lat is None:
            self.dist_m = self.bearing = self.elevation = None
            return
        self.dist_m, self.bearing = distance_bearing(home[0], home[1], self.lat, self.lon)
        if self.min_dist_m is None or self.dist_m < self.min_dist_m:
            self.min_dist_m = self.dist_m
        if self.alt_m is not None:
            self.elevation = math.degrees(math.atan2(self.alt_m - home[2], max(self.dist_m, 1.0)))


class Link(threading.Thread):
    """Finds the Cube and keeps a MAVLink connection alive, reconnecting automatically.

    Pushes ("link", (state, text)) and ("msg", mavlink_msg) into the GUI queue.
    """

    def __init__(self, settings, out_q):
        super().__init__(daemon=True)
        self.mode = settings["connection"]
        self.com_pref = settings["com_port"].split(" - ")[0].strip()
        self.baud = int(settings["baud"])
        self.udp_port = int(settings["udp_port"])
        self.share = bool(settings["share_with_mp"])
        self.q = out_q
        self.stop_event = threading.Event()

    def stop(self):
        self.stop_event.set()

    def _status(self, state, text):
        self.q.put(("link", (state, text)))

    def run(self):
        while not self.stop_event.is_set():
            result = None
            if self.mode != MODE_UDP:
                port = self.com_pref if (self.mode == MODE_COM and self.com_pref) else find_cube_port()
                if port:
                    result = self._run_serial(port)
                elif self.mode == MODE_COM or self.mode == MODE_AUTO:
                    self._status("searching", "Searching for Cube Orange - plug in the USB cable")
            if self.stop_event.is_set():
                break
            if self.mode == MODE_UDP or (self.mode == MODE_AUTO and result == "busy"):
                self._run_udp(auto=self.mode == MODE_AUTO)
            self.stop_event.wait(1.5)

    # ---- direct USB
    def _run_serial(self, port):
        try:
            conn = mavutil.mavlink_connection(port, baud=self.baud, source_system=255,
                                              source_component=190, autoreconnect=False)
        except Exception as e:  # noqa: BLE001
            text = str(e)
            if "denied" in text.lower() or "PermissionError" in text:
                if self.mode == MODE_COM:
                    self._status("busy", f"{port} is in use (Mission Planner?) - disconnect it there")
                return "busy"
            self._status("error", f"Cannot open {port}: {text}")
            return "error"

        relay = None
        if self.share:
            relay = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            relay.bind(("127.0.0.1", 0))
            relay.setblocking(False)
            if hasattr(socket, "SIO_UDP_CONNRESET"):
                relay.ioctl(socket.SIO_UDP_CONNRESET, False)  # ignore "port unreachable" when MP is closed
        mp_addr = ("127.0.0.1", self.udp_port)

        self._status("connecting", f"Opened {port} - waiting for Cube heartbeat")
        now = time.time()
        last_rx, last_hb, streams_at, vehicle = now, 0.0, 0.0, None
        try:
            while not self.stop_event.is_set():
                now = time.time()
                if now - last_hb > 1.0:
                    conn.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                            mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
                    last_hb = now
                if relay:  # Mission Planner -> Cube
                    try:
                        while True:
                            data, _ = relay.recvfrom(65535)
                            conn.write(data)
                    except (BlockingIOError, OSError):
                        pass
                msg = conn.recv_match(blocking=True, timeout=0.05)
                if msg is None:
                    if now - last_rx > 5:
                        self._status("lost", f"No data from {port} - reconnecting")
                        return "lost"
                    continue
                t = msg.get_type()
                if t == "BAD_DATA":
                    continue
                last_rx = now
                if relay:  # Cube -> Mission Planner
                    try:
                        relay.sendto(msg.get_msgbuf(), mp_addr)
                    except OSError:
                        pass
                if (t == "HEARTBEAT" and vehicle is None
                        and msg.autopilot != mavutil.mavlink.MAV_AUTOPILOT_INVALID):
                    vehicle = (msg.get_srcSystem(), msg.get_srcComponent())
                    share = f"  ·  shared to Mission Planner on UDP {self.udp_port}" if relay else ""
                    self._status("direct", f"Connected to Cube on {port} (USB){share}")
                if vehicle and now - streams_at > 20:
                    streams_at = now
                    conn.mav.request_data_stream_send(vehicle[0], vehicle[1],
                                                      mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1)
                    conn.mav.request_data_stream_send(vehicle[0], vehicle[1],
                                                      mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 15, 1)
                if t in TELEM_TYPES:
                    self.q.put(("msg", msg))
        except Exception as e:  # noqa: BLE001
            self._status("lost", f"Connection to {port} lost ({e}) - reconnecting")
            return "lost"
        finally:
            try:
                conn.close()
            except Exception:  # noqa: BLE001
                pass
            if relay:
                relay.close()
        return "stopped"

    # ---- via Mission Planner (it holds the USB port)
    def _run_udp(self, auto):
        """Read Mission Planner's live telemetry log (no setup needed) and its MAVLink Mirror (UDP)."""
        conn = None
        try:
            conn = mavutil.mavlink_connection(f"udpin:0.0.0.0:{self.udp_port}")
        except Exception:  # noqa: BLE001
            pass  # port taken - the live log still works
        tail = TlogTail(mission_planner_log_dir()) if auto else None
        self._status("mirror_wait", "Mission Planner is using the USB port - waiting for its live data "
                                    "(make sure Mission Planner is connected to the Cube)")
        start = time.time()
        last_rx = {"log": None, "udp": None}
        source = None
        try:
            while not self.stop_event.is_set():
                got = []
                if tail:
                    got += [("log", m) for m in tail.poll()]
                if conn:
                    msg = conn.recv_match(blocking=True, timeout=0.2)
                    if msg is not None and msg.get_type() != "BAD_DATA":
                        got.append(("udp", msg))
                else:
                    self.stop_event.wait(0.2)
                now = time.time()
                log_live = last_rx["log"] and now - last_rx["log"] < 3
                for src, msg in got:
                    last_rx[src] = now
                    if src == "udp" and log_live:
                        continue  # same data already arriving through the log
                    want = "log" if src == "log" else "udp"
                    if source != want:
                        source = want
                        self._status("mirror", "Reading Mission Planner's live data (Mission Planner has the USB port)"
                                     if want == "log" else
                                     f"Connected via Mission Planner mirror (UDP {self.udp_port})")
                    if msg.get_type() in TELEM_TYPES:
                        self.q.put(("msg", msg))
                newest = max([t for t in last_rx.values() if t] or [start])
                if now - newest > 8:
                    if source is not None:
                        source = None
                        self._status("mirror_wait", "No data from Mission Planner - is it still connected?")
                    if auto:
                        return  # go back and check if the USB port became free
        except Exception as e:  # noqa: BLE001
            self._status("lost", f"Error reading Mission Planner data: {e}")
        finally:
            if conn:
                try:
                    conn.close()
                except Exception:  # noqa: BLE001
                    pass
            if tail:
                tail.close()


def mission_planner_log_dir():
    """Mission Planner's telemetry log folder (from its config.xml), or None."""
    home = os.path.expanduser("~")
    for base in (os.path.join(home, "OneDrive", "Documents", "Mission Planner"),
                 os.path.join(home, "Documents", "Mission Planner")):
        cfg = os.path.join(base, "config.xml")
        try:
            with open(cfg, "r", encoding="utf-8", errors="ignore") as f:
                m = re.search(r"<logdirectory>(.*?)</logdirectory>", f.read())
            if m and os.path.isdir(m.group(1).strip()):
                return m.group(1).strip()
        except OSError:
            pass
        if os.path.isdir(os.path.join(base, "logs")):
            return os.path.join(base, "logs")
    return None


class TlogTail:
    """Follows the newest .tlog that Mission Planner is writing and decodes new MAVLink packets.

    A tlog is a sequence of [8-byte timestamp][raw MAVLink packet] records.
    """

    def __init__(self, log_dir):
        self.log_dir = log_dir
        self.path = None
        self.f = None
        self.buf = b""
        self.last_scan = 0.0
        self.started = time.time()
        self.mav = mavutil.mavlink.MAVLink(None)
        self.mav.robust_parsing = True

    def close(self):
        if self.f:
            self.f.close()
            self.f = None

    def _newest(self):
        files = glob.glob(os.path.join(self.log_dir, "**", "*.tlog"), recursive=True)
        live = []
        for p in files:
            try:
                mtime = os.path.getmtime(p)
            except OSError:
                continue
            if time.time() - mtime < 30:  # being written right now
                live.append((mtime, p))
        return max(live)[1] if live else None

    def poll(self):
        if not self.log_dir:
            return []
        now = time.time()
        if now - self.last_scan > 2:
            self.last_scan = now
            newest = self._newest()
            if newest and newest != self.path:
                self.close()
                try:
                    self.f = open(newest, "rb")
                    created = os.path.getctime(newest)
                    # a log that started before us: only read what's new; a fresh one: read it all
                    self.f.seek(0, os.SEEK_SET if created > self.started - 5 else os.SEEK_END)
                    self.path, self.buf = newest, b""
                except OSError:
                    self.f = None
        if not self.f:
            return []
        try:
            data = self.f.read()
        except OSError:
            return []
        if not data:
            return []
        self.buf += data
        return self._parse()

    def _parse(self):
        out, buf, i = [], self.buf, 0
        while len(buf) - i >= 8 + 3:
            magic = buf[i + 8]
            if magic == 0xFE:
                plen = buf[i + 9] + 8
            elif magic == 0xFD:
                plen = buf[i + 9] + 12 + (13 if buf[i + 10] & 0x01 else 0)
            else:
                i += 1
                continue
            if len(buf) < i + 8 + plen:
                break  # wait for the rest of the packet
            try:
                msgs = self.mav.parse_buffer(buf[i + 8:i + 8 + plen]) or []
            except Exception:  # noqa: BLE001
                msgs = []
            good = [m for m in msgs if m.get_type() != "BAD_DATA"]
            if good:
                out.extend(good)
                i += 8 + plen
            else:
                self.mav = mavutil.mavlink.MAVLink(None)  # reset parser state and resync
                self.mav.robust_parsing = True
                i += 1
        self.buf = buf[i:]
        return out


class OnlineLookup(threading.Thread):
    """Looks up registration / type / route on api.adsbdb.com (free, no key)."""

    def __init__(self, out_q):
        super().__init__(daemon=True)
        self.in_q = queue.Queue()
        self.out_q = out_q
        self.done = set()

    def request(self, icao_hex, callsign):
        key = (icao_hex, callsign)
        if key not in self.done:
            self.done.add(key)
            self.in_q.put(key)

    @staticmethod
    def _get(url):
        req = urllib.request.Request(url, headers={"User-Agent": "adsb-overhead-monitor/1.0"})
        try:
            with urllib.request.urlopen(req, timeout=8) as r:
                return json.loads(r.read().decode("utf-8")).get("response")
        except urllib.error.HTTPError:
            return None  # 404 = unknown aircraft / callsign

    def run(self):
        while True:
            icao_hex, callsign = self.in_q.get()
            info = {}
            try:
                resp = self._get(f"https://api.adsbdb.com/v0/aircraft/{icao_hex}")
                if isinstance(resp, dict) and "aircraft" in resp:
                    a = resp["aircraft"]
                    info.update({
                        "Registration": a.get("registration"),
                        "Type": " ".join(x for x in (a.get("manufacturer"), a.get("type")) if x),
                        "ICAO type": a.get("icao_type"),
                        "Owner": a.get("registered_owner"),
                        "Country": a.get("registered_owner_country_name"),
                    })
                if callsign:
                    resp = self._get(f"https://api.adsbdb.com/v0/callsign/{callsign}")
                    if isinstance(resp, dict) and "flightroute" in resp:
                        fr = resp["flightroute"]

                        def place(p):
                            if not p:
                                return None
                            code = p.get("iata_code") or p.get("icao_code") or ""
                            return f"{p.get('municipality') or p.get('name')} ({code})"

                        info["Airline"] = (fr.get("airline") or {}).get("name")
                        info["From"] = place(fr.get("origin"))
                        info["To"] = place(fr.get("destination"))
                info = {k: v for k, v in info.items() if v}
                if not info:
                    info = {"Lookup": "not found in database"}
            except Exception as e:  # noqa: BLE001
                info = {"Lookup": f"offline ({e.__class__.__name__})"}
                self.done.discard((icao_hex, callsign))  # allow retry later
            self.out_q.put(("lookup", (icao_hex, info)))


SAMPLE_RATE = 44100


def render_tune(tune, volume=1.0, lead=0.35, tail=0.1):
    """Synthesize a tune to float32 samples. The silent lead-in gives Bluetooth speakers time to
    wake up so the first notes aren't cut off."""
    import numpy as np
    parts = [np.zeros(int(SAMPLE_RATE * lead))]
    for freq, ms in TUNES[tune][1]:
        n = int(SAMPLE_RATE * ms / 1000)
        if freq <= 0:
            parts.append(np.zeros(n))
            continue
        t = np.arange(n) / SAMPLE_RATE
        # a few harmonics make it louder and clearer on small speakers than a pure sine
        w = (np.sin(2 * np.pi * freq * t) + 0.35 * np.sin(4 * np.pi * freq * t)
             + 0.2 * np.sin(6 * np.pi * freq * t))
        fade = min(n // 4, int(SAMPLE_RATE * 0.006))
        if fade:
            w[:fade] *= np.linspace(0, 1, fade)
            w[-fade:] *= np.linspace(1, 0, fade)
        parts.append(w)
    parts.append(np.zeros(int(SAMPLE_RATE * tail)))
    y = np.concatenate(parts)
    peak = np.max(np.abs(y)) or 1.0
    return (y / peak * 0.9 * max(0.05, min(1.0, volume))).astype(np.float32)


def output_devices():
    """Names of the sound outputs (Windows DirectSound list, which has full device names)."""
    if sd is None:
        return []
    try:
        apis = sd.query_hostapis()
        return [d["name"] for d in sd.query_devices()
                if d["max_output_channels"] > 0 and apis[d["hostapi"]]["name"] == "Windows DirectSound"
                and d["name"] != "Primary Sound Driver"]
    except Exception:  # noqa: BLE001
        return []


def default_output_name():
    if sd is None:
        return "?"
    try:
        return sd.query_devices(sd.default.device[1])["name"]
    except Exception:  # noqa: BLE001
        return "?"


class AlarmPlayer(threading.Thread):
    """Plays alarm tunes as real audio on the chosen speaker (Windows default = your Bluetooth speaker).

    * play(tune)  - play once (queued, a tune already waiting isn't queued twice)
    * latch(tune) - loop the tune until acknowledge() is called
    """

    def __init__(self, enabled_fn, device_fn, volume_fn):
        super().__init__(daemon=True)
        self.enabled = enabled_fn
        self.device_name = device_fn
        self.volume = volume_fn
        self.pending = []
        self.latched = None
        self.lock = threading.Lock()
        self.wake = threading.Event()
        self.last_error = ""
        self.cache = {}

    def play(self, tune, preview=False):
        """preview=True plays even when the master alarm switch is off (Settings ▶ buttons)."""
        with self.lock:
            if (tune, preview) not in self.pending and tune != self.latched:
                self.pending.append((tune, preview))
        self.wake.set()

    def latch(self, tune):
        with self.lock:
            # a more urgent alarm replaces the one that is sounding
            if self.latched is None or TUNE_PRIORITY.index(tune) < TUNE_PRIORITY.index(self.latched):
                self.latched = tune
        self.wake.set()

    def acknowledge(self):
        with self.lock:
            self.latched = None
            self.pending.clear()
        self.wake.set()

    def _device(self):
        name = self.device_name()
        if not name or sd is None:
            return None  # Windows default output
        apis = sd.query_hostapis()
        for i, d in enumerate(sd.query_devices()):
            if d["name"] == name and d["max_output_channels"] > 0 and apis[d["hostapi"]]["name"] == "Windows DirectSound":
                return i
        return None

    def _start(self, tune, loop):
        """Start playing; returns the duration in seconds."""
        vol = self.volume()
        key = (tune, loop, vol)
        if key not in self.cache:
            self.cache[key] = render_tune(tune, vol, lead=0.35, tail=0.7 if loop else 0.1)
        data = self.cache[key]
        if sd is not None:
            try:
                sd.play(data, SAMPLE_RATE, device=self._device(), loop=loop)
                self.last_error = ""
                return len(data) / SAMPLE_RATE
            except Exception as e:  # noqa: BLE001 - e.g. chosen speaker disconnected: use the default
                try:
                    sd.play(data, SAMPLE_RATE, loop=loop)
                    self.last_error = f"chosen speaker unavailable ({e.__class__.__name__}) - using default"
                    return len(data) / SAMPLE_RATE
                except Exception as e2:  # noqa: BLE001
                    self.last_error = f"sound error: {e2}"
        if winsound is not None:  # fallback: a WAV file through Windows' default output
            path = os.path.join(SOUND_DIR, f"{tune}{'_loop' if loop else ''}.wav")
            write_wav(path, data)
            flags = winsound.SND_FILENAME | winsound.SND_ASYNC | (winsound.SND_LOOP if loop else 0)
            winsound.PlaySound(path, flags)
        return len(data) / SAMPLE_RATE

    def _stop(self):
        try:
            if sd is not None:
                sd.stop()
            if winsound is not None:
                winsound.PlaySound(None, winsound.SND_PURGE)
        except Exception:  # noqa: BLE001
            pass

    def run(self):
        looping = None
        while True:
            with self.lock:
                want = self.latched
                job = self.pending.pop(0) if (want is None and self.pending) else None
            if want != looping:
                if want and self.enabled():
                    self._start(want, loop=True)
                else:
                    self._stop()
                looping = want if self.enabled() else None
            elif job:
                tune, preview = job
                if preview or self.enabled():
                    duration = self._start(tune, loop=False)
                    self.wake.clear()
                    if self.wake.wait(duration):  # a latch or STOP interrupts a one-shot tune
                        continue
            self.wake.wait(0.25)
            self.wake.clear()


def write_wav(path, data):
    import wave
    import numpy as np
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with wave.open(path, "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(SAMPLE_RATE)
        w.writeframes((np.clip(data, -1, 1) * 32767).astype("<i2").tobytes())


# --------------------------------------------------------------------------- widgets

class Card(tk.Frame):
    """Dark rounded-look panel with a small uppercase title."""

    def __init__(self, parent, title, **kw):
        super().__init__(parent, bg=CARD, highlightthickness=1, highlightbackground=BORDER, **kw)
        self.title_label = tk.Label(self, text=title.upper(), bg=CARD, fg=MUTED, font=(FONT, 9, "bold"))
        self.title_label.pack(anchor="w", padx=12, pady=(10, 4))
        self.body = tk.Frame(self, bg=CARD)
        self.body.pack(fill="both", expand=True, padx=12, pady=(0, 12))


class Pill(tk.Frame):
    """Status pill: coloured dot + text."""

    def __init__(self, parent, text):
        super().__init__(parent, bg=PANEL)
        self.dot = tk.Canvas(self, width=12, height=12, bg=PANEL, highlightthickness=0)
        self.dot.pack(side="left", padx=(0, 6))
        self.label = tk.Label(self, text=text, bg=PANEL, fg=TEXT, font=(FONT, 10))
        self.label.pack(side="left")
        self.set(text, MUTED)

    def set(self, text, color):
        self.dot.delete("all")
        self.dot.create_oval(2, 2, 11, 11, fill=color, outline="")
        self.label.configure(text=text)


class Horizon(tk.Canvas):
    """Artificial horizon (roll / pitch) with heading readout."""

    PX_PER_DEG = 3.2

    def __init__(self, parent, size=230):
        super().__init__(parent, width=size, height=size + 30, bg=CARD, highlightthickness=0)
        self.size = size

    def draw(self, roll_deg, pitch_deg, heading_deg):
        self.delete("all")
        s = self.size
        cx = cy = s / 2
        rad = s / 2 - 6
        th = math.radians(roll_deg)
        cos_t, sin_t = math.cos(th), math.sin(th)

        def rot(x, y):
            return cx + x * cos_t + y * sin_t, cy - x * sin_t + y * cos_t

        off = pitch_deg * self.PX_PER_DEG
        big = s * 2
        self.create_rectangle(0, 0, s, s, fill="#2f7fd0", outline="")       # sky
        ground = [rot(-big, off), rot(big, off), rot(big, big), rot(-big, big)]
        self.create_polygon(*[c for p in ground for c in p], fill="#8a5a2b", outline="")
        self.create_line(*rot(-big, off), *rot(big, off), fill="white", width=2)
        for deg in (-30, -20, -10, 10, 20, 30):
            y = off - deg * self.PX_PER_DEG
            half = 22 if deg % 20 else 34
            self.create_line(*rot(-half, y), *rot(half, y), fill="white", width=1)
            tx, ty = rot(half + 12, y)
            self.create_text(tx, ty, text=str(abs(deg)), fill="white", font=(FONT, 7))
        # round mask
        self.create_oval(cx - rad - 60, cy - rad - 60, cx + rad + 60, cy + rad + 60,
                         outline=CARD, width=120)
        self.create_oval(cx - rad, cy - rad, cx + rad, cy + rad, outline=BORDER, width=2)
        # roll scale + pointer
        for a in (-60, -45, -30, -20, -10, 0, 10, 20, 30, 45, 60):
            ar = math.radians(a)
            r1, r2 = rad - 2, rad - (12 if a % 30 == 0 else 7)
            self.create_line(cx + r1 * math.sin(ar), cy - r1 * math.cos(ar),
                             cx + r2 * math.sin(ar), cy - r2 * math.cos(ar), fill="white", width=2)
        pr = math.radians(-roll_deg)
        tip = (cx + (rad - 14) * math.sin(pr), cy - (rad - 14) * math.cos(pr))
        left = (cx + (rad - 26) * math.sin(pr - 0.07), cy - (rad - 26) * math.cos(pr - 0.07))
        right = (cx + (rad - 26) * math.sin(pr + 0.07), cy - (rad - 26) * math.cos(pr + 0.07))
        self.create_polygon(*tip, *left, *right, fill="#ffd400", outline="")
        # fixed aircraft symbol
        y = cy
        self.create_line(cx - 50, y, cx - 18, y, cx - 10, y + 9, fill="#ffd400", width=4)
        self.create_line(cx + 50, y, cx + 18, y, cx + 10, y + 9, fill="#ffd400", width=4)
        self.create_oval(cx - 3, y - 3, cx + 3, y + 3, fill="#ffd400", outline="")
        # heading box below the instrument
        self.create_rectangle(cx - 46, s + 2, cx + 46, s + 26, fill="#0b0f14", outline=BORDER)
        self.create_text(cx, s + 14, text=f"HDG {heading_deg:03.0f}° {compass(heading_deg)}",
                         fill=TEXT, font=(FONT, 9, "bold"))


class CubeView(tk.Canvas):
    """3D view of the flight controller orientation (roll, pitch, yaw)."""

    # body frame: x forward, y right, z down (Cube is ~ 94 x 44 x 30 mm on its carrier)
    L, W, H = 1.5, 1.0, 0.45

    def __init__(self, parent, size=230):
        super().__init__(parent, width=size, height=size + 30, bg=CARD, highlightthickness=0)
        self.size = size

    def draw(self, roll_deg, pitch_deg, yaw_deg):
        self.delete("all")
        s = self.size
        r, p, y = map(math.radians, (roll_deg, pitch_deg, yaw_deg))
        cr, sr, cp, sp, cy_, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
        # body -> NED rotation (ZYX)
        R = [
            [cp * cy_, sr * sp * cy_ - cr * sy, cr * sp * cy_ + sr * sy],
            [cp * sy, sr * sp * sy + cr * cy_, cr * sp * sy - sr * cy_],
            [-sp, sr * cp, cr * cp],
        ]
        def project(bx, by, bz):
            return self._camera(R[0][0] * bx + R[0][1] * by + R[0][2] * bz,
                                R[1][0] * bx + R[1][1] * by + R[1][2] * bz,
                                R[2][0] * bx + R[2][1] * by + R[2][2] * bz)

        # ground grid below the cube
        for i in range(-2, 3):
            for a, b in (((i, -2), (i, 2)), ((-2, i), (2, i))):
                pa, pb = self._camera(a[0] * 0.6, a[1] * 0.6, 0.9), self._camera(b[0] * 0.6, b[1] * 0.6, 0.9)
                self.create_line(pa[0], pa[1], pb[0], pb[1], fill="#22303d")

        L, W, H = self.L / 2, self.W / 2, self.H / 2
        v = [(x, yy, z) for x in (-L, L) for yy in (-W, W) for z in (-H, H)]
        pv = [project(*pt) for pt in v]
        idx = {pt: i for i, pt in enumerate(v)}

        def vid(x, yy, z):
            return idx[(x, yy, z)]

        faces = [
            ("top", [vid(-L, -W, -H), vid(L, -W, -H), vid(L, W, -H), vid(-L, W, -H)], CUBE_ORANGE),
            ("bottom", [vid(-L, -W, H), vid(-L, W, H), vid(L, W, H), vid(L, -W, H)], "#5a3510"),
            ("front", [vid(L, -W, -H), vid(L, -W, H), vid(L, W, H), vid(L, W, -H)], "#d9d9d9"),
            ("back", [vid(-L, -W, -H), vid(-L, W, -H), vid(-L, W, H), vid(-L, -W, H)], "#6b6b6b"),
            ("right", [vid(-L, W, -H), vid(L, W, -H), vid(L, W, H), vid(-L, W, H)], "#b5651d"),
            ("left", [vid(-L, -W, -H), vid(-L, -W, H), vid(L, -W, H), vid(L, -W, -H)], "#b5651d"),
        ]
        faces.sort(key=lambda f: -sum(pv[i][2] for i in f[1]) / 4)  # far to near
        for name, ids, color in faces:
            self.create_polygon(*[c for i in ids for c in pv[i][:2]], fill=color,
                                outline="#111111", width=1.5)
            if name == "top":
                # forward arrow on the top face
                a = project(-L * 0.5, 0, -H - 0.001)
                b = project(L * 0.75, 0, -H - 0.001)
                l1 = project(L * 0.35, -W * 0.4, -H - 0.001)
                r1 = project(L * 0.35, W * 0.4, -H - 0.001)
                self.create_line(a[0], a[1], b[0], b[1], fill="#1a1a1a", width=3)
                self.create_line(l1[0], l1[1], b[0], b[1], r1[0], r1[1], fill="#1a1a1a", width=3)
            if name == "front":
                c = project(L + 0.001, 0, 0)
                self.create_text(c[0], c[1], text="FRONT", fill="#222", font=(FONT, 7, "bold"))
        self.create_text(s / 2, 12, text="View looking north", fill=MUTED, font=(FONT, 8))
        self.create_rectangle(s / 2 - 100, s + 2, s / 2 + 100, s + 26, fill="#0b0f14", outline=BORDER)
        self.create_text(s / 2, s + 14, text=f"R {roll_deg:+.0f}°   P {pitch_deg:+.0f}°   Y {yaw_deg % 360:.0f}°",
                         fill=TEXT, font=(FONT, 9, "bold"))

    def _camera(self, n, e, d):
        """NED point -> screen (x, y, depth); camera sits south of the vehicle looking north, tilted down."""
        s, tilt = self.size, math.radians(28)
        up, depth = -d, n
        up2 = up * math.cos(tilt) + depth * math.sin(tilt)
        depth2 = depth * math.cos(tilt) - up * math.sin(tilt)
        k = 6.0 / (6.0 + depth2)
        return s / 2 + e * k * s * 0.36, s / 2 - 4 - up2 * k * s * 0.36, depth2


# --------------------------------------------------------------------------- GUI

def make_tiles(parent, items, cols, bg="#141b23"):
    """Grid of small label/value tiles. Returns {key: value_label}."""
    vals = {}
    for i, (key, label) in enumerate(items):
        f = tk.Frame(parent, bg=bg, highlightthickness=1, highlightbackground=BORDER)
        f.grid(row=i // cols, column=i % cols, sticky="nsew", padx=3, pady=3)
        tk.Label(f, text=label, bg=bg, fg=MUTED, font=(FONT, 8)).pack(anchor="w", padx=8, pady=(6, 0))
        v = tk.Label(f, text="—", bg=bg, fg=TEXT, font=(FONT, 13, "bold"))
        v.pack(anchor="w", padx=8, pady=(0, 6))
        vals[key] = v
    for c in range(cols):
        parent.columnconfigure(c, weight=1, uniform="tiles")
    return vals


def when_text(t, now):
    day = time.strftime("%Y-%m-%d", time.localtime(t))
    if day == time.strftime("%Y-%m-%d", time.localtime(now)):
        prefix = "Today"
    elif day == time.strftime("%Y-%m-%d", time.localtime(now + 86400)):
        prefix = "Tomorrow"
    else:
        prefix = time.strftime("%a", time.localtime(t))
    return f"{prefix} {time.strftime('%H:%M', time.localtime(t))}"


def in_text(seconds):
    seconds = int(seconds)
    if seconds < 3600:
        return f"in {seconds // 60} min"
    return f"in {seconds // 3600} h {seconds % 3600 // 60:02d} min"


def short_sat_name(s):
    """Compact label: G06 (GPS), E20 (Galileo), C24 (BeiDou), R721 (GLONASS) or the common name."""
    if s.get("target"):
        return s["target"]
    name = s["name"]
    for pattern, prefix in ((r"PRN (\d+)", "G"), (r"GALILEO[- ](\d+)", "E"), (r"\(C(\d+)\)", "C")):
        m = re.search(pattern, name)
        if m:
            return f"{prefix}{int(m.group(1)):02d}"
    m = re.search(r"COSMOS \d+ \((\d+)\)", name)
    if m:
        return f"R{m.group(1)}"
    return name.split(" (")[0][:12]


class App(tk.Tk):
    AC_COLUMNS = [
        ("src", "Src", 34), ("kind", "Kind", 116), ("callsign", "Flight", 74), ("reg", "Reg", 66),
        ("type", "Type", 46), ("alt", "Alt ft", 64), ("dist", "km", 46), ("dir", "Dir", 44),
        ("spd", "km/h", 42), ("sqk", "Sqk", 46), ("age", "Seen", 36),
    ]
    SAT_COLUMNS = [
        ("name", "Satellite", 190), ("group", "Group", 100), ("el", "Elev°", 55), ("dir", "Direction", 80),
        ("range", "Distance km", 90), ("height", "Height km", 80), ("vis", "Visible to eye", 100),
    ]
    PASS_COLUMNS = [
        ("name", "Satellite", 130), ("when", "Rises", 120), ("in", "", 110), ("max", "Highest", 70),
        ("path", "Path", 110), ("vis", "Visible to eye", 100),
    ]
    GNSS_COLUMNS = [
        ("name", "Satellite", 220), ("sys", "System", 90), ("el", "Elev°", 60), ("dir", "Direction", 90),
        ("range", "Distance km", 100),
    ]
    TILES = [
        ("mode", "Flight mode"), ("armed", "Status"), ("batt", "Battery"),
        ("alt_rel", "Altitude (rel)"), ("alt_msl", "Altitude (MSL)"), ("climb", "Climb"),
        ("roll", "Roll"), ("pitch", "Pitch"), ("yaw", "Yaw / heading"),
        ("gs", "Ground speed"), ("gps", "GPS"), ("baro", "Pressure / temp"),
    ]
    GPS_TILES = [
        ("fix", "Fix"), ("sats", "Satellites used"), ("hdop", "HDOP / VDOP"),
        ("acc", "Accuracy (H / V)"), ("pos", "Position"), ("alt", "GPS altitude"),
        ("spd", "Speed / course"), ("gps2", "Second GPS"), ("pred", "Nav satellites overhead"),
    ]

    def __init__(self):
        super().__init__()
        self.title("ADS-B Overhead Monitor - Cube Orange+")
        self.configure(bg=BG)
        self.geometry("1600x920")
        self.minsize(1280, 760)
        try:
            self.state("zoomed")
        except tk.TclError:
            pass

        self.settings = load_settings()
        self.q = queue.Queue()
        self.link = None
        self.link_state = ("idle", "")
        self.aircraft = {}
        self.telem = {}
        self.vehicle_gps = None
        self.autopilot_seen = 0.0
        self.adsb_rx_seen = 0.0
        self.adsb_count = 0
        self.msg_times = []
        self.selected = None          # ("ac", icao) or ("sat", catalog id)
        self.att_dirty = True
        self.flash_on = False
        self.status_lines = []
        self.sats = None              # latest satellite snapshot
        self.sat_passes = []
        self.sat_status = "Satellites: starting…"
        self.sat_alerted = set()
        self.sat_primed = False       # first satellite snapshot: note what's up without alarming
        self.tune_last = {}           # tune -> last time it was played (rate limit for satellites)
        self.latched = None           # {"tune", "text", "since"} while an alarm keeps sounding
        self.online = False           # start offline: only data received by the Cube
        self.sweep_angle = 0.0        # radar beam bearing (deg)
        self.radar_geom = None
        self.radar_blips = []         # (bearing, x, y, colour) of aircraft on the radar
        self.pings = []               # (start time, x, y, colour) rings where the beam hit an aircraft
        self.player = AlarmPlayer(lambda: bool(self.settings["alarm"]),
                                  lambda: self.settings["sound_device"],
                                  lambda: float(self.settings["volume"]) / 100.0)
        self.player.start()
        self.net_time = None
        self.net_count = 0
        self.net_sources = []
        self.net_error = ""
        self.tracker = None

        self.lookup = OnlineLookup(self.q)
        self.lookup.start()
        self.feed = OnlineFeed(self.q)
        self._configure_feed()
        self.feed.start()

        self._style()
        self._build_ui()
        self._start_link()
        self._start_tracker()
        self._update_sat_tab(time.time())
        self._update_gnss(time.time())
        self.after(50, self._poll_queue)
        self.after(40, self._animate_sweep)
        self.after(70, self._draw_attitude)
        self.after(1000, self._refresh)
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    # ---------------------------------------------------------------- style
    def _style(self):
        st = ttk.Style(self)
        st.theme_use("clam")
        st.configure(".", background=PANEL, foreground=TEXT, fieldbackground=CARD,
                     bordercolor=BORDER, lightcolor=BORDER, darkcolor=BORDER, font=(FONT, 10))
        st.configure("Treeview", background=CARD, fieldbackground=CARD, foreground=TEXT,
                     rowheight=27, borderwidth=0, font=(FONT, 10))
        st.map("Treeview", background=[("selected", "#24476b")], foreground=[("selected", "white")])
        st.configure("Treeview.Heading", background=PANEL, foreground=MUTED, relief="flat",
                     font=(FONT, 9, "bold"), padding=6)
        st.map("Treeview.Heading", background=[("active", BORDER)])
        st.configure("Vertical.TScrollbar", background=PANEL, troughcolor=CARD, arrowcolor=MUTED)
        st.configure("TEntry", fieldbackground=CARD, foreground=TEXT, insertcolor=TEXT, padding=4)
        st.configure("TCombobox", fieldbackground=CARD, foreground=TEXT, background=PANEL,
                     arrowcolor=TEXT, padding=4)
        st.map("TCombobox", fieldbackground=[("readonly", CARD)], foreground=[("readonly", TEXT)])
        self.option_add("*TCombobox*Listbox.background", CARD)
        self.option_add("*TCombobox*Listbox.foreground", TEXT)
        self.option_add("*TCombobox*Listbox.selectBackground", "#24476b")
        st.configure("TCheckbutton", background=PANEL, foreground=TEXT)
        st.map("TCheckbutton", background=[("active", PANEL)])
        st.configure("Card.TCheckbutton", background=CARD, foreground=TEXT, font=(FONT, 9))
        st.map("Card.TCheckbutton", background=[("active", CARD)])
        st.configure("Accent.TButton", background=ACCENT, foreground="#071320",
                     font=(FONT, 10, "bold"), padding=(14, 6), borderwidth=0)
        st.map("Accent.TButton", background=[("active", "#6cc8ff")])
        st.configure("Ghost.TButton", background=CARD, foreground=TEXT, padding=(12, 6), borderwidth=1)
        st.map("Ghost.TButton", background=[("active", BORDER)])
        st.configure("Online.TButton", background="#1f7a4a", foreground="white", font=(FONT, 10, "bold"),
                     padding=(12, 6), borderwidth=0)
        st.map("Online.TButton", background=[("active", "#27935a")])
        st.configure("TNotebook", background=CARD, borderwidth=0, tabmargins=(0, 0, 0, 0))
        st.configure("TNotebook.Tab", background=PANEL, foreground=MUTED, padding=(16, 7),
                     font=(FONT, 10, "bold"), borderwidth=0)
        st.map("TNotebook.Tab", background=[("selected", "#24476b")], foreground=[("selected", "white")])
        st.configure("Dark.TNotebook", background=PANEL)

    # ---------------------------------------------------------------- layout
    def _build_ui(self):
        # header
        header = tk.Frame(self, bg=PANEL, height=64)
        header.pack(fill="x")
        title = tk.Frame(header, bg=PANEL)
        title.pack(side="left", padx=18, pady=10)
        tk.Label(title, text="✈  ADS-B Overhead Monitor", bg=PANEL, fg=TEXT,
                 font=(FONT, 17, "bold")).pack(anchor="w")
        self.subtitle = tk.Label(title, text="Cube Orange+ ADS-B", bg=PANEL, fg=MUTED, font=(FONT, 9))
        self.subtitle.pack(anchor="w")

        right = tk.Frame(header, bg=PANEL)
        right.pack(side="right", padx=18)
        ttk.Button(right, text="⚙  Settings", style="Ghost.TButton",
                   command=self._open_settings).pack(side="right", padx=(12, 0))
        ttk.Button(right, text="↻", width=3, style="Ghost.TButton",
                   command=self._start_link).pack(side="right", padx=(12, 0))
        self.mute_btn = ttk.Button(right, style="Ghost.TButton", command=self._toggle_alarm)
        self.mute_btn.pack(side="right", padx=(12, 0))
        self._update_mute_btn()
        self.online_btn = ttk.Button(right, command=self._toggle_online)
        self.online_btn.pack(side="right", padx=(12, 0))
        self._update_online_btn()
        self.pill_sat = Pill(right, "Sats")
        self.pill_sat.pack(side="right", padx=6)
        self.pill_net = Pill(right, "Online")
        self.pill_net.pack(side="right", padx=6)
        self.pill_adsb = Pill(right, "ADS-B")
        self.pill_adsb.pack(side="right", padx=6)
        self.pill_fc = Pill(right, "FC")
        self.pill_fc.pack(side="right", padx=6)
        self.pill_link = Pill(right, "Starting…")
        self.pill_link.pack(side="right", padx=6)

        self.link_msg = tk.Label(self, text="", bg=BG, fg=MUTED, font=(FONT, 9), anchor="w")
        self.link_msg.pack(fill="x", padx=18, pady=(6, 0))

        self.banner_frame = tk.Frame(self, bg="#1d2a38")
        self.banner_frame.pack(fill="x", padx=14, pady=8)
        self.banner = tk.Label(self.banner_frame, text="", font=(FONT, 16, "bold"), bg="#1d2a38", fg="white",
                               anchor="w", padx=18, pady=12)
        self.banner.pack(side="left", fill="x", expand=True)
        self.stop_btn = tk.Button(self.banner_frame, text="🔕  STOP ALARM", font=(FONT, 14, "bold"),
                                  bg="white", fg="#b00020", activebackground="#ffd6d6", activeforeground="#b00020",
                                  relief="flat", padx=18, pady=6, cursor="hand2", command=self._acknowledge)
        # keyboard shortcuts on the main window only (not the Settings dialog)
        self.bind("<space>", lambda e: self._acknowledge() if self.latched else None)
        self.bind("<Escape>", lambda e: self._acknowledge() if self.latched else None)

        main = tk.Frame(self, bg=BG)
        main.pack(fill="both", expand=True, padx=14, pady=(0, 14))
        main.columnconfigure(1, weight=1)
        main.rowconfigure(0, weight=1)

        # --- left: flight controller
        left = tk.Frame(main, bg=BG)
        left.grid(row=0, column=0, sticky="nsw", padx=(0, 10))
        fc = Card(left, "Flight controller - orientation")
        fc.pack(fill="x")
        inst = tk.Frame(fc.body, bg=CARD)
        inst.pack()
        self.horizon = Horizon(inst, size=210)
        self.horizon.pack(side="left", padx=(0, 8))
        self.cube_view = CubeView(inst, size=210)
        self.cube_view.pack(side="left")
        tiles = Card(left, "Flight data")
        tiles.pack(fill="x", pady=(10, 0))
        self.tile_vals = make_tiles(tiles.body, self.TILES, 3)
        msgs = Card(left, "Messages from Cube")
        msgs.pack(fill="both", expand=True, pady=(10, 0))
        self.msg_box = tk.Text(msgs.body, width=40, height=5, bg=CARD, fg=MUTED, font=("Consolas", 9),
                               relief="flat", wrap="word", highlightthickness=0)
        self.msg_box.pack(fill="both", expand=True)
        self.msg_box.configure(state="disabled")

        # --- centre: tabs
        centre = tk.Frame(main, bg=CARD, highlightthickness=1, highlightbackground=BORDER)
        centre.grid(row=0, column=1, sticky="nsew")
        self.nb = ttk.Notebook(centre)
        self.nb.pack(fill="both", expand=True, padx=8, pady=8)
        self.nb.add(self._build_aircraft_tab(self.nb), text="  ✈  Aircraft  ")
        self.nb.add(self._build_sat_tab(self.nb), text="  🛰  Satellites  ")
        self.nb.add(self._build_gps_tab(self.nb), text="  📡  GPS  ")
        self.nb.bind("<<NotebookTabChanged>>", self._tab_changed)

        # --- right: radar / sky view + details
        rightcol = tk.Frame(main, bg=BG)
        rightcol.grid(row=0, column=2, sticky="nse", padx=(10, 0))
        self.view_card = Card(rightcol, "Radar - aircraft around you")
        self.view_card.pack(fill="x")
        self.radar = tk.Canvas(self.view_card.body, width=340, height=340, bg="#0b1620", highlightthickness=0)
        self.radar.pack()
        self.legend = tk.Label(self.view_card.body, text="", bg=CARD, fg=MUTED, font=(FONT, 8), justify="left")
        self.legend.pack(anchor="w", pady=(6, 0))
        self.detail_card = Card(rightcol, "Selected")
        self.detail_card.pack(fill="both", expand=True, pady=(10, 0))
        self.detail = tk.Text(self.detail_card.body, width=40, height=14, font=("Consolas", 10),
                              bg=CARD, fg=TEXT, relief="flat", wrap="word", highlightthickness=0)
        self.detail.pack(fill="both", expand=True)
        self.detail.configure(state="disabled")
        self._update_legend()

    def _tree(self, parent, columns, height=None):
        frame = tk.Frame(parent, bg=CARD)
        tree = ttk.Treeview(frame, columns=[c[0] for c in columns], show="headings",
                            **({"height": height} if height else {}))
        for key, title_, width in columns:
            tree.heading(key, text=title_)
            tree.column(key, width=width, anchor="center", minwidth=35, stretch=True)
        vsb = ttk.Scrollbar(frame, orient="vertical", command=tree.yview)
        tree.configure(yscrollcommand=vsb.set)
        tree.pack(side="left", fill="both", expand=True)
        vsb.pack(side="right", fill="y")
        return frame, tree

    def _build_aircraft_tab(self, nb):
        tab = tk.Frame(nb, bg=CARD)
        bar = tk.Frame(tab, bg=CARD)
        bar.pack(fill="x", pady=(8, 6))
        tk.Label(bar, text="Show:", bg=CARD, fg=MUTED, font=(FONT, 9)).pack(side="left", padx=(2, 6))
        self.filter_vars = {}
        for key, (label, icon, _col) in KINDS.items():
            var = tk.BooleanVar(value=self.settings[f"show_{key}"])
            self.filter_vars[key] = var
            ttk.Checkbutton(bar, text=f"{icon} {label}", variable=var, style="Card.TCheckbutton",
                            command=self._filters_changed).pack(side="left", padx=4)
        self.show_net_var = tk.BooleanVar(value=self.settings["show_net"])
        ttk.Checkbutton(bar, text="🌐 Online", variable=self.show_net_var, style="Card.TCheckbutton",
                        command=self._filters_changed).pack(side="left", padx=(12, 4))

        frame, self.tree = self._tree(tab, self.AC_COLUMNS)
        frame.pack(fill="both", expand=True)
        self.tree.tag_configure("net", foreground="#9fb3c8")
        self.tree.tag_configure("stale", foreground="#5c6773")
        self.tree.tag_configure("special", background="#3d2459", foreground="#e9d5ff", font=(FONT, 10, "bold"))
        self.tree.tag_configure("new", background="#4a1f26", foreground="#ffc2c2", font=(FONT, 10, "bold"))
        self.tree.tag_configure("emergency", background="#7a1020", foreground="white", font=(FONT, 10, "bold"))
        self.tree.bind("<<TreeviewSelect>>", lambda e: self._on_select(self.tree, "ac"))
        self.empty_hint = tk.Label(frame, bg=CARD, fg=MUTED, font=(FONT, 11), justify="center",
                                   text="No aircraft yet.\n\nAircraft appear here when the Cube's ADS-B receiver hears "
                                        "them,\nor when the online feed reports them near you.")
        self.empty_hint.place(relx=0.5, rely=0.4, anchor="center")
        self.ac_footer = tk.Label(tab, text="", bg=CARD, fg=MUTED, font=(FONT, 9), anchor="w")
        self.ac_footer.pack(fill="x", pady=(6, 0))
        return tab

    def _build_sat_tab(self, nb):
        tab = tk.Frame(nb, bg=CARD)
        top = tk.Frame(tab, bg=CARD)
        top.pack(fill="x", pady=(8, 2))
        self.sat_summary = tk.Label(top, text="", bg=CARD, fg=TEXT, font=(FONT, 11, "bold"), anchor="w")
        self.sat_summary.pack(side="left")
        self.sat_show_starlink = tk.BooleanVar(value=self.settings["sat_show_starlink"])
        ttk.Checkbutton(top, text="List Starlink", variable=self.sat_show_starlink, style="Card.TCheckbutton",
                        command=self._sat_filters_changed).pack(side="right", padx=4)
        self.sat_show_gnss = tk.BooleanVar(value=self.settings["sat_show_gnss"])
        ttk.Checkbutton(top, text="List GPS/GNSS", variable=self.sat_show_gnss, style="Card.TCheckbutton",
                        command=self._sat_filters_changed).pack(side="right", padx=4)
        self.sat_status_lbl = tk.Label(tab, text="", bg=CARD, fg=MUTED, font=(FONT, 9), anchor="w")
        self.sat_status_lbl.pack(fill="x")

        tk.Label(tab, text="UPCOMING PASSES  ·  ISS, Tiangong, Hubble (next 24 h, higher than 10°)", bg=CARD,
                 fg=MUTED, font=(FONT, 9, "bold"), anchor="w").pack(fill="x", pady=(10, 4))
        pframe, self.pass_tree = self._tree(tab, self.PASS_COLUMNS, height=4)
        pframe.pack(fill="x")
        self.pass_tree.tag_configure("now", background="#3d2459", foreground="#e9d5ff", font=(FONT, 10, "bold"))
        self.pass_tree.tag_configure("vis", foreground="#ffd43b")

        tk.Label(tab, text="ABOVE THE HORIZON NOW  ·  highest first", bg=CARD, fg=MUTED,
                 font=(FONT, 9, "bold"), anchor="w").pack(fill="x", pady=(10, 4))
        sframe, self.sat_tree = self._tree(tab, self.SAT_COLUMNS)
        sframe.pack(fill="both", expand=True)
        self.sat_tree.tag_configure("target", background="#3d2459", foreground="#e9d5ff", font=(FONT, 10, "bold"))
        self.sat_tree.tag_configure("vis", foreground="#ffd43b")
        self.sat_tree.tag_configure("dim", foreground="#7f8c99")
        self.sat_tree.bind("<<TreeviewSelect>>", lambda e: self._on_select(self.sat_tree, "sat"))
        return tab

    def _build_gps_tab(self, nb):
        tab = tk.Frame(nb, bg=CARD)
        tk.Label(tab, text="CUBE GPS RECEIVER", bg=CARD, fg=MUTED, font=(FONT, 9, "bold"),
                 anchor="w").pack(fill="x", pady=(10, 4))
        tf = tk.Frame(tab, bg=CARD)
        tf.pack(fill="x")
        self.gps_vals = make_tiles(tf, self.GPS_TILES, 3)
        self.gps_note = tk.Label(tab, text="", bg=CARD, fg=ORANGE, font=(FONT, 9), anchor="w", justify="left",
                                 wraplength=600)
        self.gps_note.pack(fill="x", pady=(6, 0))
        tk.Label(tab, text="NAVIGATION SATELLITES ABOVE YOU NOW  ·  predicted from their orbits", bg=CARD,
                 fg=MUTED, font=(FONT, 9, "bold"), anchor="w").pack(fill="x", pady=(12, 4))
        self.gnss_counts = tk.Label(tab, text="", bg=CARD, fg=TEXT, font=(FONT, 10, "bold"), anchor="w")
        self.gnss_counts.pack(fill="x", pady=(0, 4))
        gframe, self.gnss_tree = self._tree(tab, self.GNSS_COLUMNS)
        gframe.pack(fill="both", expand=True)
        for name in ("GPS", "Galileo", "GLONASS", "BeiDou"):
            self.gnss_tree.tag_configure(name, foreground=SAT_COLORS[name])
        self.gnss_tree.bind("<<TreeviewSelect>>", lambda e: self._on_select(self.gnss_tree, "sat"))
        tk.Label(tab, text="ArduPilot reports how many satellites the Cube's GPS is using, not which ones - "
                           "this list shows which ones are overhead.", bg=CARD, fg=MUTED, font=(FONT, 8),
                 anchor="w").pack(fill="x", pady=(6, 0))
        return tab

    # ---------------------------------------------------------------- settings dialog
    def _open_settings(self):
        win = tk.Toplevel(self)
        win.title("Settings")
        win.configure(bg=PANEL)
        win.transient(self)
        win.grab_set()
        win.resizable(False, False)
        s = self.settings
        pad = {"padx": 10, "pady": 5}
        nb = ttk.Notebook(win)
        nb.pack(fill="both", expand=True, padx=10, pady=10)
        pages = {}
        for name in ("Connection", "Location", "Alarms & tunes", "Online & satellites"):
            f = tk.Frame(nb, bg=PANEL)
            nb.add(f, text=f"  {name}  ")
            pages[name] = f

        def label(page, row, text):
            tk.Label(page, text=text, bg=PANEL, fg=TEXT, font=(FONT, 10)).grid(row=row, column=0, sticky="w", **pad)

        def check(page, row, text, value):
            var = tk.BooleanVar(value=value)
            ttk.Checkbutton(page, text=text, variable=var).grid(row=row, column=0, columnspan=3, sticky="w", **pad)
            return var

        def entry(page, row, text, value, width=10):
            label(page, row, text)
            var = tk.StringVar(value=value)
            ttk.Entry(page, textvariable=var, width=width).grid(row=row, column=1, sticky="w", **pad)
            return var

        p = pages["Connection"]
        label(p, 0, "Mode")
        mode_var = tk.StringVar(value=s["connection"])
        ttk.Combobox(p, textvariable=mode_var, state="readonly", width=36,
                     values=[MODE_AUTO, MODE_COM, MODE_UDP]).grid(row=0, column=1, columnspan=2, sticky="w", **pad)
        label(p, 1, "COM port (blank = auto-detect)")
        com_var = tk.StringVar(value=s["com_port"])
        com_box = ttk.Combobox(p, textvariable=com_var, width=36, values=[""] + list_com_ports())
        com_box.grid(row=1, column=1, sticky="w", **pad)
        ttk.Button(p, text="↻", width=3, style="Ghost.TButton",
                   command=lambda: com_box.configure(values=[""] + list_com_ports())).grid(row=1, column=2)
        baud_var = entry(p, 2, "Baud", str(s["baud"]))
        udp_var = entry(p, 3, "UDP port (Mission Planner)", str(s["udp_port"]))
        share_var = check(p, 4, "Share the Cube with Mission Planner (connect MP via UDP 14550)", s["share_with_mp"])

        p = pages["Location"]
        latlon_var = entry(p, 0, "Our location (lat, lon)", f"{s['home_lat']}, {s['home_lon']}", 28)
        alt_var = entry(p, 1, "Ground altitude (m MSL)", str(s["home_alt_m"]))
        lookup_var = check(p, 2, "Look up registration / type / route online (adsbdb.com)", s["online_lookup"])

        p = pages["Alarms & tunes"]
        label(p, 0, "Play alarms on")
        default_label = f"Windows default  ({default_output_name()})"
        dev_var = tk.StringVar(value=s["sound_device"] or default_label)
        ttk.Combobox(p, textvariable=dev_var, state="readonly", width=44,
                     values=[default_label] + output_devices()).grid(row=0, column=1, sticky="w", **pad)
        label(p, 1, "Volume")
        vol_var = tk.IntVar(value=int(s["volume"]))
        vol_row = tk.Frame(p, bg=PANEL)
        vol_row.grid(row=1, column=1, sticky="w", **pad)
        tk.Scale(vol_row, from_=10, to=100, orient="horizontal", variable=vol_var, length=200, bg=PANEL, fg=TEXT,
                 highlightthickness=0, troughcolor=CARD, showvalue=True).pack(side="left")
        sound_msg = tk.Label(p, text="", bg=PANEL, fg=MUTED, font=(FONT, 9))

        def apply_sound():
            s["sound_device"] = "" if dev_var.get() == default_label else dev_var.get()
            s["volume"] = int(vol_var.get())

        def test_sound():
            apply_sound()
            self.player.play("plane", preview=True)
            win.after(2500, lambda: sound_msg.configure(
                text=self.player.last_error or "Played the airplane alarm - did you hear it?",
                fg=RED if self.player.last_error else GREEN))

        ttk.Button(vol_row, text="🔊  Test sound", style="Accent.TButton", command=test_sound).pack(side="left", padx=12)
        sound_msg.grid(row=2, column=1, sticky="w", padx=10)
        alarm_var = check(p, 3, "Alarm sound on (master switch - same as the bell button)", s["alarm"])
        latch_ac_var = check(p, 4, "Aircraft alarms keep sounding until STOP ALARM is pressed", s["latch_aircraft"])
        latch_sat_var = check(p, 5, "Satellite alarms keep sounding until STOP ALARM is pressed", s["latch_sats"])
        rad_var = entry(p, 6, "Alarm range for Cube aircraft (km, 0 = unlimited)", f"{s['alert_km']:g}")
        net_alarm_var = entry(p, 7, "Alarm for online-feed aircraft within (km, 0 = never)", f"{s['net_alarm_km']:g}")
        sat_el_var = entry(p, 8, "Satellite alarm when higher than (degrees)", f"{s['sat_alarm_el']:g}")
        tunes = tk.Frame(p, bg=PANEL)
        tunes.grid(row=9, column=0, columnspan=3, sticky="w", padx=10, pady=(10, 4))
        tune_vars = {}
        for col, (title_, keys) in enumerate((("AIRCRAFT", AIRCRAFT_TUNES), ("SATELLITES", SAT_TUNES))):
            tk.Label(tunes, text=title_, bg=PANEL, fg=ACCENT, font=(FONT, 9, "bold")).grid(
                row=0, column=col * 2, columnspan=2, sticky="w", pady=(0, 4))
            for row, key in enumerate(keys, start=1):
                var = tk.BooleanVar(value=s[f"tune_{key}"])
                tune_vars[key] = var
                ttk.Checkbutton(tunes, text=TUNES[key][0], variable=var).grid(
                    row=row, column=col * 2, sticky="w", padx=(0, 4), pady=2)
                ttk.Button(tunes, text="▶", width=3, style="Ghost.TButton",
                           command=lambda k=key: (apply_sound(), self.player.play(k, preview=True))).grid(
                    row=row, column=col * 2 + 1, padx=(0, 30), pady=2)
        tk.Label(p, text="▶ plays the tune. Starlink and GPS satellites rise every few minutes, "
                         "so their tunes start switched off.", bg=PANEL, fg=MUTED, font=(FONT, 8)).grid(
            row=10, column=0, columnspan=3, sticky="w", padx=10, pady=(4, 8))

        p = pages["Online & satellites"]
        net_var = check(p, 0, "Show aircraft from online ADS-B networks (adsb.fi, adsb.lol)", s["net_feed"])
        net_rad_var = entry(p, 1, "Online feed radius (km, max 460)", f"{s['net_radius_km']:g}")
        sat_var = check(p, 2, "Track satellites (orbits from celestrak.org, refreshed every 12 h)", s["satellites"])
        starlink_var = check(p, 3, "Include Starlink (~10,000 satellites)", s["sat_starlink"])

        err = tk.Label(win, text="", bg=PANEL, fg=RED, font=(FONT, 9))
        err.pack(anchor="w", padx=12)

        def save():
            try:
                lat_s, lon_s = [x.strip() for x in latlon_var.get().replace(";", ",").split(",")[:2]]
                lat, lon = float(lat_s), float(lon_s)
                if not (-90 <= lat <= 90 and -180 <= lon <= 180):
                    raise ValueError
                new = {
                    "connection": mode_var.get(), "com_port": com_var.get(),
                    "baud": int(baud_var.get()), "udp_port": int(udp_var.get()),
                    "share_with_mp": share_var.get(), "home_lat": lat, "home_lon": lon,
                    "home_alt_m": float(alt_var.get() or 0),
                    "alert_km": max(0.0, float(rad_var.get() or 0)),
                    "alarm": alarm_var.get(), "latch_aircraft": latch_ac_var.get(),
                    "latch_sats": latch_sat_var.get(),
                    "online_lookup": lookup_var.get(),
                    "net_feed": net_var.get(),
                    "net_radius_km": min(460.0, max(5.0, float(net_rad_var.get() or 150))),
                    "net_alarm_km": max(0.0, float(net_alarm_var.get() or 0)),
                    "satellites": sat_var.get(), "sat_starlink": starlink_var.get(),
                    "sat_alarm_el": float(sat_el_var.get() or 10),
                }
                new.update({f"tune_{k}": v.get() for k, v in tune_vars.items()})
                new["sound_device"] = "" if dev_var.get() == default_label else dev_var.get()
                new["volume"] = int(vol_var.get())
            except ValueError:
                err.configure(text="Check the values - location as 'lat, lon', e.g. 29.8543, 77.8880")
                return
            restart_link = any(new[k] != s[k] for k in ("connection", "com_port", "baud", "udp_port", "share_with_mp"))
            restart_sats = any(new[k] != s[k] for k in ("satellites", "sat_starlink"))
            moved = any(new[k] != s[k] for k in ("home_lat", "home_lon", "home_alt_m"))
            s.update(new)
            save_settings(s)
            self._update_mute_btn()
            if not s["alarm"] and self.latched:
                self._acknowledge()
            self._configure_feed()
            if restart_sats:
                self._start_tracker()
            elif moved and self.tracker:
                self.tracker.set_observer(s["home_lat"], s["home_lon"], s["home_alt_m"])
            win.destroy()
            if restart_link:
                self._start_link()

        btns = tk.Frame(win, bg=PANEL)
        btns.pack(fill="x", padx=10, pady=(4, 12))
        ttk.Button(btns, text="Cancel", style="Ghost.TButton", command=win.destroy).pack(side="right", padx=(8, 0))
        ttk.Button(btns, text="Save", style="Accent.TButton", command=save).pack(side="right")

    # ---------------------------------------------------------------- background services
    def _start_link(self):
        if self.link:
            self.link.stop()
            self.link.join(timeout=3)
        self.autopilot_seen = self.adsb_rx_seen = 0.0
        self.link = Link(self.settings, self.q)
        self.link.start()

    def _net_on(self):
        return self.online and bool(self.settings["net_feed"])

    def _lookup_on(self):
        return self.online and bool(self.settings["online_lookup"])

    def _toggle_online(self):
        """Offline (default): only what the Cube receives. Online: also the internet-based data
        (online aircraft feed, aircraft details lookup, satellites)."""
        self.online = not self.online
        log_event("Switched ONLINE" if self.online else "Switched OFFLINE (Cube data only)")
        self._update_online_btn()
        self._configure_feed()
        self._start_tracker()
        if self.online:
            if self.settings["online_lookup"]:
                for a in self.aircraft.values():
                    if a.last_cube and not a.info:
                        self.lookup.request(a.icao_hex, a.callsign)
        else:
            for icao in [k for k, a in self.aircraft.items() if a.last_cube is None]:
                del self.aircraft[icao]  # drop aircraft that only came from the internet
            self.net_time, self.net_count, self.net_error = None, 0, ""
            self.sat_alerted, self.sat_primed = set(), False
        now = time.time()
        self._update_table(now)
        self._update_sat_tab(now)
        self._update_gnss(now)
        self._update_link_pill()
        self._draw_view()

    def _update_online_btn(self):
        if self.online:
            self.online_btn.configure(text="🌐  Online", style="Online.TButton")
        else:
            self.online_btn.configure(text="📴  Offline · Go online", style="Ghost.TButton")

    def _configure_feed(self):
        s = self.settings
        self.feed.configure(float(s["home_lat"]), float(s["home_lon"]), float(s["net_radius_km"]),
                            self._net_on())

    def _start_tracker(self):
        if self.tracker:
            self.tracker.stop()
            self.tracker = None
        self.sats, self.sat_passes = None, []
        if not self.online:
            self.sat_status = "Offline - press  📴 Offline · Go online  at the top to show satellites"
            return
        if SatTracker is None:
            self.sat_status = "Satellites disabled: run  pip install numpy sgp4"
            return
        if not self.settings["satellites"]:
            self.sat_status = "Satellite tracking is off (Settings > Online & satellites)"
            return
        s = self.settings
        self.tracker = SatTracker(self.q, include_starlink=bool(s["sat_starlink"]))
        self.tracker.set_observer(float(s["home_lat"]), float(s["home_lon"]), float(s["home_alt_m"]))
        self.tracker.start()

    def home(self):
        s = self.settings
        if s["home_lat"] != "" and s["home_lon"] != "":
            return float(s["home_lat"]), float(s["home_lon"]), float(s["home_alt_m"])
        g = self.vehicle_gps
        if g is not None and g.fix_type >= 3:
            return g.lat / 1e7, g.lon / 1e7, g.alt / 1000.0
        return None

    def alert_m(self):
        km = float(self.settings["alert_km"] or 0)
        return None if km <= 0 else km * 1000.0

    # ---------------------------------------------------------------- events
    def _poll_queue(self):
        try:
            for _ in range(500):
                kind, data = self.q.get_nowait()
                if kind == "msg":
                    self._on_msg(data)
                elif kind == "link":
                    if data != self.link_state:
                        log_event(f"[{data[0]}] {data[1]}")
                    self.link_state = data
                    self._update_link_pill()
                elif kind == "lookup":
                    icao_hex, info = data
                    for a in self.aircraft.values():
                        if a.icao_hex == icao_hex:
                            a.info.update(info)
                elif kind == "net":
                    self._on_net(*data)
                elif kind == "net_error":
                    self.net_error = data
                elif kind == "sats":
                    self._on_sats(data)
                elif kind == "sat_passes":
                    if self.online:
                        self.sat_passes = data
                        self._update_passes()
                elif kind == "sat_status":
                    self.sat_status = data
        except queue.Empty:
            pass
        self.after(50, self._poll_queue)

    def _on_msg(self, msg):
        t = msg.get_type()
        now = time.time()
        self.msg_times.append(now)
        if t == "ADSB_VEHICLE":
            self._on_adsb(msg)
            return
        if t == "HEARTBEAT":
            if msg.type == mavutil.mavlink.MAV_TYPE_ADSB:
                self.adsb_rx_seen = now
            elif msg.autopilot != mavutil.mavlink.MAV_AUTOPILOT_INVALID:
                self.autopilot_seen = now
                self.telem["HEARTBEAT"] = msg
            return
        if t == "STATUSTEXT":
            self.status_lines.append(f"{time.strftime('%H:%M:%S')}  {msg.text}")
            self.status_lines = self.status_lines[-40:]
            self.msg_box.configure(state="normal")
            self.msg_box.delete("1.0", "end")
            self.msg_box.insert("1.0", "\n".join(reversed(self.status_lines)))
            self.msg_box.configure(state="disabled")
            return
        if t == "GPS_RAW_INT":
            self.vehicle_gps = msg
        if t == "ATTITUDE":
            self.att_dirty = True
        self.telem[t] = msg

    def _on_adsb(self, msg):
        self.adsb_count += 1
        self.adsb_rx_seen = time.time()
        a = self.aircraft.get(msg.ICAO_address)
        if a is None:
            a = self.aircraft[msg.ICAO_address] = Aircraft(msg.ICAO_address)
        old_callsign = a.callsign
        a.update(msg)
        a.compute_geometry(self.home())
        log_adsb(a)
        if self._lookup_on() and (a.msgs == 1 or a.callsign != old_callsign):
            self.lookup.request(a.icao_hex, a.callsign)

    def _on_net(self, fetched_at, records, sources):
        if not self._net_on():
            return
        self.net_time, self.net_count, self.net_sources, self.net_error = fetched_at, len(records), sources, ""
        for rec in records:
            try:
                icao = int(rec["hex"].lstrip("~"), 16)
            except ValueError:
                continue
            if rec["hex"].startswith("~"):
                continue  # TIS-B / non-ICAO addresses
            a = self.aircraft.get(icao)
            new = a is None
            if new:
                a = self.aircraft[icao] = Aircraft(icao)
                a.last_seen = 0.0
            old_callsign = a.callsign
            a.update_from_net(rec, fetched_at)
            if self._lookup_on() and a.callsign and (new or a.callsign != old_callsign):
                self.lookup.request(a.icao_hex, a.callsign)

    @staticmethod
    def _sat_tune(s):
        if s["target"]:
            return {"ISS": "iss", "Tiangong": "tiangong", "Hubble": "hubble"}[s["target"]]
        return {"bright": "bright", "weather": "weather", "starlink": "starlink", "gnss": "gnss"}.get(s["kind"])

    def _on_sats(self, snap):
        if not self.online:
            return  # late message from a tracker that was just stopped
        self.sats = snap
        now = time.time()
        min_el = float(self.settings["sat_alarm_el"])
        up = {s["id"]: s for s in snap["above"] if s["el"] >= min_el}
        risen = [s for sid, s in up.items() if sid not in self.sat_alerted]
        # re-arm once a satellite drops 2° below the alarm elevation (no flapping at the threshold)
        el_now = {s["id"]: s["el"] for s in snap["above"]}
        self.sat_alerted = {sid for sid in self.sat_alerted if el_now.get(sid, -90) >= min_el - 2} | set(up)
        if not self.sat_primed:
            self.sat_primed = True  # don't alarm for everything that's already up when the app starts
            risen = []
        by_tune = {}
        for s in risen:
            tune = self._sat_tune(s)
            if tune and self.settings[f"tune_{tune}"]:
                by_tune.setdefault(tune, []).append(s)
        for tune, sats in sorted(by_tune.items(), key=lambda kv: TUNE_PRIORITY.index(kv[0])):
            s = max(sats, key=lambda x: x["el"])
            name = s["target"] or s["name"]
            more = f" (+{len(sats) - 1} more)" if len(sats) > 1 else ""
            text = (f"🛰  {name}{more} rising: {s['el']:.0f}° {compass(s['az'])}, {s['height']:,.0f} km up"
                    + ("  ·  visible - look up!" if s["visible"] else ""))
            if tune in ("starlink", "gnss", "bright", "weather") and now - self.tune_last.get(tune, 0) < 20:
                continue  # these rise often - at most one tune per group every 20 s
            self.tune_last[tune] = now
            log_event(f"SATELLITE {TUNES[tune][0]}: {name}{more} {s['el']:.0f}° {compass(s['az'])}, "
                      f"{s['height']:.0f} km up{' (visible)' if s['visible'] else ''}")
            self._alarm(tune, text, latch=self.settings["latch_sats"])
        self._update_sat_tab(now)
        self._update_gnss(now)
        if self.nb.index("current") != 0:
            self._draw_view()
            self._show_detail()

    def _on_select(self, tree, kind):
        sel = tree.selection()
        if not sel:
            return
        iid = sel[0]
        self.selected = ("ac", int(iid, 16)) if kind == "ac" else ("sat", iid[1:])
        self._show_detail()
        self._draw_view()

    def _tab_changed(self, _event=None):
        idx = self.nb.index("current")
        titles = ["Radar - aircraft around you", "Sky view - satellites above you",
                  "Sky view - navigation satellites"]
        self.view_card.title_label.configure(text=titles[idx].upper())
        if (idx == 0) != (self.selected is not None and self.selected[0] == "ac"):
            self.selected = None
        self._update_legend()
        self._draw_view()
        self._show_detail()

    def _filters_changed(self):
        for key, var in self.filter_vars.items():
            self.settings[f"show_{key}"] = var.get()
        self.settings["show_net"] = self.show_net_var.get()
        save_settings(self.settings)
        self._update_table(time.time())
        self._draw_view()

    def _sat_filters_changed(self):
        self.settings["sat_show_starlink"] = self.sat_show_starlink.get()
        self.settings["sat_show_gnss"] = self.sat_show_gnss.get()
        save_settings(self.settings)
        self._update_sat_tab(time.time())
        self._draw_view()

    def _shown(self, a):
        if not self.settings[f"show_{a.kind}"]:
            return False
        if a.last_cube is None:  # came only from the internet
            return self._net_on() and self.settings["show_net"]
        return True

    # ---------------------------------------------------------------- attitude (fast loop)
    def _draw_attitude(self):
        if self.att_dirty:
            self.att_dirty = False
            att = self.telem.get("ATTITUDE")
            roll = math.degrees(att.roll) if att else 0.0
            pitch = math.degrees(att.pitch) if att else 0.0
            yaw = math.degrees(att.yaw) % 360 if att else 0.0
            self.horizon.draw(roll, pitch, yaw)
            self.cube_view.draw(roll, pitch, yaw)
            if att:
                self.tile_vals["roll"].configure(text=f"{roll:+.1f}°")
                self.tile_vals["pitch"].configure(text=f"{pitch:+.1f}°")
                self.tile_vals["yaw"].configure(text=f"{yaw:.0f}° {compass(yaw)}")
        self.after(70, self._draw_attitude)

    # ---------------------------------------------------------------- slow refresh (1 Hz)
    def _refresh(self):
        now = time.time()
        home = self.home()
        alert_m = self.alert_m()
        net_alarm_m = float(self.settings["net_alarm_km"] or 0) * 1000.0

        for icao in [k for k, a in self.aircraft.items() if now - a.last_seen > REMOVE_S]:
            del self.aircraft[icao]
            if self.selected == ("ac", icao):
                self.selected = None

        newly = []
        for a in self.aircraft.values():
            a.compute_geometry(home)
            if a.cube_recent(now):
                a.in_range = alert_m is None or (a.dist_m is not None and a.dist_m <= alert_m)
            elif a.net_recent(now) and self._net_on():
                a.in_range = net_alarm_m > 0 and a.dist_m is not None and a.dist_m <= net_alarm_m
            else:
                a.in_range = False
            if a.in_range and not a.alerted and self._shown(a):
                a.alerted = True
                a.detected_at = now
                newly.append(a)
                log_event(f"DETECTED [{a.source}] {KIND_NAMES[a.kind]} {a.callsign or '-'} ({a.icao_hex})  "
                          + self._summary(a))
            elif not a.in_range and a.dist_m is not None:
                limit = alert_m if a.cube_recent(now) else (net_alarm_m or None)
                if limit and a.dist_m > limit * 1.3:
                    a.alerted = False  # re-arm after it leaves the alarm range

        self._update_table(now)
        self._update_banner(now)
        self._update_link_pill()
        self._update_tiles(now)
        self._update_gps_tab(now)
        if self.nb.index("current") == 0:
            self._draw_view()
        self._show_detail()

        # emergency squawks alarm even if the aircraft was already detected
        for a in self.aircraft.values():
            if a.squawk in SPECIAL_SQUAWKS and not getattr(a, "emerg_alerted", False) and self._shown(a):
                a.emerg_alerted = True
                if a not in newly:
                    newly.append(a)

        if newly:
            newly.sort(key=lambda a: TUNE_PRIORITY.index(self._aircraft_tune(a)))
            for a in newly:
                tune = self._aircraft_tune(a)
                label = SPECIAL_SQUAWKS[a.squawk] if tune == "emergency" else KIND_NAMES[a.kind]
                icon = "⚠" if tune == "emergency" else KINDS[a.kind][1]
                src = "  [online feed]" if a.source == "Net" else ""
                self._alarm(tune, f"{icon}  {label.upper()}:  {self._summary(a)}{src}",
                            latch=self.settings["latch_aircraft"])
            if self.nb.index("current") == 0:
                self.tree.selection_set(newly[0].icao_hex)
        self._update_stop_btn()
        self.after(1000, self._refresh)

    @staticmethod
    def _aircraft_tune(a):
        if a.squawk in SPECIAL_SQUAWKS:
            return "emergency"
        if a.kind != "plane":
            return a.kind
        return "net" if a.source == "Net" else "plane"

    # ---------------------------------------------------------------- alarm
    def _toggle_alarm(self):
        self.settings["alarm"] = not self.settings["alarm"]
        save_settings(self.settings)
        self._update_mute_btn()
        if not self.settings["alarm"]:
            self._acknowledge()

    def _update_mute_btn(self):
        self.mute_btn.configure(text="🔔  Alarm on" if self.settings["alarm"] else "🔕  Alarm off")

    def _alarm(self, tune, text, latch):
        """Sound the tune for a detection. latch=True keeps it sounding until STOP ALARM is pressed."""
        if not self.settings.get(f"tune_{tune}", True):
            return
        self._flash_taskbar()
        if not self.settings["alarm"]:
            return  # master switch off: banner and log only
        if latch:
            self.player.latch(tune)
            if self.latched is None or TUNE_PRIORITY.index(tune) < TUNE_PRIORITY.index(self.latched["tune"]):
                self.latched = {"tune": tune, "text": text, "since": time.time()}
            self.latched["count"] = self.latched.get("count", 0) + 1
            self._update_stop_btn()
        else:
            self.player.play(tune)

    def _acknowledge(self):
        if self.latched:
            log_event(f"ALARM stopped by user after {time.time() - self.latched['since']:.0f} s "
                      f"({TUNES[self.latched['tune']][0]})")
        self.latched = None
        self.player.acknowledge()
        self._update_stop_btn()
        self._update_banner(time.time())

    def _update_stop_btn(self):
        if self.latched and not self.stop_btn.winfo_ismapped():
            self.stop_btn.pack(side="right", padx=12, pady=8)
        elif not self.latched and self.stop_btn.winfo_ismapped():
            self.stop_btn.pack_forget()

    def _flash_taskbar(self):
        """Flash the taskbar button so the alert is noticed when the window is in the background."""
        try:
            import ctypes
            from ctypes import wintypes

            class FLASHWINFO(ctypes.Structure):
                _fields_ = [("cbSize", wintypes.UINT), ("hwnd", wintypes.HWND), ("dwFlags", wintypes.DWORD),
                            ("uCount", wintypes.UINT), ("dwTimeout", wintypes.DWORD)]

            hwnd = ctypes.windll.user32.GetParent(self.winfo_id())
            info = FLASHWINFO(ctypes.sizeof(FLASHWINFO), hwnd, 0x3 | 0xC, 0, 0)  # FLASHW_ALL | TIMERNOFG
            ctypes.windll.user32.FlashWindowEx(ctypes.byref(info))
        except Exception:  # noqa: BLE001
            pass

    # ---------------------------------------------------------------- status
    def _update_link_pill(self):
        now = time.time()
        state, text = self.link_state
        colors = {"direct": GREEN, "mirror": GREEN, "connecting": ORANGE, "searching": ORANGE,
                  "mirror_wait": ORANGE, "busy": RED, "lost": RED, "error": RED}
        labels = {"direct": "USB", "mirror": "Via MP", "connecting": "Connecting",
                  "searching": "No Cube", "mirror_wait": "Waiting MP",
                  "busy": "Port busy", "lost": "Reconnecting", "error": "Error"}
        self.pill_link.set(labels.get(state, "Starting"), colors.get(state, MUTED))
        self.link_msg.configure(text=text, fg=colors.get(state, MUTED) if state not in ("direct", "mirror") else MUTED)
        fc_ok = now - self.autopilot_seen < 4
        self.pill_fc.set("FC", GREEN if fc_ok else MUTED)
        rx_ok = now - self.adsb_rx_seen < 4
        self.pill_adsb.set(f"ADS-B {self.adsb_count}", GREEN if rx_ok else MUTED)
        if not self.online:
            self.pill_net.set("Internet off", MUTED)
        elif not self._net_on():
            self.pill_net.set("Online feed off", MUTED)
        elif self.net_time and now - self.net_time < 40:
            self.pill_net.set(f"Online {self.net_count}", GREEN)
        else:
            self.pill_net.set("Online", RED if self.net_error else ORANGE)
        if self.sats and now - self.sats["time"] < 10:
            self.pill_sat.set(f"Sats {len(self.sats['above'])}", GREEN)
        else:
            self.pill_sat.set("Sats", MUTED if not (self.online and self.settings["satellites"]) else ORANGE)
        hb = self.telem.get("HEARTBEAT")
        if hb and fc_ok:
            self.subtitle.configure(text=f"Cube Orange+  ·  {mavutil.mode_string_v10(hb)}  ·  "
                                         f"system {hb.get_srcSystem()}")

    def _update_tiles(self, now):
        t = self.tile_vals
        self.msg_times = [x for x in self.msg_times if now - x < 1.0]
        fresh = now - self.autopilot_seen < 4
        hb = self.telem.get("HEARTBEAT")
        if hb and fresh:
            t["mode"].configure(text=mavutil.mode_string_v10(hb), fg=ACCENT)
            armed = hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            t["armed"].configure(text="ARMED" if armed else "Disarmed", fg=RED if armed else GREEN)
        else:
            t["mode"].configure(text="—", fg=MUTED)
            t["armed"].configure(text="No link", fg=MUTED)
        ss = self.telem.get("SYS_STATUS")
        if ss:
            v = ss.voltage_battery / 1000.0
            pct = f"  {ss.battery_remaining}%" if ss.battery_remaining >= 0 else ""
            t["batt"].configure(text=f"{v:.2f} V{pct}", fg=ORANGE if v < 5 else TEXT)
        gp = self.telem.get("GLOBAL_POSITION_INT")
        if gp:
            t["alt_rel"].configure(text=f"{gp.relative_alt / 1000:.2f} m")
        hud = self.telem.get("VFR_HUD")
        if hud:
            t["alt_msl"].configure(text=f"{hud.alt:.1f} m")
            t["climb"].configure(text=f"{hud.climb:+.2f} m/s")
            t["gs"].configure(text=f"{hud.groundspeed:.1f} m/s")
        g = self.telem.get("GPS_RAW_INT")
        if g:
            fix = GPS_FIX.get(g.fix_type, str(g.fix_type))
            t["gps"].configure(text=f"{fix} · {g.satellites_visible} sat",
                               fg=GREEN if g.fix_type >= 3 else ORANGE)
        sp = self.telem.get("SCALED_PRESSURE")
        if sp:
            t["baro"].configure(text=f"{sp.press_abs:.0f} hPa · {sp.temperature / 100:.0f}°C")

    # ---------------------------------------------------------------- aircraft tab
    def _sorted_aircraft(self, only_shown=True):
        rows = [a for a in self.aircraft.values() if a.last_seen > 0 and (not only_shown or self._shown(a))]
        return sorted(rows, key=lambda a: (a.dist_m is None, a.dist_m or 0, -a.last_seen))

    def _update_table(self, now):
        rows = self._sorted_aircraft()
        if rows:
            self.empty_hint.place_forget()
        else:
            self.empty_hint.place(relx=0.5, rely=0.4, anchor="center")
        existing = set(self.tree.get_children())
        wanted = []
        for a in rows:
            iid = a.icao_hex
            wanted.append(iid)
            label, icon, _col = KINDS[a.kind]
            if a.on_ground and a.source == "Net":
                alt = "ground"
            else:
                alt = f"{a.alt_m * 3.28084:,.0f}" if a.alt_m is not None else "-"
            values = (
                {"Cube": "📡", "Net": "🌐", "Cube+Net": "📡🌐"}[a.source],
                f"{icon} {KIND_NAMES[a.kind]}",
                a.callsign or iid,
                a.info.get("Registration", ""),
                a.info.get("ICAO type") or (a.info.get("Type") or "")[:8],
                alt,
                f"{a.dist_m / 1000:.1f}" if a.dist_m is not None else "-",
                compass(a.bearing) if a.bearing is not None else "-",
                f"{a.speed_ms * 3.6:.0f}" if a.speed_ms is not None else "-",
                a.squawk or "-",
                f"{max(0, now - a.last_seen):.0f}s",
            )
            tags = []
            fresh = a.detected_at and now - a.detected_at < NEW_S
            if a.squawk in SPECIAL_SQUAWKS:
                tags.append("emergency")
            elif fresh and a.kind in SPECIAL_KINDS:
                tags.append("special")
            elif fresh:
                tags.append("new")
            elif now - a.last_seen > STALE_S:
                tags.append("stale")
            elif a.source == "Net":
                tags.append("net")
            if iid in existing:
                self.tree.item(iid, values=values, tags=tags)
            else:
                self.tree.insert("", "end", iid=iid, values=values, tags=tags)
        for iid in existing - set(wanted):
            self.tree.delete(iid)
        for index, iid in enumerate(wanted):
            self.tree.move(iid, "", index)

        cube = sum(1 for a in self.aircraft.values() if a.cube_recent(now))
        if not self._net_on():
            net = "offline - Cube data only" if not self.online else "online feed off"
        elif self.net_time:
            net = (f"online feed: {self.net_count} aircraft within {self.settings['net_radius_km']:g} km "
                   f"({', '.join(self.net_sources)}, {now - self.net_time:.0f}s ago)")
        else:
            net = self.net_error or "online feed: connecting…"
        self.ac_footer.configure(text=f"Cube receiver: {cube} aircraft now   ·   {net}   ·   "
                                      f"alarm: Cube aircraft at "
                                      f"{'any distance' if not self.alert_m() else str(self.settings['alert_km']) + ' km'}"
                                      f", online aircraft within {self.settings['net_alarm_km']:g} km")

    @staticmethod
    def _summary(a):
        name = a.callsign or a.icao_hex
        extra = " ".join(x for x in (a.info.get("Registration"), a.info.get("ICAO type") or a.info.get("Type")) if x)
        parts = [f"{name}  {extra}".strip()]
        if a.on_ground and a.source == "Net":
            parts.append("on ground")
        else:
            parts.append(f"{a.alt_m * 3.28084:,.0f} ft" if a.alt_m is not None else "alt ?")
        if a.dist_m is not None:
            parts.append(f"{a.dist_m / 1000:.1f} km {compass(a.bearing)}")
        if a.info.get("From") and a.info.get("To"):
            parts.append(f"{a.info['From']} → {a.info['To']}")
        return "   ·   ".join(parts)

    def _update_banner(self, now):
        live = [a for a in self._sorted_aircraft() if now - a.last_seen < STALE_S]
        recent = sorted((a for a in live if a.detected_at and now - a.detected_at < NEW_S),
                        key=lambda a: (a.kind not in SPECIAL_KINDS, -a.detected_at))
        emerg = [a for a in self.aircraft.values() if a.squawk in SPECIAL_SQUAWKS]
        sat_up = [s for s in (self.sats or {}).get("above", [])
                  if s["target"] and s["el"] >= float(self.settings["sat_alarm_el"])]
        self.flash_on = not self.flash_on
        if self.latched:
            L = self.latched
            secs = int(now - L["since"])
            more = f"   ·   {L['count']} alerts" if L.get("count", 1) > 1 else ""
            sat = L["tune"] in SAT_TUNES
            bg = ("#7b2cbf" if self.flash_on else "#3d1466") if sat else ("#d0021b" if self.flash_on else "#6e0010")
            self.banner.configure(bg=bg, text=f"🔔  {L['text']}   ·   alarm on for {secs // 60}:{secs % 60:02d}{more}")
        elif emerg:
            a = emerg[0]
            self.banner.configure(bg="#b00020" if self.flash_on else "#7a0016",
                                  text=f"⚠  {SPECIAL_SQUAWKS[a.squawk]} squawk {a.squawk}   ·   {self._summary(a)}")
        elif recent:
            a = recent[0]
            flashing = now - a.detected_at < 10
            more = f"   (+{len(recent) - 1} more)" if len(recent) > 1 else ""
            src = "" if a.source != "Net" else "   [online feed]"
            if a.kind in SPECIAL_KINDS:
                label, icon, _ = KINDS[a.kind]
                bg = "#7b2cbf" if (self.flash_on or not flashing) else "#4a1a73"
                text = f"{icon}  {KIND_NAMES[a.kind].upper()} DETECTED:  {self._summary(a)}{src}{more}"
            else:
                bg = "#c0392b" if (self.flash_on or not flashing) else "#7d2318"
                text = f"✈  AIRCRAFT DETECTED:  {self._summary(a)}{src}{more}"
            self.banner.configure(bg=bg, text=text)
        elif sat_up:
            s = max(sat_up, key=lambda x: x["el"])
            look = "   ·   VISIBLE - look up!" if s["visible"] else ""
            self.banner.configure(bg="#5b2a86", text=f"🛰  {s['target'].upper()} OVERHEAD NOW:  {s['el']:.0f}° high in the "
                                                     f"{compass(s['az'])}   ·   {s['height']:.0f} km up{look}")
        elif live:
            self.banner.configure(bg="#1d4e89",
                                  text=f"✈  Tracking {len(live)} aircraft   ·   nearest: {self._summary(live[0])}")
        else:
            nxt = [p for p in self.sat_passes if p["start"] > now]
            sat = (f"   ·   next pass: {nxt[0]['name']} {when_text(nxt[0]['start'], now)}"
                   f" ({nxt[0]['max_el']:.0f}°)") if nxt else ""
            km = float(self.settings["alert_km"] or 0)
            rng = "any distance" if km <= 0 else f"alarm range {km:g} km"
            self.banner.configure(bg="#1d2a38", text=f"Listening for aircraft…   ·   {rng}{sat}")
        self.banner_frame.configure(bg=self.banner.cget("bg"))

    # ---------------------------------------------------------------- satellites tab
    def _update_sat_tab(self, now):
        snap = self.sats
        self.sat_status_lbl.configure(text=self.sat_status)
        if not snap:
            self.sat_summary.configure(text="📴  Offline - satellites need the internet" if not self.online
                                       else "Satellites: loading orbits…")
            for tree in (self.sat_tree, self.pass_tree):
                tree.delete(*tree.get_children())
            return
        counts = snap["counts"]
        order = ["Space station", "Bright", "Weather", "Starlink", "GPS", "Galileo", "GLONASS", "BeiDou"]
        parts = [f"{k} {counts[k]}" for k in order if counts.get(k)]
        sky = "night" if snap["sun_el"] < -6 else ("twilight" if snap["sun_el"] < 0 else "daylight")
        self.sat_summary.configure(text=f"{len(snap['above'])} satellites above you   ·   sky: {sky}")
        self.sat_status_lbl.configure(text="  ·  ".join(parts) + "      " + self.sat_status)
        show_sl, show_gnss = self.settings["sat_show_starlink"], self.settings["sat_show_gnss"]
        rows = [s for s in snap["above"]
                if (show_sl or s["kind"] != "starlink") and (show_gnss or s["kind"] != "gnss")]
        rows.sort(key=lambda s: (s["target"] is None, -s["el"]))
        rows = rows[:300]
        existing = set(self.sat_tree.get_children())
        wanted = []
        for s in rows:
            iid = "s" + s["id"]
            wanted.append(iid)
            values = (s["target"] + "  (" + s["name"] + ")" if s["target"] else s["name"], s["label"],
                      f"{s['el']:.0f}", f"{compass(s['az'])} {s['az']:.0f}°", f"{s['range']:,.0f}",
                      f"{s['height']:,.0f}", "👁 yes" if s["visible"] else "")
            tags = ["target"] if s["target"] else (["vis"] if s["visible"] else (["dim"] if s["el"] < 10 else []))
            if iid in existing:
                self.sat_tree.item(iid, values=values, tags=tags)
            else:
                self.sat_tree.insert("", "end", iid=iid, values=values, tags=tags)
        for iid in existing - set(wanted):
            self.sat_tree.delete(iid)
        for index, iid in enumerate(wanted):
            self.sat_tree.move(iid, "", index)
        self._update_passes()

    def _update_passes(self):
        now = time.time()
        self.pass_tree.delete(*self.pass_tree.get_children())
        for i, p in enumerate([p for p in self.sat_passes if p["end"] > now][:8]):
            on_now = p["start"] <= now <= p["end"]
            self.pass_tree.insert("", "end", iid=f"p{i}", values=(
                p["name"], when_text(p["start"], now), "NOW" if on_now else in_text(p["start"] - now),
                f"{p['max_el']:.0f}°", f"{compass(p['rise_az'])} → {compass(p['set_az'])}",
                "👁 yes" if p["visible"] else "no"),
                tags=["now"] if on_now else (["vis"] if p["visible"] else []))

    # ---------------------------------------------------------------- GPS tab
    def _update_gps_tab(self, now):
        v = self.gps_vals
        g = self.telem.get("GPS_RAW_INT")
        fresh = g is not None and now - self.autopilot_seen < 5
        if fresh:
            v["fix"].configure(text=GPS_FIX.get(g.fix_type, str(g.fix_type)),
                               fg=GREEN if g.fix_type >= 3 else (ORANGE if g.fix_type else RED))
            v["sats"].configure(text=str(g.satellites_visible) if g.satellites_visible != 255 else "?")
            hd = f"{g.eph / 100:.2f}" if g.eph != 65535 else "?"
            vd = f"{g.epv / 100:.2f}" if g.epv != 65535 else "?"
            v["hdop"].configure(text=f"{hd} / {vd}")
            h_acc = getattr(g, "h_acc", 0) or 0
            v_acc = getattr(g, "v_acc", 0) or 0
            v["acc"].configure(text=f"{h_acc / 1000:.1f} / {v_acc / 1000:.1f} m" if h_acc else "—")
            if g.fix_type >= 2:
                v["pos"].configure(text=f"{g.lat / 1e7:.5f}, {g.lon / 1e7:.5f}", font=(FONT, 11, "bold"))
                v["alt"].configure(text=f"{g.alt / 1000:.1f} m")
                cog = f" · {g.cog / 100:.0f}°" if g.cog != 65535 else ""
                v["spd"].configure(text=f"{g.vel / 100 * 3.6:.1f} km/h{cog}")
            else:
                for k in ("pos", "alt", "spd"):
                    v[k].configure(text="—")
        else:
            for k in ("fix", "sats", "hdop", "acc", "pos", "alt", "spd"):
                v[k].configure(text="—", fg=MUTED)
        g2 = self.telem.get("GPS2_RAW")
        if g2 is not None and now - self.autopilot_seen < 5:
            v["gps2"].configure(text=f"{GPS_FIX.get(g2.fix_type, g2.fix_type)} · {g2.satellites_visible} sat")
        else:
            v["gps2"].configure(text="none", fg=MUTED)
        if not fresh:
            note = "No data from the Cube yet."
        elif g.fix_type == 0:
            note = ("No GPS module is connected to the Cube. Plug one into the GPS1 port - the overhead "
                    "navigation satellites below are what it would be able to use.")
        elif g.fix_type < 3:
            note = "GPS connected but no 3D fix yet - it needs a clear view of the sky (can take a few minutes)."
        else:
            note = ""
        self.gps_note.configure(text=note)

    def _update_gnss(self, now):
        snap = self.sats
        if not snap:
            self.gnss_counts.configure(text="📴  Offline - go online to see which navigation satellites are overhead"
                                       if not self.online else "Loading orbits…")
            self.gnss_tree.delete(*self.gnss_tree.get_children())
            self.gps_vals["pred"].configure(text="—")
            return
        rows = [s for s in snap["above"] if s["kind"] == "gnss"]
        counts = {}
        for s in rows:
            counts[s["label"]] = counts.get(s["label"], 0) + 1
        good = sum(1 for s in rows if s["el"] >= 10)
        self.gnss_counts.configure(text=f"{len(rows)} above the horizon ({good} higher than 10°):   " + "   ".join(
            f"{k} {counts.get(k, 0)}" for k in ("GPS", "Galileo", "GLONASS", "BeiDou")))
        self.gps_vals["pred"].configure(text=f"{good} usable")
        rows.sort(key=lambda s: -s["el"])
        self.gnss_tree.delete(*self.gnss_tree.get_children())
        for s in rows:
            self.gnss_tree.insert("", "end", iid="s" + s["id"], values=(
                s["name"], s["label"], f"{s['el']:.0f}", f"{compass(s['az'])} {s['az']:.0f}°", f"{s['range']:,.0f}"),
                tags=[s["label"]])

    # ---------------------------------------------------------------- right panel: radar / sky
    def _update_legend(self):
        idx = self.nb.index("current") if hasattr(self, "nb") else 0
        if idx == 0:
            text = ("●  heard by the Cube     ○  online feed only     red = just detected\n"
                    "●  airplane   ■  helicopter   ◆  drone   ▲  balloon/glider")
        elif idx == 1:
            text = ("●  ISS / Tiangong / Hubble   ●  bright   ●  weather   ·  Starlink\n"
                    "centre = straight up, edge = horizon")
        else:
            text = "●  GPS   ●  Galileo   ●  GLONASS   ●  BeiDou      centre = straight up"
        self.legend.configure(text=text)

    def _draw_view(self):
        idx = self.nb.index("current")
        if idx == 0:
            self._draw_radar()
        else:
            self._draw_sky(gnss=idx == 2)

    # ---- radar sweep animation
    SWEEP_PERIOD = 4.0      # seconds per full turn
    SWEEP_TRAIL = 70.0      # degrees of fading afterglow behind the beam
    RADAR_BG = "#0b1620"

    def _animate_sweep(self):
        now = time.time()
        prev = self.sweep_angle
        self.sweep_angle = (now % self.SWEEP_PERIOD) / self.SWEEP_PERIOD * 360.0
        step = (self.sweep_angle - prev) % 360.0
        # the beam just passed these aircraft: flash a ping ring on them
        for bearing, x, y, col in self.radar_blips:
            if 0 < (bearing - prev) % 360.0 <= step:
                self.pings.append((now, x, y, col))
        self.pings = [p for p in self.pings if now - p[0] < 1.2]
        self._draw_sweep(now)
        self.after(40, self._animate_sweep)

    def _draw_sweep(self, now=None):
        c = self.radar
        c.delete("sweep")
        c.delete("ping")
        if self.radar_geom is None or self.nb.index("current") != 0:
            return
        now = now or time.time()
        cx, cy, r = self.radar_geom
        a = self.sweep_angle
        slices = 28
        width = self.SWEEP_TRAIL / slices
        for i in range(slices):
            col = blend("#1f9c62", self.RADAR_BG, (i / slices) ** 0.6)
            # Tk arcs: degrees counter-clockwise from east; bearings: clockwise from north
            c.create_arc(cx - r, cy - r, cx + r, cy + r, start=90 - (a - i * width), extent=width + 0.4,
                         style="pieslice", fill=col, outline="", tags="sweep")
        rad = math.radians(a)
        c.create_line(cx, cy, cx + r * math.sin(rad), cy - r * math.cos(rad), fill="#5dffb0", width=2,
                      tags="sweep")
        c.tag_lower("sweep")  # rings, labels and aircraft stay on top of the beam
        for t0, x, y, col in self.pings:
            age = now - t0
            rr = 5 + 18 * age
            c.create_oval(x - rr, y - rr, x + rr, y + rr, outline=blend(col, self.RADAR_BG, min(1.0, age / 1.2)),
                          width=2, tags="ping")

    def _draw_radar(self):
        c = self.radar
        c.delete("all")
        w, h = int(c["width"]), int(c["height"])
        cx, cy = w / 2, h / 2
        r = min(w, h) / 2 - 20
        self.radar_geom = (cx, cy, r)
        blips = []
        home = self.home()
        alert_km = float(self.settings["alert_km"] or 0)
        shown = self._sorted_aircraft()
        dists = [a.dist_m / 1000 for a in shown if a.dist_m is not None]
        rng = max([20.0, alert_km * 1.2] + dists) * 1.1
        rng = next((s for s in (25, 50, 75, 100, 150, 200, 300, 400, 500) if s >= rng), rng)

        for i in range(1, 5):
            rr = r * i / 4
            c.create_oval(cx - rr, cy - rr, cx + rr, cy + rr, outline="#1e3a4f")
            c.create_text(cx + 4, cy - rr + 8, text=f"{rng * i / 4:g} km", fill="#4f7f9f", anchor="w", font=(FONT, 8))
        c.create_line(cx - r, cy, cx + r, cy, fill="#1e3a4f")
        c.create_line(cx, cy - r, cx, cy + r, fill="#1e3a4f")
        for label, x, y in (("N", cx, cy - r - 10), ("S", cx, cy + r + 10), ("E", cx + r + 10, cy), ("W", cx - r - 10, cy)):
            c.create_text(x, y, text=label, fill=ACCENT, font=(FONT, 9, "bold"))
        if alert_km > 0:
            ro = r * alert_km / rng
            c.create_oval(cx - ro, cy - ro, cx + ro, cy + ro, outline=RED, dash=(4, 3), width=2)
        net_km = float(self.settings["net_alarm_km"] or 0)
        if self._net_on() and net_km > 0:
            ro = r * net_km / rng
            c.create_oval(cx - ro, cy - ro, cx + ro, cy + ro, outline="#9fb3c8", dash=(2, 4))
        c.create_oval(cx - 5, cy - 5, cx + 5, cy + 5, fill=ACCENT, outline="")
        c.create_text(cx, cy + 14, text="You", fill=ACCENT, font=(FONT, 8))
        self.radar_blips = blips
        self._draw_sweep()
        if home is None:
            return
        now = time.time()
        for a in shown:
            if a.dist_m is None:
                continue
            d = min(r, r * (a.dist_m / 1000) / rng)
            ang = math.radians(a.bearing)
            x, y = cx + d * math.sin(ang), cy - d * math.cos(ang)
            fresh = a.detected_at and now - a.detected_at < NEW_S
            col = "#5c6773" if now - a.last_seen > STALE_S else (RED if fresh else KINDS[a.kind][2])
            filled = a.source != "Net"
            if a.icao == (self.selected[1] if self.selected and self.selected[0] == "ac" else None):
                c.create_oval(x - 11, y - 11, x + 11, y + 11, outline="white", width=2)
            self._marker(c, x, y, a.kind, col, filled)
            blips.append((a.bearing, x, y, col))
            if a.heading is not None and not a.on_ground:
                hr = math.radians(a.heading)
                c.create_line(x, y, x + 16 * math.sin(hr), y - 16 * math.cos(hr), fill=col, width=2)
            alt = f"  FL{a.alt_m * 3.28084 / 100:03.0f}" if a.alt_m is not None and not a.on_ground else ""
            c.create_text(x + 8, y + 6, text=f"{a.callsign or a.icao_hex}{alt}", fill=col, anchor="nw", font=(FONT, 8))

    @staticmethod
    def _marker(c, x, y, kind, col, filled):
        fill = col if filled else ""
        s = 5
        if kind == "heli":
            c.create_rectangle(x - s, y - s, x + s, y + s, fill=fill, outline=col, width=2)
        elif kind == "drone":
            c.create_polygon(x, y - s - 1, x + s + 1, y, x, y + s + 1, x - s - 1, y, fill=fill, outline=col, width=2)
        elif kind == "balloon":
            c.create_polygon(x, y - s - 1, x + s + 1, y + s, x - s - 1, y + s, fill=fill, outline=col, width=2)
        else:
            c.create_oval(x - 4, y - 4, x + 4, y + 4, fill=fill, outline=col, width=2)

    def _draw_sky(self, gnss):
        c = self.radar
        c.delete("all")
        w, h = int(c["width"]), int(c["height"])
        cx, cy = w / 2, h / 2
        r = min(w, h) / 2 - 20
        for el in (0, 30, 60):
            rr = r * (90 - el) / 90
            c.create_oval(cx - rr, cy - rr, cx + rr, cy + rr, outline="#1e3a4f", width=2 if el == 0 else 1)
            c.create_text(cx + 4, cy - rr + 8, text=f"{el}°", fill="#4f7f9f", anchor="w", font=(FONT, 8))
        c.create_line(cx - r, cy, cx + r, cy, fill="#1e3a4f")
        c.create_line(cx, cy - r, cx, cy + r, fill="#1e3a4f")
        for label, x, y in (("N", cx, cy - r - 10), ("S", cx, cy + r + 10), ("E", cx + r + 10, cy), ("W", cx - r - 10, cy)):
            c.create_text(x, y, text=label, fill=ACCENT, font=(FONT, 9, "bold"))
        c.create_text(cx, cy, text="+", fill="#4f7f9f", font=(FONT, 10))
        snap = self.sats
        if not snap:
            c.create_text(cx, cy + 30, text="Offline - press 'Go online' to see satellites" if not self.online
                          else "Loading satellite orbits…", fill=ORANGE, font=(FONT, 10))
            return
        sel = self.selected[1] if self.selected and self.selected[0] == "sat" else None

        def pos(s):
            d = r * (90 - s["el"]) / 90
            a = math.radians(s["az"])
            return cx + d * math.sin(a), cy - d * math.cos(a)

        sats = [s for s in snap["above"] if (s["kind"] == "gnss") == gnss]
        sats.sort(key=lambda s: (s["kind"] != "starlink", s["target"] is not None))  # Starlink dots underneath
        for s in sats:
            x, y = pos(s)
            if s["kind"] == "starlink":
                if gnss:
                    continue
                c.create_oval(x - 1.5, y - 1.5, x + 1.5, y + 1.5, fill=SAT_COLORS["starlink"], outline="")
                continue
            col = SAT_COLORS.get(s["label"]) if s["kind"] == "gnss" else SAT_COLORS[s["kind"]]
            size = 6 if s["target"] else 4
            if s["id"] == sel:
                c.create_oval(x - 11, y - 11, x + 11, y + 11, outline="white", width=2)
            c.create_oval(x - size, y - size, x + size, y + size, fill=col, outline="" if not s["visible"] else "white")
            if gnss or s["target"] or s["visible"] or s["id"] == sel:
                c.create_text(x + 6, y + 3, text=short_sat_name(s), fill=col, anchor="nw",
                              font=(FONT, 7 if gnss else 8, "bold" if s["target"] else "normal"))
        if not gnss:
            n_sl = snap["counts"].get("Starlink", 0)
            c.create_text(6, h - 8, text=f"{n_sl} Starlink" if n_sl else "", fill=SAT_COLORS["starlink"],
                          anchor="w", font=(FONT, 8))

    # ---------------------------------------------------------------- details
    def _find_sat(self, sat_id):
        for s in (self.sats or {}).get("above", []):
            if s["id"] == sat_id:
                return s
        return None

    def _show_detail(self):
        lines = []

        def add(k, v):
            if v not in (None, ""):
                lines.append(f"{k:<13}{v}")

        sel = self.selected
        if sel and sel[0] == "ac" and sel[1] in self.aircraft:
            a = self.aircraft[sel[1]]
            self.detail_card.title_label.configure(text="SELECTED AIRCRAFT")
            add("Callsign", a.callsign)
            add("ICAO hex", a.icao_hex)
            add("Kind", KIND_NAMES[a.kind])
            add("Source", {"Cube": "Cube receiver", "Net": "Online feed only", "Cube+Net": "Cube + online"}[a.source])
            for k in ("Registration", "Type", "ICAO type", "Airline", "From", "To", "Owner", "Country", "Lookup"):
                add(k, a.info.get(k))
            add("Category", EMITTER_TYPES.get(a.emitter, a.emitter))
            if a.on_ground and a.source == "Net":
                add("Altitude", "on the ground")
            elif a.alt_m is not None:
                add("Altitude", f"{a.alt_m * 3.28084:,.0f} ft / {a.alt_m:,.0f} m ({a.alt_type})")
            if a.speed_ms is not None:
                add("Speed", f"{a.speed_ms * 3.6:.0f} km/h / {a.speed_ms * 1.94384:.0f} kt")
            if a.vspeed_ms is not None:
                add("Vert speed", f"{a.vspeed_ms:+.1f} m/s")
            add("Heading", f"{a.heading:.0f}°" if a.heading is not None else None)
            if a.lat is not None:
                add("Position", f"{a.lat:.5f}, {a.lon:.5f}")
            if a.dist_m is not None:
                add("Distance", f"{a.dist_m / 1000:.2f} km {compass(a.bearing)} ({a.bearing:.0f}°)")
                add("Closest", f"{a.min_dist_m / 1000:.2f} km")
            if a.elevation is not None:
                add("Elevation", f"{a.elevation:.0f}° above horizon")
            add("Squawk", a.squawk + (f"  {SPECIAL_SQUAWKS[a.squawk]}" if a.squawk in SPECIAL_SQUAWKS else ""))
            add("Cube msgs", a.msgs)
            add("Last seen", f"{max(0, time.time() - a.last_seen):.0f} s ago")
        elif sel and sel[0] == "sat" and self._find_sat(sel[1]):
            s = self._find_sat(sel[1])
            self.detail_card.title_label.configure(text="SELECTED SATELLITE")
            add("Name", s["target"] or s["name"])
            if s["target"]:
                add("Catalog name", s["name"])
            add("Group", s["label"])
            add("NORAD id", s["id"])
            add("Elevation", f"{s['el']:.1f}° above horizon")
            add("Direction", f"{compass(s['az'])} ({s['az']:.0f}°)")
            add("Distance", f"{s['range']:,.0f} km")
            add("Height", f"{s['height']:,.0f} km above Earth")
            add("Sunlit", "yes" if s["sunlit"] else "no (in Earth's shadow)")
            add("Visible", "yes - look up!" if s["visible"] else "no")
            if s["target"]:
                nxt = [p for p in self.sat_passes if p["name"] == s["target"] and p["start"] > time.time()]
                if nxt:
                    add("Next pass", f"{when_text(nxt[0]['start'], time.time())}, {nxt[0]['max_el']:.0f}°")
        else:
            self.detail_card.title_label.configure(text="SELECTED")
            lines.append("Click an aircraft or satellite in the list to see details.")
        text = "\n".join(lines)
        if self.detail.get("1.0", "end-1c") != text:
            self.detail.configure(state="normal")
            self.detail.delete("1.0", "end")
            self.detail.insert("1.0", text)
            self.detail.configure(state="disabled")

    def _on_close(self):
        save_settings(self.settings)
        for svc in (self.link, self.tracker, self.feed):
            if svc:
                svc.stop()
        if self.link:
            self.link.join(timeout=2)
        self.destroy()


if __name__ == "__main__":
    App().mainloop()
