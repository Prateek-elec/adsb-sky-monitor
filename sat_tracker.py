"""
Satellite tracker: positions of satellites above an observer, computed with SGP4 from
CelesTrak orbital elements (refreshed every 12 h, cached on disk).

Satellites don't transmit ADS-B, so they are *predicted* from their published orbits,
which is accurate to within a few km - plenty to tell what is over your head.
"""

import math
import os
import threading
import time
import urllib.request

import numpy as np
from sgp4.api import Satrec, SatrecArray

CACHE_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "sat_cache")
MAX_AGE_S = 12 * 3600
URL = "https://celestrak.org/NORAD/elements/gp.php?GROUP={group}&FORMAT=tle"

# CelesTrak group -> (label, kind). Order matters: a satellite listed in several groups keeps the first.
GROUPS = {
    "stations": ("Space station", "station"),
    "visual": ("Bright", "bright"),
    "weather": ("Weather", "weather"),
    "gps-ops": ("GPS", "gnss"),
    "galileo": ("Galileo", "gnss"),
    "glo-ops": ("GLONASS", "gnss"),
    "beidou": ("BeiDou", "gnss"),
    "starlink": ("Starlink", "starlink"),
}
# satellites we predict passes for and raise an alarm for
PASS_TARGETS = {"ISS (ZARYA)": "ISS", "CSS (TIANHE)": "Tiangong", "HST": "Hubble"}

R_EARTH = 6378.137
WGS84_F = 1 / 298.257223563
AU_KM = 149597870.7


# --------------------------------------------------------------------------- orbit data

def parse_tle(text):
    lines = [ln.rstrip() for ln in text.splitlines() if ln.strip()]
    out = []
    for i in range(len(lines) - 2):
        if lines[i + 1].startswith("1 ") and lines[i + 2].startswith("2 ") and not lines[i].startswith(("1 ", "2 ")):
            out.append((lines[i].strip(), lines[i + 1], lines[i + 2]))
    return out


def load_group(group, status=None):
    """TLEs for a CelesTrak group, from the disk cache or downloaded when older than 12 h."""
    os.makedirs(CACHE_DIR, exist_ok=True)
    path = os.path.join(CACHE_DIR, f"{group}.tle")
    fresh = os.path.exists(path) and time.time() - os.path.getmtime(path) < MAX_AGE_S
    if not fresh:
        try:
            if status:
                status(f"Downloading orbits: {GROUPS[group][0]}…")
            req = urllib.request.Request(URL.format(group=group),
                                         headers={"User-Agent": "adsb-overhead-monitor/1.0"})
            with urllib.request.urlopen(req, timeout=60) as r:
                text = r.read().decode("utf-8", "ignore")
            if parse_tle(text):
                with open(path, "w", encoding="utf-8") as f:
                    f.write(text)
        except Exception:  # noqa: BLE001 - fall back to an older cached copy
            pass
    try:
        with open(path, "r", encoding="utf-8") as f:
            return parse_tle(f.read())
    except OSError:
        return []


# --------------------------------------------------------------------------- geometry

def gmst(jd_ut1):
    """Greenwich mean sidereal time (rad), IAU-82 as used with SGP4/TEME."""
    t = (jd_ut1 - 2451545.0) / 36525.0
    sec = -6.2e-6 * t ** 3 + 0.093104 * t ** 2 + (876600.0 * 3600 + 8640184.812866) * t + 67310.54841
    return np.radians((sec % 86400.0) / 240.0)


def observer_ecef(lat, lon, alt_m):
    lat, lon = math.radians(lat), math.radians(lon)
    e2 = WGS84_F * (2 - WGS84_F)
    n = R_EARTH / math.sqrt(1 - e2 * math.sin(lat) ** 2)
    h = alt_m / 1000.0
    return np.array([(n + h) * math.cos(lat) * math.cos(lon),
                     (n + h) * math.cos(lat) * math.sin(lon),
                     (n * (1 - e2) + h) * math.sin(lat)])


def enu_matrix(lat, lon):
    lat, lon = math.radians(lat), math.radians(lon)
    sl, cl, sp, cp = math.sin(lon), math.cos(lon), math.sin(lat), math.cos(lat)
    return np.array([[-sl, cl, 0.0],
                     [-sp * cl, -sp * sl, cp],
                     [cp * cl, cp * sl, sp]])


def sun_direction_eci(jd):
    """Unit vector to the Sun (approximate, good to ~0.01 deg)."""
    n = jd - 2451545.0
    L = np.radians(280.460 + 0.9856474 * n)
    g = np.radians(357.528 + 0.9856003 * n)
    lam = L + np.radians(1.915) * np.sin(g) + np.radians(0.020) * np.sin(2 * g)
    eps = np.radians(23.439 - 0.0000004 * n)
    return np.stack([np.cos(lam), np.cos(eps) * np.sin(lam), np.sin(eps) * np.sin(lam)], axis=-1)


class Observer:
    def __init__(self, lat, lon, alt_m):
        self.lat, self.lon, self.alt_m = lat, lon, alt_m
        self.ecef = observer_ecef(lat, lon, alt_m)
        self.enu = enu_matrix(lat, lon)


def look_angles(satrecs, unix_times, obs):
    """Propagate satellites to the given times. Returns dict of arrays shaped (n_sat, n_time)."""
    t = np.atleast_1d(np.asarray(unix_times, dtype=float))
    jd = 2440587.5 + np.floor(t / 86400.0)
    fr = (t % 86400.0) / 86400.0
    err, r, _v = satrecs.sgp4(jd, fr)                       # r: (n, m, 3) km, TEME frame
    g = gmst(jd + fr)                                       # (m,)
    cg, sg = np.cos(g), np.sin(g)
    x = cg * r[..., 0] + sg * r[..., 1]                     # TEME -> Earth-fixed
    y = -sg * r[..., 0] + cg * r[..., 1]
    z = r[..., 2]
    d = np.stack([x - obs.ecef[0], y - obs.ecef[1], z - obs.ecef[2]], axis=-1)
    enu = d @ obs.enu.T
    e, n, u = enu[..., 0], enu[..., 1], enu[..., 2]
    horiz = np.hypot(e, n)
    el = np.degrees(np.arctan2(u, horiz))
    az = np.degrees(np.arctan2(e, n)) % 360.0
    rng = np.sqrt(horiz ** 2 + u ** 2)
    height = np.linalg.norm(r, axis=-1) - R_EARTH

    # sunlight: cylindrical Earth-shadow model
    s = sun_direction_eci(jd + fr)                          # (m, 3)
    proj = np.einsum("nmk,mk->nm", r, s)
    perp = np.linalg.norm(r - proj[..., None] * s[None, :, :], axis=-1)
    sunlit = (proj > 0) | (perp > R_EARTH)
    # Sun elevation at the observer (to know if the sky is dark enough to see satellites)
    sx = cg * s[:, 0] + sg * s[:, 1]
    sy = -sg * s[:, 0] + cg * s[:, 1]
    sun_enu = np.stack([sx, sy, s[:, 2]], axis=-1) @ obs.enu.T
    sun_el = np.degrees(np.arcsin(np.clip(sun_enu[:, 2], -1, 1)))
    valid = (err == 0) & np.isfinite(el)
    return {"az": az, "el": el, "range": rng, "height": height, "sunlit": sunlit,
            "sun_el": sun_el, "valid": valid}


# --------------------------------------------------------------------------- tracker thread

class SatTracker(threading.Thread):
    """Loads orbits, then every 2 s publishes the satellites above the horizon.

    Pushes into out_q:
      ("sat_status", text)
      ("sats", {"time", "above": [dict...], "counts": {label: n}, "loaded": {label: n}, "sun_el"})
      ("sat_passes", [dict...])
    """

    def __init__(self, out_q, include_starlink=True):
        super().__init__(daemon=True)
        self.q = out_q
        self.include_starlink = include_starlink
        self.obs = None
        self.stop_event = threading.Event()
        self.names, self.labels, self.kinds, self.catnums = [], [], [], []
        self.array = None
        self.loaded_at = 0.0
        self.missing = []

    def set_observer(self, lat, lon, alt_m):
        self.obs = Observer(lat, lon, alt_m)
        self.passes_at = 0.0  # recompute passes for the new place

    def stop(self):
        self.stop_event.set()

    def _status(self, text):
        self.q.put(("sat_status", text))

    def _load(self):
        names, labels, kinds, catnums, recs, seen = [], [], [], [], [], set()
        loaded = {}
        for group, (label, kind) in GROUPS.items():
            if group == "starlink" and not self.include_starlink:
                continue
            tles = load_group(group, self._status)
            for name, l1, l2 in tles:
                cat = l1[2:7]
                if cat in seen:
                    continue
                try:
                    recs.append(Satrec.twoline2rv(l1, l2))
                except Exception:  # noqa: BLE001
                    continue
                seen.add(cat)
                names.append(name)
                labels.append(label)
                kinds.append(kind)
                catnums.append(cat)
                loaded[label] = loaded.get(label, 0) + 1
        wanted = [g for g in GROUPS if g != "starlink" or self.include_starlink]
        self.missing = [GROUPS[g][0] for g in wanted if GROUPS[g][0] not in loaded]
        self.loaded_at = time.time()
        if not recs:
            self._status("No satellite orbits available (no internet and no cached copy yet)")
            return False
        self.names, self.labels, self.kinds, self.catnums = names, labels, kinds, catnums
        self.recs = recs
        self.array = SatrecArray(recs)
        self.loaded = loaded
        self.target_idx = [i for i, n in enumerate(names) if n in PASS_TARGETS]
        self.target_array = SatrecArray([recs[i] for i in self.target_idx]) if self.target_idx else None
        note = f"  (not available yet, retrying: {', '.join(self.missing)})" if self.missing else ""
        self._status(f"{len(recs):,} satellites loaded{note}")
        return True

    def run(self):
        self.passes_at = 0.0
        while not self.stop_event.is_set():
            age = time.time() - self.loaded_at
            if self.array is None or age > MAX_AGE_S or (self.missing and age > 1800):
                if not self._load() and self.array is None:
                    self.stop_event.wait(60)
                    continue
            if self.obs is not None:
                try:
                    self._publish()
                    if time.time() - self.passes_at > 600:
                        self.passes_at = time.time()
                        self._passes()
                except Exception as e:  # noqa: BLE001
                    self._status(f"Satellite calculation error: {e}")
            self.stop_event.wait(2.0)

    def _publish(self):
        now = time.time()
        obs = self.obs
        la = look_angles(self.array, [now], obs)
        el, az, rng, h = la["el"][:, 0], la["az"][:, 0], la["range"][:, 0], la["height"][:, 0]
        sunlit, valid = la["sunlit"][:, 0], la["valid"][:, 0]
        sun_el = float(la["sun_el"][0])
        dark = sun_el < -6
        above = []
        counts = {}
        for i in np.nonzero(valid & (el > 0))[0]:
            label = self.labels[i]
            counts[label] = counts.get(label, 0) + 1
            # naked-eye visible: a bright low-orbit object, lit by the Sun, in a dark sky
            eye = self.kinds[i] in ("station", "bright") and h[i] < 2000
            above.append({
                "id": self.catnums[i], "name": self.names[i], "label": label, "kind": self.kinds[i],
                "az": float(az[i]), "el": float(el[i]), "range": float(rng[i]), "height": float(h[i]),
                "sunlit": bool(sunlit[i]), "visible": bool(eye and sunlit[i] and dark and el[i] > 10),
                "target": PASS_TARGETS.get(self.names[i]),
            })
        self.q.put(("sats", {"time": now, "above": above, "counts": counts, "loaded": dict(self.loaded),
                             "sun_el": sun_el}))

    def _passes(self):
        """Next passes (max elevation >= 10 deg) of ISS / Tiangong / Hubble over the next 24 h."""
        if self.target_array is None:
            return
        start = time.time()
        times = start + np.arange(0, 24 * 3600, 20.0)
        la = look_angles(self.target_array, times, self.obs)
        passes = []
        for k, idx in enumerate(self.target_idx):
            el = np.where(la["valid"][k], la["el"][k], -90)
            up = el > 0
            j = 0
            while j < len(times):
                if not up[j]:
                    j += 1
                    continue
                s = j
                while j < len(times) and up[j]:
                    j += 1
                e = j - 1
                peak = s + int(np.argmax(el[s:e + 1]))
                if el[peak] >= 10 and s > 0:
                    passes.append({
                        "name": PASS_TARGETS[self.names[idx]], "start": float(times[s]), "end": float(times[e]),
                        "peak_time": float(times[peak]), "max_el": float(el[peak]),
                        "peak_az": float(la["az"][k][peak]), "rise_az": float(la["az"][k][s]),
                        "set_az": float(la["az"][k][e]),
                        "visible": bool(la["sunlit"][k][peak] and la["sun_el"][peak] < -6),
                    })
        passes.sort(key=lambda p: p["start"])
        self.q.put(("sat_passes", passes[:12]))
