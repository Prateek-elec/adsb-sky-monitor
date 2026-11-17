"""
Online ADS-B feed: aircraft around a location from free community ADS-B networks
(adsb.fi and adsb.lol). Used alongside the Cube's own receiver so aircraft the local
antenna misses still show up.
"""

import json
import threading
import time
import urllib.request

SOURCES = [
    ("adsb.fi", "https://opendata.adsb.fi/api/v2/lat/{lat:.4f}/lon/{lon:.4f}/dist/{nm:.0f}"),
    ("adsb.lol", "https://api.adsb.lol/v2/point/{lat:.4f}/{lon:.4f}/{nm:.0f}"),
]

# ADS-B emitter category ("A3", ...) -> MAVLink ADSB_EMITTER_TYPE number
CATEGORY_TO_EMITTER = {
    "A1": 1, "A2": 2, "A3": 3, "A4": 4, "A5": 5, "A6": 6, "A7": 7,
    "B1": 9, "B2": 10, "B3": 11, "B4": 12, "B6": 14, "B7": 15,
    "C1": 17, "C2": 18, "C3": 19,
}


class OnlineFeed(threading.Thread):
    """Polls the networks every `interval` s; pushes ("net", (time, [aircraft dicts], [ok sources]))
    or ("net_error", text) into out_q."""

    def __init__(self, out_q, interval=10.0):
        super().__init__(daemon=True)
        self.q = out_q
        self.interval = interval
        self.lat = self.lon = None
        self.radius_km = 150.0
        self.enabled = True
        self.stop_event = threading.Event()

    def configure(self, lat, lon, radius_km, enabled):
        self.lat, self.lon = lat, lon
        self.radius_km = radius_km
        self.enabled = enabled

    def stop(self):
        self.stop_event.set()

    def run(self):
        while not self.stop_event.is_set():
            if self.enabled and self.lat is not None:
                self._poll()
            self.stop_event.wait(self.interval)

    def _poll(self):
        nm = max(1.0, min(250.0, self.radius_km / 1.852))
        merged, ok, errors = {}, [], []
        for name, url in SOURCES:
            try:
                req = urllib.request.Request(url.format(lat=self.lat, lon=self.lon, nm=nm),
                                             headers={"User-Agent": "adsb-overhead-monitor/1.0"})
                with urllib.request.urlopen(req, timeout=10) as r:
                    data = json.loads(r.read().decode("utf-8"))
                ok.append(name)
            except Exception as e:  # noqa: BLE001
                errors.append(f"{name}: {e.__class__.__name__}")
                continue
            for ac in data.get("ac") or data.get("aircraft") or []:
                if ac.get("lat") is None or ac.get("lon") is None or not ac.get("hex"):
                    continue
                key = ac["hex"].lower().lstrip("~")
                prev = merged.get(key)
                if prev is None or ac.get("seen_pos", 99) < prev.get("seen_pos", 99):
                    merged[key] = ac
        if ok:
            self.q.put(("net", (time.time(), list(merged.values()), ok)))
        else:
            self.q.put(("net_error", "Online feed unreachable (" + ", ".join(errors) + ")"))
