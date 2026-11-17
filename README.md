# ADS-B Sky Monitor

A desktop app that shows everything flying above you, using a **Cube Orange+** with its ADS-B receiver.

- Live flight data from the Cube: attitude, altitude, battery and GPS, with an artificial horizon and a 3D orientation view
- ADS-B aircraft list (planes, helicopters, drones, balloons), with alarms
- Extra aircraft from free online ADS-B networks (adsb.fi, adsb.lol) that the local antenna misses
- Satellites overhead (ISS, Tiangong, Hubble, Starlink, weather, GPS / Galileo / GLONASS / BeiDou), predicted with SGP4 from CelesTrak orbits, with pass predictions
- Re-shares the Cube's USB link to Mission Planner on UDP 14550

## Run

```bash
pip install pymavlink pyserial numpy sgp4
```
```bash
python adsb_monitor.py
```

Or double-click `run_adsb_monitor.bat`. In Mission Planner, pick **UDP** → Connect → port **14550**.

## Files

| File | Purpose |
|---|---|
| `adsb_monitor.py` | Main app |
| `sat_tracker.py` | SGP4 satellite tracker |
| `online_feed.py` | Online ADS-B network feed |

Not uploaded: `adsb_monitor_settings.json` (contains your home location), `adsb_logs/` and `sat_cache/` (both are regenerated at runtime).
