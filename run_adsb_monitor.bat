@echo off
cd /d "%~dp0"
python -c "import pymavlink, serial, numpy, sgp4" 2>nul || python -m pip install pymavlink pyserial numpy sgp4
start "" pythonw adsb_monitor.py
