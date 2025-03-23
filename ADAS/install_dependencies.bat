@echo off
call conda activate svs-env-test

pip install opencv-python networkx numpy paho-mqtt pygame shapely ultralytics carla

pause