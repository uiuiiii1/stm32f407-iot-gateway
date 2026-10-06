@echo off
rem Local MQTT broker (amqtt) for testing. Keep this window open.
rem Device/ota_send connect to PC LAN IP 192.168.0.105:1883.
rem Switch back to public broker.emqx.io when internet is stable.
"C:\Python\Python313\Scripts\amqtt.exe" -c "%~dp0broker.yaml"
pause
