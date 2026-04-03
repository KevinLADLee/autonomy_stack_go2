# Autostart Layout

Put boot-time scripts in `autostart/scripts/` and matching systemd units in `autostart/services/`.

Install and enable one service with:
```
bash autostart/install_service.sh switch_off_utlidar
```

Common commands:
```
sudo systemctl status switch_off_utlidar.service
sudo systemctl start switch_off_utlidar.service
sudo systemctl disable switch_off_utlidar.service
```