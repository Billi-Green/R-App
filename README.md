# RD-03D Radar GhostESP App

Native GhostESP C5 app for discovering RD-03D radar nodes over ESP-NOW.

The app provides a `Connect` screen for ready radar discovery and multi-select,
plus a `Radar` screen with an 80-degree 2D field-of-view display and 8m range.
The view matches the datasheet's orientation (a): positive azimuth is shown
to the left and negative azimuth to the right.

## Build

```powershell
gbt dist . --target esp32c5 --gapp
```

If a Windows build reports that `ESP_ROM_ELF_DIR` is undefined, run `. "$HOME\.ghostbt\esp-idf\export.ps1"` in that PowerShell session and build again.

Copy the generated `.gapp` from `dist/` to `/mnt/ghostesp/apps/` on the SD card and reboot the device.

Keep the bundled files under `sdk/` in source control so catalog CI can build the same SDK version.

The app expects telemetry in this format:

```text
RADAR,sequence,target_id,detected,x_mm,y_mm,speed,distance_mm,angle_deg
```

Each radar node sends one record for each of its three RD-03D target slots.

Raw multi-target slots are displayed directly from the RD-03D telemetry.
