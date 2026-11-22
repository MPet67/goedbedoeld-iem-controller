# Goed Bedoeld — IEM Controller (public source review)

ESP32-S3 Freenove FNK0104N IEM controller and ESP32-S3 Mini X32 OSC simulator.

## Firmware versions
- **v2.0 RC** is the previously validated reference on the user's device. The source in this public-review package is a **sanitized derivative**, not byte-identical to the frozen, tested version. It is **not a validated replacement firmware**.
- X32 simulator v0.2j: public-review derivative with locally configured access-point credentials.
- OTA is not yet implemented or tested. Do not issue an installable release asset yet.

## Network configuration
- Freenove: no pre-loaded shared test password/profile. Configure Wi-Fi on the touch UI: **SETUP > WI-FI**.
- Simulator: copy `simulator/GoedBedoeld_X32_Simulator_v02j_FX_COLORS/local_secrets.example.h` to `local_secrets.h`, provide unique AP password (>=8 characters); the real file is ignored by Git. Reconnect Freenove to the changed SSID/password.
- Wi-Fi profiles entered on the controller are stored in its NVS flash; they are not in this repository.

## Publication warning
Before switching an existing GitHub repository to public, **review its entire commit history**, not only the latest files. If a shared password was committed, assume it is exposed when the repo goes public; rotate the password on the devices. Rewriting/removing Git history does not undo credentials that were shared or copied. Recreating the public repo with fresh clean history is the simplest route when unsure.

See `docs/publication-checklist.md` and `docs/ota-plan.md`.
