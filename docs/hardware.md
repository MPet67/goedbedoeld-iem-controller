# Hardware / validatiestatus

## IEM Controller
- Freenove FNK0104N, ESP32-S3, 480×320 ST77922 display, 8 MB OPI PSRAM.
- HW-040: CLK GPIO43, DT GPIO44, SW GPIO46 (GPIO46 is een strapping-pin; heroverwegen voor definitieve hardware).
- Touch: I²C SDA GPIO38, SCL GPIO39, RESET GPIO48, INT GPIO47.
- Schermrotatie: 180° ten opzichte van oorspronkelijke test.
- Live X32 OSC-sends, geselecteerde IEM-bus, VU-meters, kanaalindeling, Wi-Fi-setup via touch/encoder.

## Simulator
- ESP32-S3 Mini, eigen accesspoint `GB-X32-SIM`, OSC UDP 10023.
- OSC-implementatie voor ontwikkeling, geen één-op-één emulatie van alle X32-berichten.

## Versiebeleid
- IEMC v2.0 RC is bevroren.
- Nieuwe functies beginnen in een aparte versie/branch.
