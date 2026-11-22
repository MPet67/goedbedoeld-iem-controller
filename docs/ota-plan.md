# Onderzoek: GitHub Releases OTA

**Niet geïmplementeerd in v2.0 RC.**

1. Verifieer flashgrootte en de huidige Arduino-partitietabel; twee app-slots moeten groot genoeg zijn.
2. Ontwerp een OTA-partitietabel inclusief NVS; wijziging daarvan kan eenmalig USB-flashen en opnieuw instellen vereisen.
3. Publiceer geteste firmware als versioned GitHub Release asset (`.bin`) met manifest, bestandsgrootte, hash en doelhardware.
4. Gebruik HTTPS met correcte certificaatverificatie; geen hardcoded persoonlijke GitHub-tokens.
5. Download en schrijf naar inactieve OTA-slot; verifieer inhoud voor switch.
6. Blokkeer mixerwrites tijdens update; eis handmatige bevestiging.
7. Controleer na reboot netwerk, display en OSC; stel een expliciete rollback-/herstelstrategie op.
8. Test offline, verbroken download, foutieve hash, onvoldoende ruimte en herstel via USB.

Bij een publieke repository kan de controller GitHub Releases zonder token opvragen. Voor automatische downloads blijft authenticiteit cruciaal: controleer HTTPS, doelhardware, versienummer en cryptografische firmwarehandtekening. Een SHA-256-hash uit hetzelfde release-manifest alleen is geen betrouwbare bescherming tegen een kwaadwillig gewijzigde release.
