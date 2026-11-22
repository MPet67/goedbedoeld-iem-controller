# Veilig publiceren — controlelijst

1. **Niet nu meteen Public klikken.** Controleer volledige Git-geschiedenis en alle andere bestanden die in GitHub staan. Deze scan betrof uitsluitend het lokale starterpakket.
2. De eerder gedeelde test-AP-credential was in de oorspronkelijke firmware en simulator opgenomen. **Wijzig de AP-credential op gebruikte apparaten**; de referentie-firmware blijft bevroren en is dus niet automatisch aangepast.
3. Publiceer alleen deze geschoonde publicatievariant (niet de oorspronkelijke v2.0 RC-bron in openbare geschiedenis). Maak bij twijfel een **nieuwe publieke repository met schone historie**. Houd het originele werkarchief privaat.
4. Simulator vereist `local_secrets.h` naast de `.ino`; neem echte wachtwoorden, `.env`, sleutels, dumps, `.bin`-backups of NVS-data niet op in Git.
5. Controleer GitHub **Settings > Secrets and variables > Actions**, workflowbestanden, Discussions, Issues, commits en eerdere release-assets.
6. `.gitignore` voorkomt nieuw tracken, maar verwijdert **geen reeds gecommitte bestanden**. Bekijk de inhoud van de commits voordat je zichtbaarheidsinstellingen wijzigt.
7. Maak public pas na credentialrotatie en na het controleren van de commitgeschiedenis. Controleer daarna de publieke bestanden in een uitgelogd browservenster.
8. OTA van public releases vereist nog afzonderlijke integriteits-, authenticiteits- en hersteltaken. Publiceer geen OTA .bin voordat de flashprocedure is getest.

## Belangrijke verschillen t.o.v. bevroren referentie

- Freenove: niet-gebruikte hardcoded setup-AP-credential verwijderd; automatisch ingevuld test-Wi-Fi-profiel verwijderd. Bestaande NVS-profielen worden niet gewijzigd.
- Simulator: AP-wachtwoord verplaatst naar lokaal, niet-gecommit headerbestand. Voor compilatie eerst dit bestand aanmaken.
- Geen wijzigingen aan mixerprotocol, encoderlogica of displaycode beoogd; **niet hardware-gevalideerd**.
