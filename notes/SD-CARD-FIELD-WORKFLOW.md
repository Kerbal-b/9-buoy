# SD card workflow between buoy tests

For each field test, treat the buoy SD card as the source record. Keep the local archive and analysis separate from meeting notes.

1. **Download the entire card before cleanup.** Copy every session folder and the card's `logs/`, `calibration/`, `missions/`, `config/`, and other files to a dated folder under `field-data/`. Record the card listing, file sizes, and a SHA-256 manifest. Include empty files in the inventory.
2. **Verify the local copy.** Compare the downloaded file list and sizes with the card listing, and recheck the local hashes. Resolve missing, truncated, or failed transfers before proceeding. Keep the original downloads unchanged; derive analysis files separately.
3. **Analyze and document the test.** Produce the combined observations and battery analysis, inspect mission and calibration events, and write a dated `notes/sessions/` meeting record linking the archive. Record unresolved data-quality issues.
4. **Clean the card before the next test.** Once the local copy is complete and reviewed, remove archived test sessions, old logs, and old calibration captures from the card. Preserve current configuration and mission files unless they are explicitly part of a planned reset. Record what was removed and the free space afterward in the next test's note.
5. **Confirm readiness.** Check that the card is mounted and writable and that a new session/calibration file can be created before going into the field.

The November 8, 2026 card was downloaded and analyzed locally; it was **not** cleaned as part of that extraction. This procedure applies to cleanup before the next test.
