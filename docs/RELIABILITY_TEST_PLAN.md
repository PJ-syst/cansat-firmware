# Reliability and rehearsal plan

Run the following with the deployment load replaced by an inert indicator. Preserve raw
telemetry logs and firmware logs for every run.

1. Cold boot twice with the complete payload and verify a safe mechanism output.
2. Repeat with each noncritical peripheral absent, then with the barometer absent.
3. Inject invalid NMEA, truncated commands, wrong team IDs, command bursts, and UART loss.
4. Run recorded pressure profiles containing spikes, noise, slow descent, and GPS loss.
5. Verify the expected state sequence and ensure high-altitude stability cannot declare landing.
6. Force one mechanism-operation failure and confirm the firmware enters `ERROR` without retrying.
7. Measure packet cadence, UART errors, queue overruns, stack margin, minimum heap, and resets.
8. Operate for the planned mission duration plus margin from the exact flight battery.
9. Measure radio packet loss and range using the final antennas, frequency, power, and enclosure.
10. Repeat the complete end-to-end rehearsal twice from a cold boot.

Pass criteria: no unintended output pulse, duplicate mechanism attempt, false transition,
watchdog reset, growing memory use, telemetry truncation, or unreported data-source loss.
