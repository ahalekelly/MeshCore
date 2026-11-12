# SX1262 AGC lockup test

Reproduces the SX1262 AGC lockup that bit 0 of register 0x08B5 prevents.

This branch is companion-v1.18.0 plus debug commands on the companion's CLI, among them `radiotx <kHz> <dBm> <ms>` (unmodulated carrier), `radioreg` / `radiowreg` (read and write a register), `radiodiag2` (live RSSI) and `radiofix rx` / `radiofix agc` (receive restart, AGC reset).

Flash a USB companion build of this branch to two SX1262 boards with antennas about 20 cm apart, then:

    uv run bursttest.py VICTIM_PORT TX_PORT out --bursts 120 --bits ab --khz 907525 --dbm -9 --burst-ms 30
    uv run burststats.py out

`--khz` and `--dbm` set the carrier, which must be legal where you are and off any channel in use. The lock rate peaks in a level window about 2 dB wide that differs by board and frequency, so sweep `--dbm` if nothing locks.
