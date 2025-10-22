## High-throughput PAwR (Periodic Advertising with Responses)

This sample demonstrates one-to-many uplink using Bluetooth LE Periodic Advertising with Responses (PAwR) on the LE 2M PHY. It is organized as two applications that run together:
- `periodic_adv_rsp`: PAwR advertiser/host that schedules subevents and receives responses from multiple devices.
- `periodic_sync_rsp`: PAwR synchronizer/responder that joins a periodic advertiser and transmits data in assigned response slots.

### What it showcases
- One-to-many uplink using PAwR with a lightweight join/claim protocol.
- Claim/ACK handshake to assign response slots, bitmap-driven retransmission, and idle-slot reclamation.
- Real-time throughput measurements and theoretical capacity reporting on the advertiser.
- Tested at high device counts (100+ responders) with 2M PHY.

## Folder layout
- `periodic_adv_rsp/`: Advertiser app (creates PAwR set, encodes control frames, receives responses, prints throughput).
- `periodic_sync_rsp/`: Synchronizer app (discovers periodic advertiser, joins, chooses an open slot, responds with test data).
- `run_devices.sh`: Helper to launch multiple instances in BabbleSim after building both apps.
- `data.txt`: Collected measurements from simulations and DK tests (various device counts and tuning variants).
- `howtorun.txt`: Notes for flashing on real DKs and viewing UART logs.

## How it works
At a high level, each periodic advertising event contains one subevent with N response slots.
The advertiser embeds a small Manufacturer Specific Data (MSD) control frame in the subevent data:
- Open-slots bitmap: which slots are currently unassigned.
- ACK list: acknowledgments for recently accepted claims (token, slot).
- Retransmit bitmap: which slots should retransmit in the next event.

Responders:
1) Sync to the periodic advertiser and listen for the control frame.
2) If unassigned, probabilistically attempt a CLAIM in an open slot by sending `[0xC1][token(4B)]`.
3) Upon seeing an ACK for its token, the responder keeps sending payloads in its assigned slot.
4) If the retransmit bit for its slot is set, it repeats the previous payload.

The advertiser counts received bytes and periodically prints measured kbps along with a theoretical estimate derived from configured timing, payload size (247 B max individual response), and number of slots.

## Requirements
- nRF Connect SDK with Zephyr and west installed.
- Hardware: Nordic DKs that support PAwR on 2M PHY (e.g., `nrf54l15dk/nrf54l15/cpuapp`).
- Optional: BabbleSim installed if you want to run large-scale simulations locally.

## Build and run on real hardware
You can build from each role subdirectory, or from the sample root by passing the source path via `-s`.

1) Configure device count for the advertiser
   - Set the number of expected responders in `periodic_adv_rsp/prj.conf` via `CONFIG_BT_MAX_THROUGHPUT_DEVICES` (or via Kconfig). This should match the number of synchronizers you plan to run.

2) Build
```bash
# Advertiser (from this sample root)
west build -p always -b nrf54l15dk/nrf54l15/cpuapp -d periodic_adv_rsp/build -s periodic_adv_rsp

# Synchronizer (repeat build for as many DKs as you plan to flash)
west build -p always -b nrf54l15dk/nrf54l15/cpuapp -d periodic_sync_rsp/build -s periodic_sync_rsp
```

3) Flash
```bash
# Advertiser (replace <id> with your DK serial)
west flash -d periodic_adv_rsp/build --snr <id>

# Synchronizers (repeat for each DK with its own serial)
west flash -d periodic_sync_rsp/build --snr <id>
```

4) View logs (115200 baud)
Open one terminal per device and run, adjusting the TTY to match the board:
```bash
/bin/stty -F /dev/ttyACM0 raw -echo -icrnl -onlcr -ixon -ixoff 115200
/bin/cat /dev/ttyACM0
```
If VCOM0 is quiet, try the board’s other VCOM port (e.g. `/dev/ttyACM1`). Press the DK’s RESET after connecting the terminal.

Tips from `howtorun.txt`:
- Ensure logging/UART options are enabled in both `prj.conf` files if you need console output.
- Set the device count correctly before building the advertiser; a mismatch can cause startup stalls.

## Run with BabbleSim (optional)
This repository includes `run_devices.sh` to quickly spin up the advertiser plus N synchronizers after building both apps for a BabbleSim-capable target. The script:
- Reads `CONFIG_BT_MAX_THROUGHPUT_DEVICES` from the advertiser’s `.config` to determine the number of responders to start.
- Launches one advertiser and NUM_SYNCHRONIZERS responders.
- Starts the 2.4 GHz PHY (`bs_2G4_phy_v1`) with the matching device count.

Usage outline:
1) Build both apps for a BabbleSim-supported board (follow your environment’s guidance).
2) From this sample root, run:
```bash
./run_devices.sh
```
Adjust paths inside the script if your build directories differ.

## Configuration
Key options (see each role’s `prj.conf` and `periodic_adv_rsp/Kconfig`):
- `CONFIG_BT_CTLR_PHY_2M=y`: Use LE 2M PHY.
- `CONFIG_BT_PER_ADV=y` and `CONFIG_BT_PER_ADV_RSP=y`: Enable PAwR support on advertiser.
- `CONFIG_BT_PER_ADV_SYNC=y` and `CONFIG_BT_PER_ADV_SYNC_RSP=y`: Enable PAwR sync and responses on synchronizer.
- `CONFIG_BT_MAX_THROUGHPUT_DEVICES=<N>`: Number of responder slots (advertiser side).
- Buffer sizes and controller counts tuned for large uplink PDUs: `CONFIG_BT_CTLR_DATA_LENGTH_MAX=251`, larger ACL buffers/MTUs, and response buffer counts on both roles.

Timing on the advertiser (`periodic_adv_rsp/src/main.c`):
- Single subevent, N response slots.
- Per-slot spacing chosen to fit 247 B responses on 2M PHY with minimal guard times.
- The advertiser prints a theoretical kbps based on N, payload size, and the computed advertising interval.
- Idle-slot reclamation frees slots that fail to respond for 3 consecutive events.

Responder payload size (`periodic_sync_rsp/src/main.c`):
- Defaults to near-max PDU (`247 - 3` bytes) to avoid overhead corner cases.

## Measured results (excerpt)
See `data.txt` for the full set. Representative measurements (advertiser prints):
- 1 device: ~175 kbps
- 2 devices: ~521 kbps
- 5 devices: ~869 kbps 
- 32 devices: ~1337 kbps 
- 64 devices: ~1470 kbps 
- 128 devices: ~1517 kbps 
- 130–155 devices: increased timing pressure; efficiencies ~80–90%, with instability beyond ~128 in the provided tuning.

Actual throughput depends on radio conditions, controller limits, and timing parameters. Increase safety margins or reduce N if you observe missed indications.

## Troubleshooting
- No logs: Ensure `CONFIG_UART_CONSOLE=y` and logging options are enabled in both `prj.conf` files, and that you’re on the correct VCOM.
- Startup stalls: Verify the advertiser’s `CONFIG_BT_MAX_THROUGHPUT_DEVICES` matches the number of synchronizers you’re running.
- High N unstable: Reduce `CONFIG_BT_MAX_THROUGHPUT_DEVICES`, or increase guard times/intervals in the advertiser timing function.
- BabbleSim path issues: Edit `run_devices.sh` to match your build output and `bsim` installation path.

## License
Apache-2.0 (see the repository’s LICENSE file).