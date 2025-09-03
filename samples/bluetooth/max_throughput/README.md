# High throughput PAwR sample

## Overview 
This sample showcases high-throughput communication using Bluetooth LE Periodic Advertising with Responses (PAwR). 
It enables efficient data exchange between one advertiser and multiple synchronized scanners, while respecting BLE hardware buffer limits.
The system dynamically calculates packet sizes and transmission intervals to maximize buffer usage and throughput.

## Key features
- Dynamic device count configuration (1-255 devices)
- Automatic packet size optimization
- Real-time buffer monitoring
- Support for 2M PHY
- Automatic retransmission handling

## System architecture

| Role         | Component              | Responsibilities                                   |
|--------------|------------------------|----------------------------------------------------|
| Advertiser   | `periodic_adv_rsp`     | Sets PAwR timing, allocates response slots, broadcasts sync info |
| Synchronizer | `periodic_sync_rsp`    | Syncs to PAwR, listens for requests, transmits response packets |


## Setup and build

### 1. Clone and Prepare

```bash
cd periodic_adv_rsp
west build -b <board_target>

cd ../periodic_sync_rsp
west build -b <board_target>

cd ..
./run_devices.sh
```

## Configurations and optimization
### Recommended Setup 
- 16 devices with 102 bytes per packet
- 1632 bytes total (98.9% buffer efficiency)
- ~69,5 KB/s throughput at 150ms interval

### Alternative configurations
| Devices | Packet Size | Buffer Usage | Efficiency | Throughput  |
| ------- | ----------- | ------------ | ---------- | ----------- |
| 8       | 206 bytes   | 1,648 bytes  | 99.9%      | 70.1 KB/s   |
| 12      | 137 bytes   | 1,644 bytes  | 99.6%      | 70.1 KB/s   |
| 20      | 82 bytes    | 1,640 bytes  | 99.4%      | \~66.0 KB/s |
| 25      | 65 bytes    | 1,625 bytes  | 98.5%      | \~62.5 KB/s |

### Device count configuration
#### Option 1: Kconfig
config BT_MAX_THROUGHPUT_DEVICES
    int "Number of PAwR scanner devices"
    default 16
    range 1 255
#### Option 2: prj.conf
CONFIG_BT_MAX_THROUGHPUT_DEVICES=16
#### Option 3: Build-time override
west build -b <board_target> -- -DCONFIG_BT_MAX_THROUGHPUT_DEVICES=16

## Performance metrics

| Metric            | Value                            |
| ----------------- | -------------------------------- |
| Throughput Range  | 69.5 KB/s – 104.5 KB/s           |
| Max Buffer Usage  | 1,650 bytes (BLE hardware limit) |
| Packet Size Range | 65–247 bytes                     |
| Max Device Count  | 255 (configurable)               |
| Buffer Efficiency | 98% – 99.9%                      |
| Interval Used     | 150 ms (recommended), tunable    |

## Monitoring and debugging

### Real-time output
- Throughput info is printed every second to console
- Response slots are tracked and retransmission bitmaps reported

### Common issues and fixes
| Issue                 | Solution                                        |
| --------------------- | ----------------------------------------------- |
| **Buffer Overflow**   | Reduce packet size or device count              |
| **Low Throughput**    | Ensure 2M PHY is active; use optimal intervals  |
| **No Device Sync**    | Check PAST support; verify BLE device discovery |
| **Response Not Sent** | Validate retransmission bitmaps and slot timing |


## Best practices
1. Start with recommended configuration (16 devices)
2. Monitor buffer utilization before changing device count
3. Use 150 ms interval for balanced performance, 100 ms for best throughput
4. Enable 2M PHY for maximum throughput
5. Let the system auto-calculate packet sizes

## Additional notes

- Tested with Nordic Semiconductors hardware using the Zephyr RTOS
- Uses CONFIG_BT_CTRL_ADV_DATA_LEN_MAX = 1650 as the upper bound
- Compatible with any BLE hardware supporting PAwR and PAST
