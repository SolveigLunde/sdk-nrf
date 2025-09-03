# Maximum Throughput Sample using PAwR

This sample demonstrates achieving high throughput in one-to-many communication using Periodic Advertising with Responses (PAwR). It consists of two applications:
- `periodic_adv_rsp`: The advertiser application
- `periodic_sync_rsp`: The synchronizer application

## Overview

The sample showcases optimal PAwR parameter calculations to maximize throughput while respecting Bluetooth LE stack limitations. The key optimization revolves around efficiently utilizing the available 1650-byte buffer (defined by `CONFIG_BT_CTLR_ADV_DATA_LEN_MAX`) while supporting multiple synchronizing devices.

## Key Features

- Automatic calculation of optimal PAwR parameters
- Support for multiple synchronizing devices (up to ~100 devices)
- Uses LE 2M PHY for enhanced throughput
- Dynamic packet size optimization based on device count

## Buffer and Throughput Calculations

The sample uses a sophisticated algorithm to calculate optimal parameters based on two main constraints:
1. Total buffer size limit: 1650 bytes
2. Individual response size limit: 247 bytes (BLE spec limit)

### Throughput Calculation Formula

For a given number of devices (N), the throughput is calculated as:

```
Packet Size = min((1650 - 20) / N, 247)  // 20 bytes overhead
Total Bytes per Interval = N * Packet Size
Throughput (bps) = (Total Bytes * 8 * 1000) / Interval_ms
```

### Example Configurations

Using 100ms interval (optimal for maximum throughput):

| Devices | Packet Size | Total Buffer Usage | Throughput |
|---------|-------------|-------------------|------------|
| 16      | 102 bytes   | 1632 bytes (98.9%) | ~13.0 Mbps |
| 32      | 51 bytes    | 1632 bytes (98.9%) | ~13.0 Mbps |
| 64      | 25 bytes    | 1600 bytes (97.0%) | ~12.8 Mbps |
| 100     | 16 bytes    | 1600 bytes (97.0%) | ~12.8 Mbps |

Note: The actual achievable throughput may be lower due to various factors like RF conditions, processing overhead, etc.

## Configuration Options

Key configuration parameters in `prj.conf`:

```conf
CONFIG_BT_MAX_THROUGHPUT_DEVICES=32        # Number of devices (default: 16)
CONFIG_BT_MAX_THROUGHPUT_PAWR_INTERVAL_MS=100  # Advertising interval in ms
CONFIG_BT_CTLR_ADV_DATA_LEN_MAX=1650      # Maximum buffer size
```

### Interval Selection

The advertising interval significantly impacts throughput:
- 100ms: Highest throughput, higher power consumption
- 150ms: Balanced throughput and power consumption
- 200ms: Lower power consumption, reduced throughput

## Buffer Utilization

The sample automatically optimizes packet sizes to maximize buffer utilization while staying within the 1650-byte limit. The calculation includes:
1. 20 bytes overhead reservation
2. Even distribution of remaining buffer space among devices
3. Clamping to BLE spec limits (247 bytes per response)

## Implementation Details

The core optimization logic is implemented in `calculate_optimal_config()` function, which:
1. Calculates maximum packet size that fits in buffer for given device count
2. Clamps values to BLE specification limits
3. Calculates expected throughput and buffer efficiency
4. Sets appropriate PAwR timing parameters

The sample automatically validates configurations to ensure they stay within buffer limits and provides detailed debug output about the chosen parameters.