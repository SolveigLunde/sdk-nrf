# Maximum throughput sample with PAwR (Periodic Advertising with Responses)

## Overview 
This sample demonstrates maximum throughput capabilities using Bluetooth LE Periodic Advertising with Responses (PAwR). It's designed to optimize communication between mutiple devices while respecting hardware buffer constraints. 

## Key features
- Dynamic device count configuration (1-255 devices)
- Automatic packet size optimization
- Real-time buffer monitoring
- Support for 2M PHY
- Automatic retransmission handling

## Hardware constraints
- Maximum buffer size: 1 650 bytes (CONFIG_BT_CTLR_ADV_DATA_LEN_MAX)
- Individual response limit: 247 bytes (BLE spec)

## Optimal Configurations
### Recommended Setup 
- 16 devices with 102 bytes per packet
- 1632 bytes total (98.9% buffer efficiency)
- ~69,5 KB/s throughput at 150ms interval

### Alternative Configurations
#### 1. Maximum Efficiency 
- 8 devices x 206 bytes = 1 648 bytes (99,9%)
- 70.1 KB/s throughput

#### 2. High Device Count
- 12 devices x 137 bytes = 1644 bytes (99,6%)
- 70,1 KB/s throughput

#### 3. Extended Setup
- 20 devices x 82 bytes = 1640 bytes (99,4%)
- 25 devices x 65 bytes = 1625 bytes (98,5%)

## Configuration

### Setting Device Count
#### 1. kconfig
config BT_MAX_THROUGHPUT_DEVICES
    int "Number of PAwR scanner devices"
    default 16 <----
    range 1 255

#### 2. prj.conf
CONFIG_BT_MAX_THROUGHPUT_DEVICES=16 <----
#### 3. during build
west build -b <board target> -- -DCONFIG_BT_MAX_THROUGHPUT_DEVICES=16 

## Building and Runnning
### 1. Build the project
cd periodic_adv_rsp
west build -b <board target>

# Build synchronizer
cd ../periodic_sync_rsp
west build -b <board target>

### 2. Run the demo
./run_devices.sh

## System Components
### 1. Advertiser (periodic_adv_rsp)
- Configures total device count
- Calculates optimal packet sizes
- Manages buffer constraints
- Handles timing and synchronization

### 2. Synchronizer (periodic_sync_rsp)
- Auto-configures based on advertiser settings
- Dynamically adjusts response sizes
- Maintains compatability with advertiser
- Falls back to optimal defaults if needed

## Monitoring and Debugging
### Real-time monitoring
tail -f throughput.log
- Check buffer utilization in console output

### Common Issues and Solutions
#### 1. Buffer Overflow Warnings
- Reduce device count or packet size
- System will auto-adjust packet sizes
#### 2. Low Throughput
- Verify 2M PHY is enabled
- Check interval settings
- Monitor for packet loss
#### 3. Connection Issues
- Verify PAST (Periodic Advertising Sync Transfer)
- Check device discovery
- Monitor connection parameters

## Performance Metrics
- Throughput: 69.5 - 104.5 KB/s (depending on configuration)
- Buffer efficiency: 98-99.9%
- Scaling: Performance advantage over classical BLE increases with more devices
- Reliabilty: Automatic retransmission handling for lost packets

## Best practices
1. Start with recommended configuration (16 devices)
2. Monitor buffer utilization before changing device count
3. Use 150 ms interval for balanced performance
4. Enable 2M PHY for maximum throughput
5. Let the system auto-calculate packet sizes

This implementation provides a solution for high-throughput Bluetooth LE communication with multiple devices, automatically balancing device count, packet size, and buffer constraints for optimal performance. 