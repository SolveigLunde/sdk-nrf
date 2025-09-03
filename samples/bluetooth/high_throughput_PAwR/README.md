# Bluetooth PAwR High throughput sample

This sample demonstrates high-througput in many-to-one communication using Periodic Advertising with Responses (PAwR). It is designed to showcase how multiple synchronizer devices can efficiently transmit data to a single advertiser, maximizing the throughput from synchronizers to the advertiser. 

## Overview

Purpose: Showcase maximum achievable throughput in a many-to-one Bluetooth PAwR scenario. 
Architecture: 
- Advertiser: Periodically advertises a single subevent
- Synchronizers: Multiple devices (configurable) synchronize to the advertiser and respond in their allocated response slots. 
- Data flow: Each synchronizer sends a large packet in its slot. The advertiser collects all responses and calculates throughput. 

## Key features

- Configurable Device count: 
Set the number of synchronizers via ```CONFIG_BT_MAX_THROUGHPUT_DEVICES=xxx``` in periodic_adv_rsp/prj.conf. This value is used throughpout the sample and simulation scripts- 
- Maximized response size: 
Each synchronizer sends the largest possible packet (sample uses 230 bytes, but protocol max is 247 bytes) to maximize throughput. 
- Packetloss handling: 
The advertiser keeps track of which response slots it did not receive data in, then informes the corresponding synchronizer that its last message was not received. Currently this only detects packetloss and does not attempt to retransmit as the sample is meant to showcase maximum throughput and therefor cannot stop to retransmit data. 

## Setup and build

```bash
cd periodic_adv_rsp
west build -b <board_target>

cd ../periodic_sync_rsp
west build -b <board_target>

cd ..
./run_devices.sh
```
## Configuration

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

### Packet size: 
Defined in source as ```MAX_INDIVIDUAL_RESPONSE_SIZE``` (sample uses 230 for stability)

## Advertising interval 
Calculated in the advertiser code to fit all responses and maximize throughput. 


## How it works

### 1. Advertiser setup: 
- Initializes periodic advertising with one subevent and multiple response slots. 
- Waits for synchronizers to connect and configures their response slots via GATT
- Collects responses from all synchronizers in each advertising interval. 
- Tracks throughput and lost packets. 

### 2. Synchronizer setup: 
- Synchronizes to advertisers's periodic advertising using PAST. 
- Receives slot allocation via GATT
- Responds in its allocated slot with a large packet. 
- Handles retransmission requests from the advertiser. 

### Throughput calculation
- Throughput is calculated by the advertiser based on the total bytes received per interval

## Future improvements

- Optimize the interval calculations. The current sample uses very conservative intervals due to some issues with the inital connections. To get better throughput the intervals can be decreased by tweaking the different parameters or changing the intervals after the inital connections where made. 
- Set up all connections before advertising starts to make the connection phase faster.
- Implement support for other board configurations
- Test on physical DKs
- Create sample test to verify sample