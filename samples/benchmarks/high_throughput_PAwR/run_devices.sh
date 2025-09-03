#!/bin/bash

# === CONFIGURATION ===
# Get number of devices from Kconfig
CONFIG_FILE="periodic_adv_rsp/build/periodic_adv_rsp/zephyr/.config"
if [ -f "$CONFIG_FILE" ]; then
    NUM_SYNCHRONIZERS=$(grep "CONFIG_BT_MAX_THROUGHPUT_DEVICES=" "$CONFIG_FILE" | cut -d'=' -f2)
else
    echo "Error: Config file not found. Please build the project first."
    exit 1
fi

# === CLEANUP ===
echo "Cleaning up /tmp/bs_azure/..."
rm -rf /tmp/bs_azure/*

# Clean up throughput log
echo "Cleaning up throughput.log..."
echo -n > throughput.log

##OBS: Remember to change the name of the build directory!!

# === START ADVERTISER ===
echo "Starting advertiser..."
./periodic_adv_rsp/build/periodic_adv_rsp/zephyr/zephyr.exe -s=2 -d=0 &

# === START SYNCHRONIZERS ===
for ((i=1; i<=NUM_SYNCHRONIZERS; i++)); do
    echo "Starting synchronizer $i..."
    ./periodic_sync_rsp/build/periodic_sync_rsp/zephyr/zephyr.exe -s=2 -d=$i &
done

# === START PHY SIMULATION ===
TOTAL_DEVICES=$((NUM_SYNCHRONIZERS + 1))
echo "Starting bs_2G4_phy_v1 with $TOTAL_DEVICES devices..."
cd /work/../../../../bsim/bsim/bin && ./bs_2G4_phy_v1 -s=2 -D=$TOTAL_DEVICES -channel=multiatt -argschannel -at=60 -atextra=50
