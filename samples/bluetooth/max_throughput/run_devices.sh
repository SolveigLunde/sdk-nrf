#!/bin/bash

# === CONFIGURATION ===
NUM_SYNCHRONIZERS=10  # Change this number as needed

# === CLEANUP ===
echo "Cleaning up /tmp/bs_azure/..."
rm -rf /tmp/bs_azure/*


##OBS: Remember to change the name of the build directory!!

# === START ADVERTISER ===
echo "Starting advertiser..."
./periodic_adv_rsp/build_p/periodic_adv_rsp/zephyr/zephyr.exe -s=2 -d=0 &

# === START SYNCHRONIZERS ===
for ((i=1; i<=NUM_SYNCHRONIZERS; i++)); do
    echo "Starting synchronizer $i..."
    ./periodic_sync_rsp/build_p/periodic_sync_rsp/zephyr/zephyr.exe -s=2 -d=$i &
done

# === START PHY SIMULATION ===
TOTAL_DEVICES=$((NUM_SYNCHRONIZERS + 1))
echo "Starting bs_2G4_phy_v1 with $TOTAL_DEVICES devices..."
cd /work/../../../../bsim/bsim/bin && ./bs_2G4_phy_v1 -s=2 -D=$TOTAL_DEVICES
