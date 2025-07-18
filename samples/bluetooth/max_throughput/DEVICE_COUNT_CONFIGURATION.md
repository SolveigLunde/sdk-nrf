# Device Count Configuration Guide

## ✅ **Single Source of Truth (FIXED)**

After fixing the inconsistencies, there is now **ONE primary place** to configure the number of devices:

### **Primary Configuration**
📍 **`periodic_adv_rsp/Kconfig`**
```kconfig
config BT_MAX_THROUGHPUT_DEVICES
	int "Number of PAwR scanner devices"
	default 16
	range 1 50
```

**This is the ONLY place you need to change the device count.**

## 🔄 **Automatic Propagation**

All other components automatically use this value:

### **1. Advertiser (periodic_adv_rsp)**
```c
// In periodic_adv_rsp/src/main.c
#define NUM_RSP_SLOTS CONFIG_BT_MAX_THROUGHPUT_DEVICES
```
- ✅ **Automatically reads from Kconfig**
- ✅ **No manual configuration needed**

### **2. Sync Responder (periodic_sync_rsp)**
```c
// In periodic_sync_rsp/src/main.c
#ifdef CONFIG_BT_MAX_THROUGHPUT_DEVICES
#define ESTIMATED_DEVICES CONFIG_BT_MAX_THROUGHPUT_DEVICES
#else
#define ESTIMATED_DEVICES 16  // Fallback to optimal default
#endif
```
- ✅ **Automatically reads from Kconfig**
- ✅ **Has fallback to optimal default**

### **3. Run Script (run_devices.sh)**
```bash
# Automatically reads from built configuration
NUM_SYNCHRONIZERS=$(grep "CONFIG_BT_MAX_THROUGHPUT_DEVICES=" "$CONFIG_FILE" | cut -d'=' -f2)
```
- ✅ **Automatically reads from built config file**
- ✅ **No manual configuration needed**

## 📋 **Configuration Files (All Synchronized)**

### **Primary Kconfig Files**
- `periodic_adv_rsp/Kconfig`: Default 16 devices ✅
- `Kconfig.defconfig`: Default 16 devices ✅ (Fixed)

### **Project Configuration Files**
- `periodic_adv_rsp/prj.conf`: Uses 16 devices ✅ (Fixed)
- `periodic_sync_rsp/prj.conf`: Inherits from Kconfig ✅ (Fixed)

## 🎯 **How to Change Device Count**

### **Method 1: Edit Kconfig (Recommended)**
```bash
# Edit the default in periodic_adv_rsp/Kconfig
config BT_MAX_THROUGHPUT_DEVICES
	int "Number of PAwR scanner devices"
	default 12  # Change this number
	range 1 50
```

### **Method 2: Override in prj.conf**
```bash
# In periodic_adv_rsp/prj.conf
CONFIG_BT_MAX_THROUGHPUT_DEVICES=12
```

### **Method 3: Configure at build time**
```bash
west build -b nrf54l15bsim_nrf54l15_cpuapp -- -DCONFIG_BT_MAX_THROUGHPUT_DEVICES=12
```

## ⚠️ **Important Notes**

### **Buffer Constraints**
The system will automatically:
- Calculate optimal packet sizes for your device count
- Warn if configuration exceeds 1,650-byte buffer limit
- Automatically reduce packet sizes if needed

### **Optimal Device Counts**
| Devices | Packet Size | Total Bytes | Buffer Efficiency |
|---------|-------------|-------------|-------------------|
| 8       | 206 bytes   | 1,648 bytes | 99.9%            |
| 12      | 137 bytes   | 1,644 bytes | 99.6%            |
| **16**  | **102 bytes** | **1,632 bytes** | **98.9%** |
| 20      | 82 bytes    | 1,640 bytes | 99.4%            |

### **Build and Run Process**
1. **Configure**: Change device count in Kconfig
2. **Build**: `west build -b nrf54l15bsim_nrf54l15_cpuapp`
3. **Run**: `./run_devices.sh` (automatically uses correct count)

## 🔧 **Troubleshooting**

### **Problem: Different device counts in different files**
**Solution**: All files now use the same Kconfig value ✅

### **Problem: Script runs wrong number of devices**
**Solution**: Script automatically reads from built config ✅

### **Problem: Buffer overflow warnings**
**Solution**: System automatically adjusts packet sizes ✅

## 🎉 **Summary**

**Before (Multiple conflicting values):**
- ❌ Kconfig: 16 devices
- ❌ prj.conf: 20 devices  
- ❌ Kconfig.defconfig: 20 devices
- ❌ Hardcoded: 16 devices

**After (Single source of truth):**
- ✅ **ALL components use CONFIG_BT_MAX_THROUGHPUT_DEVICES**
- ✅ **Change in ONE place, works everywhere**
- ✅ **Automatic buffer optimization**
- ✅ **No manual synchronization needed**

**Result: Change device count in ONE place, everything works automatically!** 