#!/usr/bin/env python
#
# Copyright (c) 2016, The OpenThread Authors.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the copyright holder nor the
#    names of its contributors may be used to endorse or promote products
#    derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# pylint: disable=all

"""
>> Thread Host Controller Interface
>> Device : nRF_Connect_SDK_13_14 THCI
>> Class : nRF_Connect_SDK_13_14
"""
import base64
import functools
import ipaddress
import json
import logging
import random
import re
import socket
import time
import traceback
from abc import abstractmethod
from queue import Queue

import serial
from serial.serialutil import SerialException

import commissioner
from GRLLibs.ThreadPacket.PlatformPackets import (
    PlatformDiagnosticPacket,
    PlatformPackets,
)
from GRLLibs.UtilityModules.ModuleHelper import ModuleHelper, ThreadRunner
from GRLLibs.UtilityModules.Plugins.AES_CMAC import Thread_PBKDF2
from GRLLibs.UtilityModules.Test import (
    Thread_Device_Role,
    Device_Data_Requirement,
    MacType,
)
from GRLLibs.UtilityModules.enums import (
    PlatformDiagnosticPacket_Direction,
    PlatformDiagnosticPacket_Type,
    DevCapb,
    TestMode,
)
from commissioner_impl import OTCommissioner
from .IThci import IThci

# Replace by the actual version string for the vendor's reference device
OT11_VERSION = "OPENTHREAD"
OT12_VERSION = "OPENTHREAD"
OT13_VERSION = "OPENTHREAD"


# Supported device capabilites in this THCI implementation
OT11_CAPBS = DevCapb.V1_1
OT12_CAPBS = DevCapb.L_AIO | DevCapb.C_FFD | DevCapb.C_RFD
OT12BR_CAPBS = DevCapb.C_BBR | DevCapb.C_Host | DevCapb.C_Comm
OT13_CAPBS = DevCapb.C_FTD13 | DevCapb.C_MTD13
OT13BR_CAPBS = DevCapb.C_BR13 | DevCapb.C_Host13 | DevCapb.C_Comm13

ZEPHYR_PREFIX = "ot "
"""CLI prefix used for nRF_Connect_SDK_13_14 commands in Zephyr systems"""

LINESEPX = re.compile(r"\r\n|\n")
"""regex: used to split lines"""

LOGX = re.compile(
    r"((\[(-|C|W|N|I|D)\])"
    r"|(-+$)"  # e.x. ------------------------------------------------------------------------
    r"|(=+\[.*\]=+$)"  # e.x. ==============================[TX len=108]===============================
    r"|(\|.+\|.+\|.+)"  # e.x. | 61 DC D2 CE FA 04 00 00 | 00 00 0A 6E 16 01 00 00 | aRNz......n....
    r")"
)
"""regex used to filter logs"""

assert LOGX.match("[-]")
assert LOGX.match("[C]")
assert LOGX.match("[W]")
assert LOGX.match("[N]")
assert LOGX.match("[I]")
assert LOGX.match("[D]")
assert LOGX.match(
    "------------------------------------------------------------------------"
)
assert LOGX.match(
    "==============================[TX len=108]==============================="
)
assert LOGX.match(
    "| 61 DC D2 CE FA 04 00 00 | 00 00 0A 6E 16 01 00 00 | aRNz......n...."
)

# OT Errors
OT_ERROR_ALREADY = 24

logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s"
)

_callStackDepth = 0


def watched(func):
    func_name = func.__name__

    @functools.wraps(func)
    def wrapped_api_func(self, *args, **kwargs):
        global _callStackDepth
        callstr = "====" * _callStackDepth + "> %s%s%s" % (
            func_name,
            str(args) if args else "",
            str(kwargs) if kwargs else "",
        )

        _callStackDepth += 1
        try:
            ret = func(self, *args, **kwargs)
            self.log("%s returns %r", callstr, ret)
            return ret
        except Exception as ex:
            self.log("FUNC %s failed: %s", func_name, str(ex))
            raise
        finally:
            _callStackDepth -= 1
            if _callStackDepth == 0:
                print("\n")

    return wrapped_api_func


def retry(n, interval=0):
    assert n >= 0, n

    def deco(func):
        @functools.wraps(func)
        def retried_func(*args, **kwargs):
            for i in range(n + 1):
                try:
                    return func(*args, **kwargs)
                except Exception:
                    if i == n:
                        raise

                    time.sleep(interval)

        return retried_func

    return deco


def API(api_func):
    try:
        return watched(api_func)
    except Exception:
        tb = traceback.format_exc()
        ModuleHelper.WriteIntoDebugLogger(
            "Exception raised while calling %s:\n%s" % (api_func.__name__, tb)
        )
        raise


def commissioning(func):
    @functools.wraps(func)
    def comm_func(self, *args, **kwargs):
        self._onCommissionStart()
        try:
            return func(self, *args, **kwargs)
        finally:
            self._onCommissionStop()

    return comm_func


class CommandError(Exception):
    def __init__(self, code, msg):
        assert isinstance(code, int), code
        self.code = code
        self.msg = msg

        super(CommandError, self).__init__("Error %d: %s" % (code, msg))


class OpenThreadTHCI(object):
    LOWEST_POSSIBLE_PARTATION_ID = 0x1
    LINK_QUALITY_CHANGE_TIME = 100
    DEFAULT_COMMAND_TIMEOUT = 10
    firmwarePrefix = "OPENTHREAD/"
    DOMAIN_NAME = "Thread"
    MLR_TIMEOUT_MIN = 300
    NETWORK_ATTACHMENT_TIMEOUT = 10

    IsBorderRouter = False
    IsHost = False
    IsBeingTestedAsCommercialBBR = False
    IsReference20200818 = False

    externalCommissioner = None
    _update_router_status = False

    _cmdPrefix = ""
    _lineSepX = LINESEPX

    _ROLE_MODE_DICT = {
        Thread_Device_Role.Leader: "rdn",
        Thread_Device_Role.Router: "rdn",
        Thread_Device_Role.SED: "-",
        Thread_Device_Role.EndDevice: "rn",
        Thread_Device_Role.REED: "rdn",
        Thread_Device_Role.EndDevice_FED: "rdn",
        Thread_Device_Role.EndDevice_MED: "rn",
        Thread_Device_Role.SSED: "-",
    }

    def __init__(self, **kwargs):
        """initialize the serial port and default network parameters
        Args:
            **kwargs: Arbitrary keyword arguments
                      Includes 'EUI' and 'SerialPort'
        """
        self.intialize(kwargs)

    @abstractmethod
    def _connect(self):
        """
        Connect to the device.
        """

    @abstractmethod
    def _disconnect(self):
        """
        Disconnect from the device
        """

    @abstractmethod
    def _cliReadLine(self):
        """Read exactly one line from the device

        Returns:
            None if no data
        """

    @abstractmethod
    def _cliWriteLine(self, line):
        """Send exactly one line to the device

        Args:
            line str: data send to device
        """

    # Override the following empty methods in the dervied classes when needed
    def _onCommissionStart(self):
        """Called when commissioning starts"""

    def _onCommissionStop(self):
        """Called when commissioning stops"""

    def _deviceBeforeReset(self):
        """Called before the device resets"""

    def _deviceAfterReset(self):
        """Called after the device resets"""

    def _restartAgentService(self):
        """Restart the agent service"""

    def _beforeRegisterMulticast(self, sAddr, timeout):
        """Called before the ipv6 address being subscribed in interface

        Args:
            sAddr   : str : Multicast address to be subscribed and notified OTA
            timeout : int : The allowed maximal time to end normally
        """

    def __sendCommand(self, cmd, expectEcho=True):
        cmd = self._cmdPrefix + cmd
        # self.log("command: %s", cmd)
        self._cliWriteLine(cmd)
        if expectEcho:
            self.__expect(cmd, endswith=True)

    _COMMAND_OUTPUT_ERROR_PATTERN = re.compile(r"Error (\d+): (.*)")

    @retry(3)
    @watched
    def __executeCommand(self, cmd, timeout=DEFAULT_COMMAND_TIMEOUT, ignore_exec=False):
        """send specific command to reference unit over serial port

        Args:
            cmd: nRF_Connect_SDK_13_14 CLI string

        Returns:
            Done: successfully send the command to reference unit and parse it
            Value: successfully retrieve the desired value from reference unit
            Error: some errors occur, indicates by the followed specific error number
        """
        if self.logThreadStatus == self.logStatus["running"]:
            self.logThreadStatus = self.logStatus["pauseReq"]
            while (
                self.logThreadStatus != self.logStatus["paused"]
                and self.logThreadStatus != self.logStatus["stop"]
            ):
                pass

        try:
            self.__sendCommand(cmd)
            response = []

            t_end = time.time() + timeout
            while time.time() < t_end:
                line = self.__readCliLine()
                if line is None:
                    time.sleep(0.01)
                    continue
                line = line.replace("\x1b[J", "")
                # self.log("readline: %s", line)
                # skip empty lines
                if line:
                    response.append(line)

                if line.endswith("Done"):
                    break
                else:
                    m = OpenThreadTHCI._COMMAND_OUTPUT_ERROR_PATTERN.match(line)
                    if m is not None:
                        code, msg = m.groups()
                        raise CommandError(int(code), msg)
            else:
                if ignore_exec:
                    return True
                else:
                    raise Exception(
                        "%s: failed to find end of response: %s" % (self, response)
                    )

        except SerialException as e:
            self.log("__executeCommand() Error: " + str(e))
            self._disconnect()
            self._connect()
            raise e

        return response

    def __expect(self, expected, timeout=5, endswith=False):
        """Find the `expected` line within `times` tries.

        Args:
            expected    str: the expected string
            times       int: number of tries
        """
        # self.log('Expecting [%s]' % (expected))

        deadline = time.time() + timeout
        while True:
            line = self.__readCliLine()
            if line is not None:
                # self.log("readline: %s", line)
                pass

            if line is None:
                if time.time() >= deadline:
                    break

                self.sleep(0.01)
                continue

            matched = line.endswith(expected) if endswith else line == expected
            if matched:
                # self.log('Expected [%s]' % (expected))
                return

        raise Exception("failed to find expected string[%s]" % expected)

    def __readCliLine(self, ignoreLogs=True):
        """Read the next line from OT CLI.d"""
        line = self._cliReadLine()
        if ignoreLogs:
            while line is not None and LOGX.match(line):
                line = self._cliReadLine()

        return line

    @API
    def getVersionNumber(self):
        """get nRF_Connect_SDK_13_14 stack firmware version number"""
        return self.__executeCommand("version")[0]

    @API
    def discoverDeviceCapability(self):
        return self.__executeCommand("thread version")[0]

    def log(self, fmt, *args):
        try:
            msg = fmt % args
            # print('%s - %s - %s' % (self.port, time.strftime('%b %d %H:%M:%S'), msg))
            print(("%s %s" % (self.port, msg)))
        except Exception:
            pass

    def sleep(self, duration):
        if duration >= 1:
            self.log("sleeping for %ss ...", duration)
        time.sleep(duration)

    @API
    def intialize(self, params):
        """initialize the serial port with baudrate, timeout parameters"""
        self.mac = params.get("EUI")
        self.backboneNetif = params.get("Param8") or "eth0"
        self.extraParams = self.__parseExtraParams(params.get("Param9"))

        # Potentially changes `self.extraParams`
        self._parseConnectionParams(params)

        self.UIStatusMsg = ""
        self.AutoDUTEnable = False
        self.isPowerDown = False
        self._is_net = False  # whether device is through ser2net
        self.logStatus = {
            "stop": "stop",
            "running": "running",
            "pauseReq": "pauseReq",
            "paused": "paused",
        }
        self.joinStatus = {
            "notstart": "notstart",
            "ongoing": "ongoing",
            "succeed": "succeed",
            "failed": "failed",
        }
        self.logThreadStatus = self.logStatus["stop"]

        self.deviceConnected = False

        # init serial port
        self._connect()
        if not self.IsBorderRouter:
            self.__detectZephyr()
        self.__discoverDeviceCapability()
        self.UIStatusMsg = self.getVersionNumber()

        if self.firmwarePrefix in self.UIStatusMsg:
            self.deviceConnected = True
        else:
            self.UIStatusMsg = (
                "Firmware Not Matching Expecting "
                + self.firmwarePrefix
                + ", Now is "
                + self.UIStatusMsg
            )
            ModuleHelper.WriteIntoDebugLogger(
                "Err: nRF_Connect_SDK_13_14 device Firmware not matching.."
            )

        # Make this class compatible with Thread referenece 20200818
        self.__detectReference20200818()

    def __repr__(self):
        if self.connectType == "ip":
            return "[%s:%d]" % (self.telnetIp, self.telnetPort)
        else:
            return "[%s]" % self.port

    def _parseConnectionParams(self, params):
        """Parse parameters related to connection to the device

        Args:
            params: Arbitrary keyword arguments including 'EUI' and 'SerialPort'
        """
        self.port = params.get("SerialPort", "")
        # params example: {'EUI': 1616240311388864514L, 'SerialBaudRate': None, 'TelnetIP': '192.168.8.181', 'SerialPort': None, 'Param7': None, 'Param6': None, 'Param5': 'ip', 'TelnetPort': '22', 'Param9': None, 'Param8': None}
        self.log("All parameters: %r", params)

        try:
            ipaddress.ip_address(self.port)
            # handle TestHarness Discovery Protocol
            self.connectType = "ip"
            self.telnetIp = self.port
            self.telnetPort = 22
            self.telnetUsername = (
                "pi" if params.get("Param6") is None else params.get("Param6")
            )
            self.telnetPassword = (
                "raspberry" if params.get("Param7") is None else params.get("Param7")
            )
        except ValueError:
            self.connectType = (params.get("Param5") or "usb").lower()
            self.telnetIp = params.get("TelnetIP")
            self.telnetPort = (
                int(params.get("TelnetPort")) if params.get("TelnetPort") else 22
            )
            # username for SSH
            self.telnetUsername = (
                "pi" if params.get("Param6") is None else params.get("Param6")
            )
            # password for SSH
            self.telnetPassword = (
                "raspberry" if params.get("Param7") is None else params.get("Param7")
            )

    @watched
    def __parseExtraParams(self, Param9):
        """
        Parse `Param9` for extra THCI parameters.

        `Param9` should be a JSON string encoded in URL-safe base64 encoding.

        Defined Extra THCI Parameters:
        - "cmd-start-otbr-agent"   : The command to start otbr-agent (default: systemctl start otbr-agent)
        - "cmd-stop-otbr-agent"    : The command to stop otbr-agent (default: systemctl stop otbr-agent)
        - "cmd-restart-otbr-agent" : The command to restart otbr-agent (default: systemctl restart otbr-agent)
        - "cmd-restart-radvd"      : The command to restart radvd (default: service radvd restart)

        For example, Param9 can be generated as below:
        Param9 = base64.urlsafe_b64encode(json.dumps({
            "cmd-start-otbr-agent": "service otbr-agent start",
            "cmd-stop-otbr-agent": "service otbr-agent stop",
            "cmd-restart-otbr-agent": "service otbr-agent restart",
            "cmd-restart-radvd": "service radvd stop; service radvd start",
        }))

        :param Param9: A JSON string encoded in URL-safe base64 encoding.
        :return: A dict containing extra THCI parameters.
        """
        if not Param9 or not Param9.strip():
            return {}

        jsonStr = base64.urlsafe_b64decode(Param9)
        params = json.loads(jsonStr)
        return params

    @API
    def closeConnection(self):
        """close current serial port connection"""
        self._disconnect()

    def __disableRouterEligible(self):
        """disable router role"""
        cmd = "routereligible disable"
        self.__executeCommand(cmd)

    def __setDeviceMode(self, mode):
        """set thread device mode:

        Args:
           mode: thread device mode
           r: rx-on-when-idle
           s: secure IEEE 802.15.4 data request
           d: full thread device
           n: full network data

        Returns:
           True: successful to set the device mode
           False: fail to set the device mode
        """
        cmd = "mode %s" % mode
        return self.__executeCommand(cmd)[-1] == "Done"

    def __setRouterUpgradeThreshold(self, iThreshold):
        """set router upgrade threshold

        Args:
            iThreshold: the number of active routers on the Thread network
                        partition below which a REED may decide to become a Router.

        Returns:
            True: successful to set the ROUTER_UPGRADE_THRESHOLD
            False: fail to set ROUTER_UPGRADE_THRESHOLD
        """
        cmd = "routerupgradethreshold %s" % str(iThreshold)
        return self.__executeCommand(cmd)[-1] == "Done"

    def __setRouterDowngradeThreshold(self, iThreshold):
        """set router downgrade threshold

        Args:
            iThreshold: the number of active routers on the Thread network
                        partition above which an active router may decide to
                        become a child.

        Returns:
            True: successful to set the ROUTER_DOWNGRADE_THRESHOLD
            False: fail to set ROUTER_DOWNGRADE_THRESHOLD
        """
        cmd = "routerdowngradethreshold %s" % str(iThreshold)
        return self.__executeCommand(cmd)[-1] == "Done"

    def __setRouterSelectionJitter(self, iRouterJitter):
        """set ROUTER_SELECTION_JITTER parameter for REED to upgrade to Router

        Args:
            iRouterJitter: a random period prior to request Router ID for REED

        Returns:
            True: successful to set the ROUTER_SELECTION_JITTER
            False: fail to set ROUTER_SELECTION_JITTER
        """
        cmd = "routerselectionjitter %s" % str(iRouterJitter)
        return self.__executeCommand(cmd)[-1] == "Done"

    def __setAddressfilterMode(self, mode):
        """set address filter mode

        Returns:
            True: successful to set address filter mode.
            False: fail to set address filter mode.
        """
        cmd = "macfilter addr " + mode
        return self.__executeCommand(cmd)[-1] == "Done"

    def __startOpenThread(self):
        """start nRF_Connect_SDK_13_14 stack

        Returns:
            True: successful to start nRF_Connect_SDK_13_14 stack and thread interface up
            False: fail to start nRF_Connect_SDK_13_14 stack
        """
        if self.hasActiveDatasetToCommit:
            if self.__executeCommand("dataset commit active")[0] != "Done":
                raise Exception("failed to commit active dataset")
            else:
                self.hasActiveDatasetToCommit = False

        # Restore allowlist/denylist address filter mode if rejoin after
        # reset
        if self.isPowerDown:
            if self._addressfilterMode == "allowlist":
                if self.__setAddressfilterMode(self.__replaceCommands["allowlist"]):
                    for addr in self._addressfilterSet:
                        self.addAllowMAC(addr)
            elif self._addressfilterMode == "denylist":
                if self.__setAddressfilterMode(self.__replaceCommands["denylist"]):
                    for addr in self._addressfilterSet:
                        self.addBlockedMAC(addr)

        # Set routerselectionjitter to 1 for certain device roles
        if self.deviceRole in [
            Thread_Device_Role.Leader,
            Thread_Device_Role.Router,
            Thread_Device_Role.REED,
        ]:
            self.__setRouterSelectionJitter(1)
        elif self.deviceRole in [Thread_Device_Role.BR_1, Thread_Device_Role.BR_2]:
            if ModuleHelper.CurrentRunningTestMode == TestMode.Commercial:
                # Allow BBR configurations for 1.2 BR_1/BR_2 roles
                self.IsBeingTestedAsCommercialBBR = True
            self.__setRouterSelectionJitter(1)

        if self.IsBeingTestedAsCommercialBBR:
            # Configure default BBR dataset
            self.__configBbrDataset(
                SeqNum=self.bbrSeqNum,
                MlrTimeout=self.bbrMlrTimeout,
                ReRegDelay=self.bbrReRegDelay,
            )
            # Add default domain prefix is not configured otherwise
            if self.__useDefaultDomainPrefix:
                self.__addDefaultDomainPrefix()

        self.__executeCommand("ifconfig up")
        self.__executeCommand("thread start")
        self.isPowerDown = False
        return True

    @watched
    def __isOpenThreadRunning(self):
        """check whether or not nRF_Connect_SDK_13_14 is running

        Returns:
            True: nRF_Connect_SDK_13_14 is running
            False: nRF_Connect_SDK_13_14 is not running
        """
        return self.__executeCommand("state")[0] != "disabled"

    @watched
    def __isDeviceAttached(self):
        """check whether or not nRF_Connect_SDK_13_14 is running

        Returns:
            True: nRF_Connect_SDK_13_14 is running
            False: nRF_Connect_SDK_13_14 is not running
        """
        detached_states = ["detached", "disabled"]
        return self.__executeCommand("state")[0] not in detached_states

    # rloc16 might be hex string or integer, need to return actual allocated router id
    def __convertRlocToRouterId(self, xRloc16):
        """mapping Rloc16 to router id

        Args:
            xRloc16: hex rloc16 short address

        Returns:
            actual router id allocated by leader
        """
        routerList = self.__executeCommand("router list")[0].split()
        rloc16 = None
        routerid = None

        for index in routerList:
            cmd = "router %s" % index
            router = self.__executeCommand(cmd)

            for line in router:
                if "Done" in line:
                    break
                elif "Router ID" in line:
                    routerid = line.split()[2]
                elif "Rloc" in line:
                    rloc16 = line.split()[1]
                else:
                    pass

            # process input rloc16
            if isinstance(xRloc16, str):
                rloc16 = "0x" + rloc16
                if rloc16 == xRloc16:
                    return routerid
            elif isinstance(xRloc16, int):
                if int(rloc16, 16) == xRloc16:
                    return routerid
            else:
                pass

        return None

    # pylint: disable=no-self-use
    def __convertLongToHex(self, iValue, fillZeros=None):
        """convert a long hex integer to string
           remove '0x' and 'L' return string

        Args:
            iValue: long integer in hex format
            fillZeros: pad string with zeros on the left to specified width

        Returns:
            string of this long integer without '0x' and 'L'
        """
        fmt = "%x"
        if fillZeros is not None:
            fmt = "%%0%dx" % fillZeros

        return fmt % iValue

    @commissioning
    def __readCommissioningLogs(self, durationInSeconds):
        """read logs during the commissioning process

        Args:
            durationInSeconds: time duration for reading commissioning logs

        Returns:
            Commissioning logs
        """
        self.logThreadStatus = self.logStatus["running"]
        logs = Queue()
        t_end = time.time() + durationInSeconds
        joinSucceed = False

        while time.time() < t_end:
            if self.logThreadStatus == self.logStatus["pauseReq"]:
                self.logThreadStatus = self.logStatus["paused"]

            if self.logThreadStatus != self.logStatus["running"]:
                self.sleep(0.01)
                continue

            try:
                line = self.__readCliLine(ignoreLogs=False)

                if line:
                    self.log("commissioning log: %s", line)
                    logs.put(line)

                    if "Join success" in line:
                        joinSucceed = True
                        # read commissioning logs for 3 more seconds
                        t_end = time.time() + 3
                    elif "Join failed" in line:
                        # read commissioning logs for 3 more seconds
                        t_end = time.time() + 3
                elif line is None:
                    self.sleep(0.01)

            except Exception:
                pass

        self.joinCommissionedStatus = (
            self.joinStatus["succeed"] if joinSucceed else self.joinStatus["failed"]
        )
        self.logThreadStatus = self.logStatus["stop"]
        return logs

    # pylint: disable=no-self-use
    def __convertChannelMask(self, channelsArray):
        """convert channelsArray to bitmask format

        Args:
            channelsArray: channel array (i.e. [21, 22])

        Returns:
            bitmask format corresponding to a given channel array
        """
        maskSet = 0

        for eachChannel in channelsArray:
            mask = 1 << eachChannel
            maskSet = maskSet | mask

        return maskSet

    def __setChannelMask(self, channelMask):
        cmd = "dataset channelmask %s" % channelMask
        self.hasActiveDatasetToCommit = True
        return self.__executeCommand(cmd)[-1] == "Done"

    def __setSecurityPolicy(self, securityPolicySecs, securityPolicyFlags):
        cmd = "dataset securitypolicy %s %s" % (
            str(securityPolicySecs),
            securityPolicyFlags,
        )
        self.hasActiveDatasetToCommit = True
        return self.__executeCommand(cmd)[-1] == "Done"

    def __setKeySwitchGuardTime(self, iKeySwitchGuardTime):
        """set the Key switch guard time

        Args:
            iKeySwitchGuardTime: key switch guard time

        Returns:
            True: successful to set key switch guard time
            False: fail to set key switch guard time
        """
        cmd = "keysequence guardtime %s" % str(iKeySwitchGuardTime)
        if self.__executeCommand(cmd)[-1] == "Done":
            self.sleep(1)
            return True
        else:
            return False

    def __getCommissionerSessionId(self):
        """get the commissioner session id allocated from Leader"""
        return self.__executeCommand("commissioner sessionid")[0]

    # pylint: disable=no-self-use
    def _deviceEscapeEscapable(self, string):
        """Escape CLI escapable characters in the given string.

        Args:
            string (str): UTF-8 input string.

        Returns:
            [str]: The modified string with escaped characters.
        """
        escapable_chars = "\\ \t\r\n"
        for char in escapable_chars:
            string = string.replace(char, "\\%s" % char)
        return string

    @API
    def setNetworkName(self, networkName="GRL"):
        """set Thread Network name

        Args:
            networkName: the networkname string to be set

        Returns:
            True: successful to set the Thread Networkname
            False: fail to set the Thread Networkname
        """
        networkName = self._deviceEscapeEscapable(networkName)
        cmd = "networkname %s" % networkName
        datasetCmd = "dataset networkname %s" % networkName
        self.hasActiveDatasetToCommit = True
        return (
            self.__executeCommand(cmd)[-1] == "Done"
            and self.__executeCommand(datasetCmd)[-1] == "Done"
        )

    @API
    def setChannel(self, channel=11):
        """set channel of Thread device operates on.

        Args:
            channel:
                    (0  - 10: Reserved)
                    (11 - 26: 2.4GHz channels)
                    (27 - 65535: Reserved)

        Returns:
            True: successful to set the channel
            False: fail to set the channel
        """
        cmd = "channel %s" % channel
        datasetCmd = "dataset channel %s" % channel
        self.hasSetChannel = True
        self.hasActiveDatasetToCommit = True
        return (
            self.__executeCommand(cmd)[-1] == "Done"
            and self.__executeCommand(datasetCmd)[-1] == "Done"
        )

    @API
    def getChannel(self):
        """get current channel"""
        return self.__executeCommand("channel")[0]

    @API
    def setMAC(self, xEUI):
        """set the extended addresss of Thread device

        Args:
            xEUI: extended address in hex format

        Returns:
            True: successful to set the extended address
            False: fail to set the extended address
        """
        if not isinstance(xEUI, str):
            address64 = self.__convertLongToHex(xEUI, 16)
        else:
            address64 = xEUI

        cmd = "extaddr %s" % address64
        if self.__executeCommand(cmd)[-1] == "Done":
            self.mac = address64
            return True
        else:
            return False

    @API
    def getMAC(self, bType=MacType.RandomMac):
        """get one specific type of MAC address
           currently nRF_Connect_SDK_13_14 only supports Random MAC address

        Args:
            bType: indicate which kind of MAC address is required

        Returns:
            specific type of MAC address
        """
        # if power down happens, return extended address assigned previously
        if self.isPowerDown:
            macAddr64 = self.mac
        else:
            if bType == MacType.FactoryMac:
                macAddr64 = self.__executeCommand("eui64")[0]
            elif bType == MacType.HashMac:
                macAddr64 = self.__executeCommand("joiner id")[0]
            elif bType == MacType.EthMac and self.IsBorderRouter:
                return self._deviceGetEtherMac()
            else:
                macAddr64 = self.__executeCommand("extaddr")[0]

        return int(macAddr64, 16)

    @API
    def getLL64(self):
        """get link local unicast IPv6 address"""
        return self.__executeCommand("ipaddr linklocal")[0]

    @API
    def getRloc16(self):
        """get rloc16 short address"""
        rloc16 = self.__executeCommand("rloc16")[0]
        return int(rloc16, 16)

    @API
    def getRloc(self):
        """get router locator unicast Ipv6 address"""
        return self.__executeCommand("ipaddr rloc")[0]

    def __getGlobal(self):
        """get global unicast IPv6 address set
        if configuring multiple entries
        """
        globalAddrs = []
        rlocAddr = self.getRloc()

        addrs = self.__executeCommand("ipaddr")

        # take rloc address as a reference for current mesh local prefix,
        # because for some TCs, mesh local prefix may be updated through
        # pending dataset management.
        for ip6Addr in addrs:
            if ip6Addr == "Done":
                break

            fullIp = ModuleHelper.GetFullIpv6Address(ip6Addr).lower()

            if fullIp.startswith("fe80") or fullIp.startswith(rlocAddr[0:19]):
                continue

            globalAddrs.append(fullIp)

        return globalAddrs

    @API
    def setNetworkKey(self, key):
        """set Thread network key

        Args:
            key: Thread network key used in secure the MLE/802.15.4 packet

        Returns:
            True: successful to set the Thread network key
            False: fail to set the Thread network key
        """
        cmdName = self.__replaceCommands["networkkey"]

        if not isinstance(key, str):
            networkKey = self.__convertLongToHex(key, 32)
            cmd = "%s %s" % (cmdName, networkKey)
            datasetCmd = "dataset %s %s" % (cmdName, networkKey)
        else:
            networkKey = key
            cmd = "%s %s" % (cmdName, networkKey)
            datasetCmd = "dataset %s %s" % (cmdName, networkKey)

        self.networkKey = networkKey
        self.hasActiveDatasetToCommit = True
        return (
            self.__executeCommand(cmd)[-1] == "Done"
            and self.__executeCommand(datasetCmd)[-1] == "Done"
        )

    def getNetworkKey(self):
        cmd = "dataset networkkey"
        result = self.__executeCommand(cmd)
        return result

    @API
    def addBlockedMAC(self, xEUI):
        """add a given extended address to the denylist entry

        Args:
            xEUI: extended address in hex format

        Returns:
            True: successful to add a given extended address to the denylist entry
            False: fail to add a given extended address to the denylist entry
        """
        if isinstance(xEUI, str):
            macAddr = xEUI
        else:
            macAddr = self.__convertLongToHex(xEUI)

        # if blocked device is itself
        if macAddr == self.mac:
            print("block device itself")
            return True

        if self._addressfilterMode != "denylist":
            if self.__setAddressfilterMode(self.__replaceCommands["denylist"]):
                self._addressfilterMode = "denylist"

        cmd = "macfilter addr add %s" % macAddr
        ret = self.__executeCommand(cmd)[-1] == "Done"

        self._addressfilterSet.add(macAddr)
        print("current denylist entries:")
        for addr in self._addressfilterSet:
            print(addr)

        return ret

    @API
    def addAllowMAC(self, xEUI):
        """add a given extended address to the allowlist addressfilter

        Args:
            xEUI: a given extended address in hex format

        Returns:
            True: successful to add a given extended address to the allowlist entry
            False: fail to add a given extended address to the allowlist entry
        """
        if isinstance(xEUI, str):
            macAddr = xEUI
        else:
            macAddr = self.__convertLongToHex(xEUI)

        if self._addressfilterMode != "allowlist":
            if self.__setAddressfilterMode(self.__replaceCommands["allowlist"]):
                self._addressfilterMode = "allowlist"

        cmd = "macfilter addr add %s" % macAddr
        ret = self.__executeCommand(cmd)[-1] == "Done"

        self._addressfilterSet.add(macAddr)
        print("current allowlist entries:")
        for addr in self._addressfilterSet:
            print(addr)
        return ret

    @API
    def clearBlockList(self):
        """clear all entries in denylist table

        Returns:
            True: successful to clear the denylist
            False: fail to clear the denylist
        """
        # remove all entries in denylist
        print("clearing denylist entries:")
        for addr in self._addressfilterSet:
            print(addr)

        # disable denylist
        if self.__setAddressfilterMode("disable"):
            self._addressfilterMode = "disable"
            # clear ops
            cmd = "macfilter addr clear"
            if self.__executeCommand(cmd)[-1] == "Done":
                self._addressfilterSet.clear()
                return True
        return False

    @API
    def clearAllowList(self):
        """clear all entries in allowlist table

        Returns:
            True: successful to clear the allowlist
            False: fail to clear the allowlist
        """
        # remove all entries in allowlist
        print("clearing allowlist entries:")
        for addr in self._addressfilterSet:
            print(addr)

        # disable allowlist
        if self.__setAddressfilterMode("disable"):
            self._addressfilterMode = "disable"
            # clear ops
            cmd = "macfilter addr clear"
            if self.__executeCommand(cmd)[-1] == "Done":
                self._addressfilterSet.clear()
                return True
        return False

    @API
    def getDeviceRole(self):
        """get current device role in Thread Network"""
        return self.__executeCommand("state")[0]

    @API
    def joinNetwork(self, eRoleId):
        """make device ready to join the Thread Network with a given role

        Args:
            eRoleId: a given device role id

        Returns:
            True: ready to set Thread Network parameter for joining desired Network
        """
        self.deviceRole = eRoleId
        mode = "-"
        if ModuleHelper.LeaderDutChannelFound and not self.hasSetChannel:
            self.channel = ModuleHelper.Default_Channel

        # FIXME: when Harness call setNetworkDataRequirement()?
        # only sleep end device requires stable networkdata now
        if eRoleId == Thread_Device_Role.Leader:
            print("join as leader")
            mode = "rdn"
            if self.AutoDUTEnable is False:
                # set ROUTER_DOWNGRADE_THRESHOLD
                self.__setRouterDowngradeThreshold(33)
        elif eRoleId == Thread_Device_Role.Router:
            print("join as router")
            mode = "rdn"
            if self.AutoDUTEnable is False:
                # set ROUTER_DOWNGRADE_THRESHOLD
                self.__setRouterDowngradeThreshold(33)
        elif eRoleId in (Thread_Device_Role.BR_1, Thread_Device_Role.BR_2):
            print("join as BBR")
            mode = "rdn"
            if self.AutoDUTEnable is False:
                # set ROUTER_DOWNGRADE_THRESHOLD
                self.__setRouterDowngradeThreshold(33)
        elif eRoleId == Thread_Device_Role.SED:
            print("join as sleepy end device")
            mode = "-"
            self.__setPollPeriod(self.__sedPollPeriod)
        elif eRoleId == Thread_Device_Role.SSED:
            print("join as SSED")
            mode = "-"
            self.setCSLperiod(self.cslPeriod)
            self.setCSLtout(self.ssedTimeout)
            self.setCSLsuspension(False)
        elif eRoleId == Thread_Device_Role.EndDevice:
            print("join as end device")
            mode = "rn"
        elif eRoleId == Thread_Device_Role.REED:
            print("join as REED")
            mode = "rdn"
            if self.AutoDUTEnable is False:
                # set ROUTER_UPGRADE_THRESHOLD
                self.__setRouterUpgradeThreshold(0)
        elif eRoleId == Thread_Device_Role.EndDevice_FED:
            print("join as FED")
            mode = "rdn"
            # always remain an ED, never request to be a router
            self.__disableRouterEligible()
        elif eRoleId == Thread_Device_Role.EndDevice_MED:
            print("join as MED")
            mode = "rn"
        else:
            pass

        if self.IsReference20200818:
            mode = "s" if mode == "-" else mode + "s"

        # set Thread device mode with a given role
        self.__setDeviceMode(mode)

        # start nRF_Connect_SDK_13_14
        self.__startOpenThread()
        self.wait_for_attach_to_the_network(
            expected_role=eRoleId,
            timeout=self.NETWORK_ATTACHMENT_TIMEOUT,
            raise_assert=True,
        )
        return True

    def wait_for_attach_to_the_network(
        self, expected_role, timeout, raise_assert=False
    ):
        start_time = time.time()

        while time.time() < start_time + timeout:
            time.sleep(0.3)
            if self.__isDeviceAttached():
                break
        else:
            if raise_assert:
                raise AssertionError(
                    "OT device {} could not attach to the network after {} s of timeout.".format(
                        self, timeout
                    )
                )
            else:
                return False

        if self._update_router_status:
            self.__updateRouterStatus()

        if expected_role == Thread_Device_Role.Router:
            while time.time() < start_time + timeout:
                time.sleep(0.3)
                if self.getDeviceRole() == "router":
                    break
            else:
                if raise_assert:
                    raise AssertionError(
                        "OT Router {} could not attach to the network after {} s of timeout.".format(
                            self, timeout * 2
                        )
                    )
                else:
                    return False

        # if self.IsBorderRouter:
        #     self._waitBorderRoutingStabilize()

        return True

    @API
    def getNetworkFragmentID(self):
        """get current partition id of Thread Network Partition from LeaderData

        Returns:
            The Thread network Partition Id
        """
        if not self.__isOpenThreadRunning():
            return None

        leaderData = self.__executeCommand("leaderdata")
        return int(leaderData[0].split()[2], 16)

    @API
    def getParentAddress(self):
        """get Thread device's parent extended address and rloc16 short address

        Returns:
            The extended address of parent in hex format
        """
        eui = None
        parentInfo = self.__executeCommand("parent")

        for line in parentInfo:
            if "Done" in line:
                break
            elif "Ext Addr" in line:
                eui = line.split()[2]
            else:
                pass

        return int(eui, 16)

    @API
    def powerDown(self):
        """power down the Thread device"""
        self._reset()

    @API
    def powerUp(self):
        """power up the Thread device"""
        self.isPowerDown = False

        if not self.__isOpenThreadRunning():
            if self.deviceRole == Thread_Device_Role.SED:
                self.__setPollPeriod(self.__sedPollPeriod)
            self.__startOpenThread()

    @watched
    def _reset(self, timeout=3):
        print(("Waiting after reset timeout: {} s".format(timeout)))
        start_time = time.time()
        self.__sendCommand("reset", expectEcho=False)
        self.isPowerDown = True

        while time.time() < start_time + timeout:
            time.sleep(0.3)
            if not self.IsBorderRouter:
                self._disconnect()
                self._connect()
            try:
                self.__executeCommand("state", timeout=0.1)
                break
            except Exception:
                continue
        else:
            raise AssertionError(
                "Could not connect with OT device {} after reset.".format(self)
            )

    def reset_and_wait_for_connection(self, timeout=3):
        self._reset(timeout=timeout)
        if self.deviceRole == Thread_Device_Role.SED:
            self.__setPollPeriod(self.__sedPollPeriod)

    @API
    def reboot(self):
        """reset and rejoin to Thread Network without any timeout

        Returns:
            True: successful to reset and rejoin the Thread Network
            False: fail to reset and rejoin the Thread Network
        """
        self.reset_and_wait_for_connection()
        self.__startOpenThread()
        return self.wait_for_attach_to_the_network(
            expected_role="", timeout=self.NETWORK_ATTACHMENT_TIMEOUT
        )

    @API
    def resetAndRejoin(self, timeout):
        """reset and join back Thread Network with a given timeout delay

        Args:
            timeout: a timeout interval before rejoin Thread Network

        Returns:
            True: successful to reset and rejoin Thread Network
            False: fail to reset and rejoin the Thread Network
        """
        self.powerDown()
        time.sleep(timeout)
        self.powerUp()
        return self.wait_for_attach_to_the_network(
            expected_role="", timeout=self.NETWORK_ATTACHMENT_TIMEOUT
        )

    @API
    def ping(
        self, strDestination, ilength=0, hop_limit=64, timeout=5, src_addr_type=""
    ):
        """send ICMPv6 echo request with a given length/hoplimit to a unicast
            destination address
        Args:
            srcDestination: the unicast destination address of ICMPv6 echo request
            ilength: the size of ICMPv6 echo request payload
            hop_limit: hop limit

        """
        print(("%s call ping" % self.port))
        print(("destination: %s" % strDestination))
        print(("source: %s" % src_addr_type))
        if src_addr_type != "":
            cmd = "ping -I %s %s %s 1 1 %d %d" % (
                src_addr_type,
                strDestination,
                str(ilength),
                hop_limit,
                timeout,
            )
        else:
            cmd = "ping %s %s 1 1 %d %d" % (
                strDestination,
                str(ilength),
                hop_limit,
                timeout,
            )
        self.__executeCommand(cmd)
        time.sleep(6)  # increase delay temporarily (+5s) to remedy TH's delay updates

    @API
    def multicast_Ping(self, destination, length=20):
        """send ICMPv6 echo request with a given length to a multicast destination
           address

        Args:
            destination: the multicast destination address of ICMPv6 echo request
            length: the size of ICMPv6 echo request payload
        """
        cmd = "ping %s %s" % (destination, str(length))
        self.__sendCommand(cmd)
        # wait echo reply
        self.sleep(1)

    @API
    def setPANID(self, xPAN):
        """set Thread Network PAN ID

        Args:
            xPAN: a given PAN ID in hex format

        Returns:
            True: successful to set the Thread Network PAN ID
            False: fail to set the Thread Network PAN ID
        """
        panid = ""
        if not isinstance(xPAN, str):
            panid = str(hex(xPAN))

        cmd = "panid %s" % panid
        datasetCmd = "dataset panid %s" % panid
        self.hasActiveDatasetToCommit = True
        return (
            self.__executeCommand(cmd)[-1] == "Done"
            and self.__executeCommand(datasetCmd)[-1] == "Done"
        )

    @API
    def reset(self):
        """factory reset"""
        self._deviceBeforeReset()

        self.__sendCommand("factoryreset", expectEcho=False)
        timeout = 10

        start_time = time.time()
        while time.time() < start_time + timeout:
            time.sleep(0.5)
            if not self.IsBorderRouter:
                self._disconnect()
                self._connect()
            try:
                self.__executeCommand("state", timeout=0.1)
                break
            except Exception:
                self._restartAgentService()
                time.sleep(2)
                self.__sendCommand("factoryreset", expectEcho=False)
                time.sleep(0.5)
                continue
        else:
            raise AssertionError(
                "Could not connect with OT device {} after reset.".format(self)
            )

        self.log("factoryreset finished within 10s timeout.")
        self._deviceAfterReset()

        if self.IsBorderRouter:
            self.__executeCommand("log level 5")

    @API
    def removeRouter(self, xRouterId):
        """kickoff router with a given router id from the Thread Network

        Args:
            xRouterId: a given router id in hex format

        Returns:
            True: successful to remove the router from the Thread Network
            False: fail to remove the router from the Thread Network
        """
        routerId = self.__convertRlocToRouterId(xRouterId)

        if routerId is None:
            return False

        cmd = "releaserouterid %s" % routerId
        return self.__executeCommand(cmd)[-1] == "Done"

    @API
    def setDefaultValues(self):
        """set default mandatory Thread Network parameter value"""
        # initialize variables
        self.networkName = ModuleHelper.Default_NwkName
        self.networkKey = ModuleHelper.Default_NwkKey
        self.channel = ModuleHelper.Default_Channel
        self.channelMask = "0x7fff800"  # (0xffff << 11)
        self.panId = ModuleHelper.Default_PanId
        self.xpanId = ModuleHelper.Default_XpanId
        self.meshLocalPrefix = ModuleHelper.Default_MLPrefix
        stretchedPSKc = Thread_PBKDF2.get(
            ModuleHelper.Default_PSKc,
            ModuleHelper.Default_XpanId,
            ModuleHelper.Default_NwkName,
        )
        self.pskc = hex(stretchedPSKc).rstrip("L").lstrip("0x")
        self.securityPolicySecs = ModuleHelper.Default_SecurityPolicy
        self.securityPolicyFlags = "onrc"
        self.activetimestamp = ModuleHelper.Default_ActiveTimestamp
        # self.sedPollingRate = ModuleHelper.Default_Harness_SED_Polling_Rate
        self.__sedPollPeriod = 3 * 1000  # in milliseconds
        self.ssedTimeout = 30  # in seconds
        self.cslPeriod = 500  # in milliseconds
        self.deviceRole = None
        self.provisioningUrl = ""
        self.hasActiveDatasetToCommit = False
        self.logThread = Queue()
        self.logThreadStatus = self.logStatus["stop"]
        self.joinCommissionedStatus = self.joinStatus["notstart"]
        # indicate Thread device requests full or stable network data
        self.networkDataRequirement = ""
        # indicate if Thread device experiences a power down event
        self.isPowerDown = False
        # indicate AddressFilter mode ['disable', 'allowlist', 'denylist']
        self._addressfilterMode = "disable"
        self._addressfilterSet = set()  # cache filter entries
        # indicate if Thread device is an active commissioner
        self.isActiveCommissioner = False
        # indicate that the channel has been set, in case the channel was set
        # to default when joining network
        self.hasSetChannel = False
        self.IsBeingTestedAsCommercialBBR = False
        # indicate whether the default domain prefix is used.
        self.__useDefaultDomainPrefix = True
        self.__isUdpOpened = False
        self.IsHost = False

        # remove stale multicast addresses
        if self.IsBorderRouter:
            self.stopListeningToAddrAll()

        # BBR dataset
        self.bbrSeqNum = random.randint(0, 126)  # 5.21.4.2
        self.bbrMlrTimeout = 3600
        self.bbrReRegDelay = 5

        # initialize device configuration
        self.setMAC(self.mac)
        self.__setChannelMask(self.channelMask)
        self.__setSecurityPolicy(self.securityPolicySecs, self.securityPolicyFlags)
        self.setChannel(self.channel)
        self.setPANID(self.panId)
        self.setXpanId(self.xpanId)
        self.setNetworkName(self.networkName)
        self.setNetworkKey(self.networkKey)
        self.setMLPrefix(self.meshLocalPrefix)
        self.setPSKc(self.pskc)
        self.setActiveTimestamp(self.activetimestamp)

    @API
    def getDeviceConncetionStatus(self):
        """check if serial port connection is ready or not"""
        return self.deviceConnected

    @API
    def setPollingRate(self, iPollingRate):
        """set data polling rate for sleepy end device

        Args:
            iPollingRate: data poll period of sleepy end device (in seconds)

        Returns:
            True: successful to set the data polling rate for sleepy end device
            False: fail to set the data polling rate for sleepy end device
        """
        iPollingRate = int(iPollingRate * 1000)

        if self.__sedPollPeriod != iPollingRate:
            if not iPollingRate:
                iPollingRate = 0xFFFF  # T5.2.1, disable polling
            elif iPollingRate < 1:
                iPollingRate = 1  # T9.2.13
            self.__sedPollPeriod = iPollingRate

            # apply immediately
            if self.__isOpenThreadRunning():
                return self.__setPollPeriod(self.__sedPollPeriod)

        return True

    def __setPollPeriod(self, iPollPeriod):
        """set data poll period for sleepy end device

        Args:
            iPollPeriod: data poll period of sleepy end device (in milliseconds)

        Returns:
            True: successful to set the data poll period for sleepy end device
            False: fail to set the data poll period for sleepy end device
        """
        cmd = "pollperiod %d" % iPollPeriod
        return self.__executeCommand(cmd)[-1] == "Done"

    @API
    def setLinkQuality(self, EUIadr, LinkQuality):
        """set custom LinkQualityIn for all receiving messages from the specified EUIadr

        Args:
            EUIadr: a given extended address
            LinkQuality: a given custom link quality
                         link quality/link margin mapping table
                         3: 21 - 255 (dB)
                         2: 11 - 20 (dB)
                         1: 3 - 9 (dB)
                         0: 0 - 2 (dB)

        Returns:
            True: successful to set the link quality
            False: fail to set the link quality
        """
        # process EUIadr
        euiHex = hex(EUIadr)
        euiStr = str(euiHex)
        euiStr = euiStr.rstrip("L")
        address64 = ""
        if "0x" in euiStr:
            address64 = self.__lstrip0x(euiStr)
            # prepend 0 at the beginning
            if len(address64) < 16:
                address64 = address64.zfill(16)
                print(address64)

        cmd = "macfilter rss add-lqi %s %s" % (address64, str(LinkQuality))
        return self.__executeCommand(cmd)[-1] == "Done"

    @API
    def setOutBoundLinkQuality(self, LinkQuality):
        """set custom LinkQualityIn for all receiving messages from the any address

        Args:
            LinkQuality: a given custom link quality
                         link quality/link margin mapping table
                         3: 21 - 255 (dB)
                         2: 11 - 20 (dB)
                         1: 3 - 9 (dB)
                         0: 0 - 2 (dB)

        Returns:
            True: successful to set the link quality
            False: fail to set the link quality
        """
        cmd = "macfilter rss add-lqi * %s" % str(LinkQuality)
        return self.__executeCommand(cmd)[-1] == "Done"

    @API
    def removeRouterPrefix(self, prefixEntry):
        """remove the configured prefix on a border router

        Args:
            prefixEntry: a on-mesh prefix entry in IPv6 dotted-quad format

        Returns:
            True: successful to remove the prefix entry from border router
            False: fail to remove the prefix entry from border router
        """
        assert ipaddress.IPv6Network(prefixEntry.decode())
        cmd = "prefix remove %s/64" % prefixEntry
        if self.__executeCommand(cmd)[-1] == "Done":
            # send server data ntf to leader
            cmd = self.__replaceCommands["netdata register"]
            return self.__executeCommand(cmd)[-1] == "Done"
        else:
            return False

    @API
    def configBorderRouter(
        self,
        P_Prefix="fd00:7d03:7d03:7d03::",
        P_stable=1,
        P_default=1,
        P_slaac_preferred=0,
        P_Dhcp=0,
        P_preference=0,
        P_on_mesh=1,
        P_nd_dns=0,
        P_dp=0,
    ):
        """configure the border router with a given prefix entry parameters

        Args:
            P_Prefix: IPv6 prefix that is available on the Thread Network in IPv6 dotted-quad format
            P_stable: true if the default router is expected to be stable network data
            P_default: true if border router offers the default route for P_Prefix
            P_slaac_preferred: true if allowing auto-configure address using P_Prefix
            P_Dhcp: is true if border router is a DHCPv6 Agent
            P_preference: is two-bit signed integer indicating router preference
            P_on_mesh: is true if P_Prefix is considered to be on-mesh
            P_nd_dns: is true if border router is able to supply DNS information obtained via ND

        Returns:
            True: successful to configure the border router with a given prefix entry
            False: fail to configure the border router with a given prefix entry
        """
        if isinstance(P_Prefix, str):
            assert ipaddress.IPv6Network(P_Prefix)
        else:
            assert ipaddress.IPv6Network(P_Prefix.decode())

        # turn off default domain prefix if configBorderRouter is called before joining network
        if P_dp == 0 and not self.__isOpenThreadRunning():
            self.__useDefaultDomainPrefix = False

        parameter = ""
        prf = ""

        if P_dp:
            P_slaac_preferred = 1

        if P_slaac_preferred == 1:
            parameter += "p"
            parameter += "a"

        if P_stable == 1:
            parameter += "s"

        if P_default == 1:
            parameter += "r"

        if P_Dhcp == 1:
            parameter += "d"

        if P_on_mesh == 1:
            parameter += "o"

        if P_dp == 1:
            assert P_slaac_preferred and P_default and P_on_mesh and P_stable
            parameter += "D"

        if P_preference == 1:
            prf = "high"
        elif P_preference == 0:
            prf = "med"
        elif P_preference == -1:
            prf = "low"
        else:
            pass

        cmd = "prefix add %s/64 %s %s" % (P_Prefix, parameter, prf)
        if self.__executeCommand(cmd)[-1] == "Done":
            # if prefix configured before starting nRF_Connect_SDK_13_14 stack
            # do not send out server data ntf pro-actively
            if not self.__isOpenThreadRunning():
                return True
            else:
                # send server data ntf to leader
                cmd = self.__replaceCommands["netdata register"]
                return self.__executeCommand(cmd)[-1] == "Done"
        else:
            return False

    @API
    def disable_dua_prefix(self):
        self.__useDefaultDomainPrefix = False

    @watched
    def getNetworkData(self):
        lines = self.__executeCommand("netdata show")
        prefixes, routes, services = [], [], []
        classify = None

        for line in lines:
            if line == "Prefixes:":
                classify = prefixes
            elif line == "Routes:":
                classify = routes
            elif line == "Services:":
                classify = services
            elif line == "Done":
                classify = None
            else:
                classify.append(line)

        return {
            "Prefixes": prefixes,
            "Routes": routes,
            "Services": services,
        }

    @API
    def setNetworkIDTimeout(self, iNwkIDTimeOut):
        """set networkid timeout for Thread device

        Args:
            iNwkIDTimeOut: a given NETWORK_ID_TIMEOUT

        Returns:
            True: successful to set NETWORK_ID_TIMEOUT
            False: fail to set NETWORK_ID_TIMEOUT
        """
        iNwkIDTimeOut /= 1000
        cmd = "networkidtimeout %s" % str(iNwkIDTimeOut)
        return self.__executeCommand(cmd)[-1] == "Done"

    @API
    def setKeepAliveTimeOut(self, iTimeOut):
        """set child timeout for device

        Args:
            iTimeOut: child timeout for device

        Returns:
            True: successful to set the child timeout for device
            False: fail to set the child timeout for device
        """
        cmd = "childtimeout %d" % iTimeOut
        return self.__executeCommand(cmd)[-1] == "Done"

    @API
    def setKeySequenceCounter(self, iKeySequenceValue):
        """set the Key sequence counter corresponding to Thread network key

        Args:
            iKeySequenceValue: key sequence value

        Returns:
            True: successful to set the key sequence
            False: fail to set the key sequence
        """
        # avoid key switch guard timer protection for reference device
        self.__setKeySwitchGuardTime(0)

        cmd = "keysequence counter %s" % str(iKeySequenceValue)
        if self.__executeCommand(cmd)[-1] == "Done":
            self.sleep(1)
            return True
        else:
            return False

    @API
    def getKeySequenceCounter(self):
        """get current Thread Network key sequence"""
        keySequence = self.__executeCommand("keysequence counter")[0]
        return keySequence

    @API
    def incrementKeySequenceCounter(self, iIncrementValue=1):
        """increment the key sequence with a given value

        Args:
            iIncrementValue: specific increment value to be added

        Returns:
            True: successful to increment the key sequence with a given value
            False: fail to increment the key sequence with a given value
        """
        # avoid key switch guard timer protection for reference device
        self.__setKeySwitchGuardTime(0)
        currentKeySeq = self.getKeySequenceCounter()
        keySequence = int(currentKeySeq, 10) + iIncrementValue
        return self.setKeySequenceCounter(keySequence)

    @API
    def setNetworkDataRequirement(self, eDataRequirement):
        """set whether the Thread device requires the full network data
           or only requires the stable network data

        Args:
            eDataRequirement: is true if requiring the full network data

        Returns:
            True: successful to set the network requirement
        """
        if eDataRequirement == Device_Data_Requirement.ALL_DATA:
            self.networkDataRequirement = "n"
        return True

    @API
    def configExternalRouter(self, P_Prefix, P_stable, R_Preference=0):
        """configure border router with a given external route prefix entry

        Args:
            P_Prefix: IPv6 prefix for the route in IPv6 dotted-quad format
            P_Stable: is true if the external route prefix is stable network data
            R_Preference: a two-bit signed integer indicating Router preference
                          1: high
                          0: medium
                         -1: low

        Returns:
            True: successful to configure the border router with a given external route prefix
            False: fail to configure the border router with a given external route prefix
        """
        assert ipaddress.IPv6Network(P_Prefix.decode())
        prf = ""
        stable = ""
        if R_Preference == 1:
            prf = "high"
        elif R_Preference == 0:
            prf = "med"
        elif R_Preference == -1:
            prf = "low"
        else:
            pass

        if P_stable:
            stable += "s"
            cmd = "route add %s/64 %s %s" % (P_Prefix, stable, prf)
        else:
            cmd = "route add %s/64 %s" % (P_Prefix, prf)

        if self.__executeCommand(cmd)[-1] == "Done":
            # send server data ntf to leader
            cmd = self.__replaceCommands["netdata register"]
            return self.__executeCommand(cmd)[-1] == "Done"

    @API
    def getNeighbouringRouters(self):
        """get neighboring routers information

        Returns:
            neighboring routers' extended address
        """
        routerInfo = []
        routerList = self.__executeCommand("router list")[0].split()

        if "Done" in routerList:
            return None

        for index in routerList:
            router = []
            cmd = "router %s" % index
            router = self.__executeCommand(cmd)

            for line in router:
                if "Done" in line:
                    break
                # elif 'Rloc' in line:
                #    rloc16 = line.split()[1]
                elif "Ext Addr" in line:
                    eui = line.split()[2]
                    routerInfo.append(int(eui, 16))
                # elif 'LQI In' in line:
                #    lqi_in = line.split()[1]
                # elif 'LQI Out' in line:
                #    lqi_out = line.split()[1]
                else:
                    pass

        return routerInfo

    @API
    def getChildrenInfo(self):
        """get all children information

        Returns:
            children's extended address
        """
        eui = None
        rloc16 = None
        childrenInfoAll = []
        childrenInfo = {"EUI": 0, "Rloc16": 0, "MLEID": ""}
        childrenList = self.__executeCommand("child list")[0].split()

        if "Done" in childrenList:
            return None

        for index in childrenList:
            cmd = "child %s" % index
            child = []
            child = self.__executeCommand(cmd)

            for line in child:
                if "Done" in line:
                    break
                elif "Rloc" in line:
                    rloc16 = line.split()[1]
                elif "Ext Addr" in line:
                    eui = line.split()[2]
                # elif 'Child ID' in line:
                #    child_id = line.split()[2]
                # elif 'Mode' in line:
                #    mode = line.split()[1]
                else:
                    pass

            childrenInfo["EUI"] = int(eui, 16)
            childrenInfo["Rloc16"] = int(rloc16, 16)
            # children_info['MLEID'] = self.getMLEID()

            childrenInfoAll.append(childrenInfo["EUI"])
            # childrenInfoAll.append(childrenInfo)

        return childrenInfoAll

    @API
    def setXpanId(self, xPanId):
        """set extended PAN ID of Thread Network

        Args:
            xPanId: extended PAN ID in hex format

        Returns:
            True: successful to set the extended PAN ID
            False: fail to set the extended PAN ID
        """
        xpanid = ""
        if not isinstance(xPanId, str):
            xpanid = self.__convertLongToHex(xPanId, 16)
            cmd = "extpanid %s" % xpanid
            datasetCmd = "dataset extpanid %s" % xpanid
        else:
            xpanid = xPanId
            cmd = "extpanid %s" % xpanid
            datasetCmd = "dataset extpanid %s" % xpanid

        self.xpanId = xpanid
        self.hasActiveDatasetToCommit = True
        return (
            self.__executeCommand(cmd)[-1] == "Done"
            and self.__executeCommand(datasetCmd)[-1] == "Done"
        )

    @API
    def getNeighbouringDevices(self):
        """gets the neighboring devices' extended address to compute the DUT
           extended address automatically

        Returns:
            A list including extended address of neighboring routers, parent
            as well as children
        """
        neighbourList = []

        # get parent info
        parentAddr = self.getParentAddress()
        if parentAddr != 0:
            neighbourList.append(parentAddr)

        # get ED/SED children info
        childNeighbours = self.getChildrenInfo()
        if childNeighbours is not None and len(childNeighbours) > 0:
            for entry in childNeighbours:
                neighbourList.append(entry)

        # get neighboring routers info
        routerNeighbours = self.getNeighbouringRouters()
        if routerNeighbours is not None and len(routerNeighbours) > 0:
            for entry in routerNeighbours:
                neighbourList.append(entry)

        return neighbourList

    @API
    def setPartationId(self, partationId):
        """set Thread Network Partition ID

        Args:
            partitionId: partition id to be set by leader

        Returns:
            True: successful to set the Partition ID
            False: fail to set the Partition ID
        """
        cmd = self.__replaceCommands["partitionid preferred"] + " "
        cmd += str(hex(partationId)).rstrip("L")
        return self.__executeCommand(cmd)[-1] == "Done"

    @API
    def getGUA(self, filterByPrefix=None, eth=False):
        """get expected global unicast IPv6 address of Thread device

        note: existing filterByPrefix are string of in lowercase. e.g.
        '2001' or '2001:0db8:0001:0000".

        Args:
            filterByPrefix: a given expected global IPv6 prefix to be matched

        Returns:
            a global IPv6 address
        """
        assert not eth
        # get global addrs set if multiple
        globalAddrs = self.__getGlobal()

        if filterByPrefix is None:
            return globalAddrs[0]
        else:
            for fullIp in globalAddrs:
                if fullIp.startswith(filterByPrefix):
                    return fullIp
            return str(globalAddrs[0])

    @API
    def getShortAddress(self):
        """get Rloc16 short address of Thread device"""
        return self.getRloc16()

    @API
    def getULA64(self):
        """get mesh local EID of Thread device"""
        return self.__executeCommand("ipaddr mleid")[0]

    @API
    def setMLPrefix(self, sMeshLocalPrefix):
        """set mesh local prefix"""
        cmd = "dataset meshlocalprefix %s" % sMeshLocalPrefix
        self.hasActiveDatasetToCommit = True
        return self.__executeCommand(cmd)[-1] == "Done"

    @API
    def getML16(self):
        """get mesh local 16 unicast address (Rloc)"""
        return self.getRloc()

    @API
    def downgradeToDevice(self):
        pass

    @API
    def upgradeToRouter(self):
        pass

    @API
    def forceSetSlaac(self, slaacAddress):
        """force to set a slaac IPv6 address to Thread interface

        Args:
            slaacAddress: a slaac IPv6 address to be set

        Returns:
            True: successful to set slaac address to Thread interface
            False: fail to set slaac address to Thread interface
        """
        cmd = "ipaddr add %s" % str(slaacAddress)
        return self.__executeCommand(cmd)[-1] == "Done"

    @API
    def setSleepyNodePollTime(self):
        pass

    @API
    def enableAutoDUTObjectFlag(self):
        """set AutoDUTenable flag"""
        self.AutoDUTEnable = True

    @API
    def getChildTimeoutValue(self):
        """get child timeout"""
        childTimeout = self.__executeCommand("childtimeout")[0]
        return int(childTimeout)

    @API
    def diagnosticGet(self, strDestinationAddr, listTLV_ids=()):
        if not listTLV_ids:
            return

        if len(listTLV_ids) == 0:
            return

        cmd = "networkdiagnostic get %s %s" % (
            strDestinationAddr,
            " ".join([str(tlv) for tlv in listTLV_ids]),
        )

        return self.__sendCommand(cmd, expectEcho=False)

    @API
    def diagnosticReset(self, strDestinationAddr, listTLV_ids=()):
        if not listTLV_ids:
            return

        if len(listTLV_ids) == 0:
            return

        cmd = "networkdiagnostic reset %s %s" % (
            strDestinationAddr,
            " ".join([str(tlv) for tlv in listTLV_ids]),
        )

        return self.__executeCommand(cmd)

    @API
    def diagnosticQuery(self, strDestinationAddr="", listTLV_ids=[], rloc16=0):
        if 29 in listTLV_ids:
            listTLV_ids.remove(29)
            cmd = "meshdiag childtable 0x%x" % rloc16
            return self.__executeCommand(cmd)

        if 30 in listTLV_ids:
            listTLV_ids.remove(30)
            cmd = "meshdiag childip6 0x%x" % rloc16
            return self.__executeCommand(cmd)

        if 31 in listTLV_ids:
            listTLV_ids.remove(31)
            cmd = "meshdiag routerneighbortable 0x%x" % rloc16
            return self.__executeCommand(cmd)

        if len(listTLV_ids) > 0:
            self.diagnosticGet(strDestinationAddr, listTLV_ids)

    @API
    def vendorInfo(self, name="", model="", version=""):
        vn = "vendor name %s" % name
        vm = "vendor model %s" % model
        vv = "vendor swversion %s" % version
        self.__executeCommand(vn)
        self.__executeCommand(vm)
        self.__executeCommand(vv)

    @API
    def startNativeCommissioner(self, strPSKc="GRLPASSPHRASE"):
        # TODO: Support the whole Native Commissioner functionality
        # Currently it only aims to trigger a Discovery Request message to pass
        # Certification test 5.8.4
        self.__executeCommand("ifconfig up")
        cmd = "joiner start %s" % (strPSKc)
        return self.__executeCommand(cmd)[-1] == "Done"

    @API
    def startExternalCommissioner(self, baAddr, baPort, check=True):
        """Start external commissioner
        Args:
            baAddr: A string represents the border agent address.
            baPort: An integer represents the border agent port.
        Returns:
            A boolean indicates whether this function succeed.
        """
        if self.externalCommissioner is None:
            config = commissioner.Configuration()
            config.isCcmMode = False
            config.domainName = OpenThreadTHCI.DOMAIN_NAME
            config.pskc = bytearray.fromhex(self.pskc)

            self.externalCommissioner = OTCommissioner(config, self)

        if not self.externalCommissioner.isActive():
            self.externalCommissioner.start(baAddr, baPort, check)

        if check and not self.externalCommissioner.isActive():
            raise commissioner.Error("external commissioner is not active")

        return True

    @API
    def stopExternalCommissioner(self):
        """Stop external commissioner
        Returns:
            A boolean indicates whether this function succeed.
        """
        if self.externalCommissioner is not None:
            self.externalCommissioner.stop()
            return not self.externalCommissioner.isActive()

    @API
    def configExternalCommissioner(self):
        if self.externalCommissioner is None:
            config = commissioner.Configuration()
            config.isCcmMode = False
            config.domainName = OpenThreadTHCI.DOMAIN_NAME
            config.pskc = bytearray.fromhex(self.pskc)

            self.externalCommissioner = OTCommissioner(config, self)

    @API
    def setAdminCode(self, code=""):
        if self.externalCommissioner is None:
            config = commissioner.Configuration()
            config.isCcmMode = False
            config.domainName = OpenThreadTHCI.DOMAIN_NAME
            config.pskc = bytearray.fromhex(self.pskc)

            self.externalCommissioner = OTCommissioner(config, self)

        if not self.externalCommissioner.isActive():
            self.externalCommissioner._execute_and_check(f"config set admincode {code}")

    @API
    def getActiveDataset(self):
        if self.externalCommissioner is not None:
            return self.externalCommissioner.MGMT_ACTIVE_GET(tlvTypes="")

    @API
    def getPendingDataset(self):
        if self.externalCommissioner is not None:
            return self.externalCommissioner.MGMT_PENDING_GET(tlvTypes="")

    @API
    def startCollapsedCommissioner(self, role=Thread_Device_Role.Leader):
        """start Collapsed Commissioner

        Returns:
            True: successful to start Commissioner
            False: fail to start Commissioner
        """
        if self.__startOpenThread():
            self.wait_for_attach_to_the_network(
                expected_role=self.deviceRole,
                timeout=self.NETWORK_ATTACHMENT_TIMEOUT,
                raise_assert=True,
            )
            cmd = "commissioner start"
            if self.__executeCommand(cmd)[-1] == "Done":
                self.isActiveCommissioner = True
                self.sleep(20)  # time for petition process
                return True
        return False

    @API
    def setJoinKey(self, strPSKc):
        pass

    @API
    def scanJoiner(self, xEUI="*", strPSKd="THREADJPAKETEST"):
        """scan Joiner

        Args:
            xEUI: Joiner's EUI-64
            strPSKd: Joiner's PSKd for commissioning

        Returns:
            True: successful to add Joiner's steering data
            False: fail to add Joiner's steering data
        """
        self.log("scanJoiner on channel %s", self.getChannel())

        # long timeout value to avoid automatic joiner removal (in seconds)
        timeout = 500

        if not isinstance(xEUI, str):
            eui64 = self.__convertLongToHex(xEUI, 16)
        else:
            eui64 = xEUI

        strPSKd = self.__normalizePSKd(strPSKd)

        cmd = "commissioner joiner add %s %s %s" % (
            self._deviceEscapeEscapable(eui64),
            strPSKd,
            str(timeout),
        )

        if self.__executeCommand(cmd)[-1] == "Done":
            if self.logThreadStatus == self.logStatus["stop"]:
                self.logThread = ThreadRunner.run(
                    target=self.__readCommissioningLogs, args=(120,)
                )
            return True
        else:
            return False

    @staticmethod
    def __normalizePSKd(strPSKd):
        return (
            strPSKd.upper()
            .replace("I", "1")
            .replace("O", "0")
            .replace("Q", "0")
            .replace("Z", "2")
        )

    @API
    def setProvisioningUrl(self, strURL="grl.com"):
        """set provisioning Url

        Args:
            strURL: Provisioning Url string

        Returns:
            True: successful to set provisioning Url
            False: fail to set provisioning Url
        """
        self.provisioningUrl = strURL
        if self.deviceRole == Thread_Device_Role.Commissioner:
            cmd = "commissioner provisioningurl %s" % (strURL)
            return self.__executeCommand(cmd)[-1] == "Done"
        return True

    @API
    def allowCommission(self):
        """start commissioner candidate petition process

        Returns:
            True: successful to start commissioner candidate petition process
            False: fail to start commissioner candidate petition process
        """
        cmd = "commissioner start"
        if self.__executeCommand(cmd)[-1] == "Done":
            self.isActiveCommissioner = True
            # time for petition process and at least one keep alive
            self.sleep(3)
            return True
        else:
            return False

    @API
    def joinCommissioned(self, strPSKd="THREADJPAKETEST", waitTime=20):
        """start joiner

        Args:
            strPSKd: Joiner's PSKd

        Returns:
            True: successful to start joiner
            False: fail to start joiner
        """
        self.log("joinCommissioned on channel %s", self.getChannel())

        if self.deviceRole in [
            Thread_Device_Role.Leader,
            Thread_Device_Role.Router,
            Thread_Device_Role.REED,
        ]:
            self.__setRouterSelectionJitter(1)
        self.__executeCommand("ifconfig up")
        strPSKd = self.__normalizePSKd(strPSKd)
        cmd = "joiner start %s %s" % (strPSKd, self.provisioningUrl)
        if self.__executeCommand(cmd)[-1] == "Done":
            maxDuration = 150  # seconds
            self.joinCommissionedStatus = self.joinStatus["ongoing"]

            if self.logThreadStatus == self.logStatus["stop"]:
                self.logThread = ThreadRunner.run(
                    target=self.__readCommissioningLogs, args=(maxDuration,)
                )

            t_end = time.time() + maxDuration
            while time.time() < t_end:
                if self.joinCommissionedStatus == self.joinStatus["succeed"]:
                    break
                elif self.joinCommissionedStatus == self.joinStatus["failed"]:
                    return False

                self.sleep(1)

            self.setMAC(self.mac)
            self.__executeCommand("thread start")
            self.wait_for_attach_to_the_network(
                expected_role=self.deviceRole,
                timeout=self.NETWORK_ATTACHMENT_TIMEOUT,
                raise_assert=True,
            )
            return True
        else:
            return False

    @API
    def getCommissioningLogs(self):
        """get Commissioning logs

        Returns:
           Commissioning logs
        """
        rawLogs = self.logThread.get()
        ProcessedLogs = []
        payload = []

        while not rawLogs.empty():
            rawLogEach = rawLogs.get()
            if "[THCI]" not in rawLogEach:
                continue

            EncryptedPacket = PlatformDiagnosticPacket()
            infoList = rawLogEach.split("[THCI]")[1].split("]")[0].split("|")
            for eachInfo in infoList:
                info = eachInfo.split("=")
                infoType = info[0].strip()
                infoValue = info[1].strip()
                if "direction" in infoType:
                    EncryptedPacket.Direction = (
                        PlatformDiagnosticPacket_Direction.IN
                        if "recv" in infoValue
                        else PlatformDiagnosticPacket_Direction.OUT
                        if "send" in infoValue
                        else PlatformDiagnosticPacket_Direction.UNKNOWN
                    )
                elif "type" in infoType:
                    EncryptedPacket.Type = (
                        PlatformDiagnosticPacket_Type.JOIN_FIN_req
                        if "JOIN_FIN.req" in infoValue
                        else PlatformDiagnosticPacket_Type.JOIN_FIN_rsp
                        if "JOIN_FIN.rsp" in infoValue
                        else PlatformDiagnosticPacket_Type.JOIN_ENT_req
                        if "JOIN_ENT.ntf" in infoValue
                        else PlatformDiagnosticPacket_Type.JOIN_ENT_rsp
                        if "JOIN_ENT.rsp" in infoValue
                        else PlatformDiagnosticPacket_Type.UNKNOWN
                    )
                elif "len" in infoType:
                    bytesInEachLine = 16
                    EncryptedPacket.TLVsLength = int(infoValue)
                    payloadLineCount = (
                        int(infoValue) + bytesInEachLine - 1
                    ) // bytesInEachLine
                    while payloadLineCount > 0:
                        payloadLineCount = payloadLineCount - 1
                        payloadLine = rawLogs.get()
                        payloadSplit = payloadLine.split("|")
                        for block in range(1, 3):
                            payloadBlock = payloadSplit[block]
                            payloadValues = payloadBlock.split(" ")
                            for num in range(1, 9):
                                if payloadValues[num].isalnum():
                                    payload.append(int(payloadValues[num], 16))

                    EncryptedPacket.TLVs = (
                        PlatformPackets.read(EncryptedPacket.Type, payload)
                        if payload != []
                        else []
                    )

            ProcessedLogs.append(EncryptedPacket)
        return ProcessedLogs

    @API
    def MGMT_ED_SCAN(
        self,
        sAddr,
        xCommissionerSessionId,
        listChannelMask,
        xCount,
        xPeriod,
        xScanDuration,
    ):
        """send MGMT_ED_SCAN message to a given destinaition.

        Args:
            sAddr: IPv6 destination address for this message
            xCommissionerSessionId: commissioner session id
            listChannelMask: a channel array to indicate which channels to be scaned
            xCount: number of IEEE 802.15.4 ED Scans (milliseconds)
            xPeriod: Period between successive IEEE802.15.4 ED Scans (milliseconds)
            xScanDuration: ScanDuration when performing an IEEE 802.15.4 ED Scan (milliseconds)

        Returns:
            True: successful to send MGMT_ED_SCAN message.
            False: fail to send MGMT_ED_SCAN message
        """
        channelMask = "0x" + self.__convertLongToHex(
            self.__convertChannelMask(listChannelMask)
        )
        cmd = "commissioner energy %s %s %s %s %s" % (
            channelMask,
            xCount,
            xPeriod,
            xScanDuration,
            sAddr,
        )
        return self.__executeCommand(cmd)[-1] == "Done"

    @API
    def MGMT_PANID_QUERY(self, sAddr, xCommissionerSessionId, listChannelMask, xPanId):
        """send MGMT_PANID_QUERY message to a given destination

        Args:
            xPanId: a given PAN ID to check the conflicts

        Returns:
            True: successful to send MGMT_PANID_QUERY message.
            False: fail to send MGMT_PANID_QUERY message.
        """
        panid = ""
        channelMask = "0x" + self.__convertLongToHex(
            self.__convertChannelMask(listChannelMask)
        )

        if not isinstance(xPanId, str):
            panid = str(hex(xPanId))

        cmd = "commissioner panid %s %s %s" % (panid, channelMask, sAddr)
        return self.__executeCommand(cmd)[-1] == "Done"

    @API
    def MGMT_ANNOUNCE_BEGIN(
        self, sAddr, xCommissionerSessionId, listChannelMask, xCount, xPeriod
    ):
        """send MGMT_ANNOUNCE_BEGIN message to a given destination

        Returns:
            True: successful to send MGMT_ANNOUNCE_BEGIN message.
            False: fail to send MGMT_ANNOUNCE_BEGIN message.
        """
        channelMask = "0x" + self.__convertLongToHex(
            self.__convertChannelMask(listChannelMask)
        )
        cmd = "commissioner announce %s %s %s %s" % (
            channelMask,
            xCount,
            xPeriod,
            sAddr,
        )
        return self.__executeCommand(cmd)[-1] == "Done"

    @API
    def MGMT_ACTIVE_GET(self, Addr="", TLVs=()):
        """send MGMT_ACTIVE_GET command

        Returns:
            True: successful to send MGMT_ACTIVE_GET
            False: fail to send MGMT_ACTIVE_GET
        """
        cmd = "dataset mgmtgetcommand active"

        if Addr != "":
            cmd += " address "
            cmd += Addr

        if len(TLVs) != 0:
            tlvs = "".join("%02x" % tlv for tlv in TLVs)
            cmd += " " + self.__replaceCommands["-x"] + " "
            cmd += tlvs

        return self.__executeCommand(cmd)[-1] == "Done"

    @API
    def MGMT_ACTIVE_SET(
        self,
        sAddr="",
        xCommissioningSessionId=None,
        listActiveTimestamp=None,
        listChannelMask=None,
        xExtendedPanId=None,
        sNetworkName=None,
        sPSKc=None,
        listSecurityPolicy=None,
        xChannel=None,
        sMeshLocalPrefix=None,
        xMasterKey=None,
        xPanId=None,
        xTmfPort=None,
        xSteeringData=None,
        xBorderRouterLocator=None,
        BogusTLV=None,
        xDelayTimer=None,
    ):
        """send MGMT_ACTIVE_SET command

        Returns:
            True: successful to send MGMT_ACTIVE_SET
            False: fail to send MGMT_ACTIVE_SET
        """
        cmd = "dataset mgmtsetcommand active"

        if listActiveTimestamp is not None:
            cmd += " activetimestamp "
            cmd += str(listActiveTimestamp[0])

        if xExtendedPanId is not None:
            cmd += " extpanid "
            xpanid = self.__convertLongToHex(xExtendedPanId, 16)

            cmd += xpanid

        if sNetworkName is not None:
            cmd += " networkname "
            cmd += self._deviceEscapeEscapable(str(sNetworkName))

        if xChannel is not None:
            cmd += " channel "
            cmd += str(xChannel)

        if sMeshLocalPrefix is not None:
            cmd += " localprefix "
            cmd += str(sMeshLocalPrefix)

        if xMasterKey is not None:
            cmd += " " + self.__replaceCommands["networkkey"] + " "
            key = self.__convertLongToHex(xMasterKey, 32)

            cmd += key

        if xPanId is not None:
            cmd += " panid "
            cmd += str(xPanId)

        if listChannelMask is not None:
            cmd += " channelmask "
            cmd += "0x" + self.__convertLongToHex(
                self.__convertChannelMask(listChannelMask)
            )

        if (
            sPSKc is not None
            or listSecurityPolicy is not None
            or xCommissioningSessionId is not None
            or xTmfPort is not None
            or xSteeringData is not None
            or xBorderRouterLocator is not None
            or BogusTLV is not None
        ):
            cmd += " " + self.__replaceCommands["-x"] + " "

        if sPSKc is not None:
            cmd += "0410"
            stretchedPskc = Thread_PBKDF2.get(
                sPSKc,
                ModuleHelper.Default_XpanId,
                ModuleHelper.Default_NwkName,
            )
            pskc = "%x" % stretchedPskc

            if len(pskc) < 32:
                pskc = pskc.zfill(32)

            cmd += pskc

        if listSecurityPolicy is not None:
            if self.DeviceCapability == DevCapb.V1_1:
                cmd += "0c03"
            else:
                cmd += "0c04"

            rotationTime = 0
            policyBits = 0

            # previous passing way listSecurityPolicy=[True, True, 3600,
            # False, False, True]
            if len(listSecurityPolicy) == 6:
                rotationTime = listSecurityPolicy[2]

                # the last three reserved bits must be 1
                policyBits = 0b00000111

                if listSecurityPolicy[0]:
                    policyBits = policyBits | 0b10000000
                if listSecurityPolicy[1]:
                    policyBits = policyBits | 0b01000000
                if listSecurityPolicy[3]:
                    policyBits = policyBits | 0b00100000
                if listSecurityPolicy[4]:
                    policyBits = policyBits | 0b00010000
                if listSecurityPolicy[5]:
                    policyBits = policyBits | 0b00001000
            else:
                # new passing way listSecurityPolicy=[3600, 0b11001111]
                rotationTime = listSecurityPolicy[0]
                # bit order
                if len(listSecurityPolicy) > 2:
                    policyBits = listSecurityPolicy[2] << 8 | listSecurityPolicy[1]
                else:
                    policyBits = listSecurityPolicy[1]

            policy = str(hex(rotationTime))[2:]

            if len(policy) < 4:
                policy = policy.zfill(4)

            cmd += policy

            flags0 = ("%x" % (policyBits & 0x00FF)).ljust(2, "0")
            cmd += flags0

            if self.DeviceCapability != DevCapb.V1_1:
                flags1 = ("%x" % ((policyBits & 0xFF00) >> 8)).ljust(2, "0")
                cmd += flags1

        if xCommissioningSessionId is not None:
            cmd += "0b02"
            sessionid = str(hex(xCommissioningSessionId))[2:]

            if len(sessionid) < 4:
                sessionid = sessionid.zfill(4)

            cmd += sessionid

        if xBorderRouterLocator is not None:
            cmd += "0902"
            locator = str(hex(xBorderRouterLocator))[2:]

            if len(locator) < 4:
                locator = locator.zfill(4)

            cmd += locator

        if xSteeringData is not None:
            steeringData = self.__convertLongToHex(xSteeringData)
            cmd += "08" + str(len(steeringData) // 2).zfill(2)
            cmd += steeringData

        if BogusTLV is not None:
            cmd += "8202aa55"

        return self.__executeCommand(cmd)[-1] == "Done"

    @API
    def MGMT_PENDING_GET(self, Addr="", TLVs=()):
        """send MGMT_PENDING_GET command

        Returns:
            True: successful to send MGMT_PENDING_GET
            False: fail to send MGMT_PENDING_GET
        """
        cmd = "dataset mgmtgetcommand pending"

        if Addr != "":
            cmd += " address "
            cmd += Addr

        if len(TLVs) != 0:
            tlvs = "".join("%02x" % tlv for tlv in TLVs)
            cmd += " " + self.__replaceCommands["-x"] + " "
            cmd += tlvs

        return self.__executeCommand(cmd)[-1] == "Done"

    @API
    def MGMT_PENDING_SET(
        self,
        sAddr="",
        xCommissionerSessionId=None,
        listPendingTimestamp=None,
        listActiveTimestamp=None,
        xDelayTimer=None,
        xChannel=None,
        xPanId=None,
        xMasterKey=None,
        sMeshLocalPrefix=None,
        sNetworkName=None,
    ):
        """send MGMT_PENDING_SET command

        Returns:
            True: successful to send MGMT_PENDING_SET
            False: fail to send MGMT_PENDING_SET
        """
        cmd = "dataset mgmtsetcommand pending"

        if listPendingTimestamp is not None:
            cmd += " pendingtimestamp "
            cmd += str(listPendingTimestamp[0])

        if listActiveTimestamp is not None:
            cmd += " activetimestamp "
            cmd += str(listActiveTimestamp[0])

        if xDelayTimer is not None:
            cmd += " delaytimer "
            cmd += str(xDelayTimer)
            # cmd += ' delaytimer 3000000'

        if xChannel is not None:
            cmd += " channel "
            cmd += str(xChannel)

        if xPanId is not None:
            cmd += " panid "
            cmd += str(xPanId)

        if xMasterKey is not None:
            cmd += " " + self.__replaceCommands["networkkey"] + " "
            key = self.__convertLongToHex(xMasterKey, 32)

            cmd += key

        if sMeshLocalPrefix is not None:
            cmd += " localprefix "
            cmd += str(sMeshLocalPrefix)

        if sNetworkName is not None:
            cmd += " networkname "
            cmd += self._deviceEscapeEscapable(str(sNetworkName))

        if xCommissionerSessionId is not None:
            cmd += " " + self.__replaceCommands["-x"] + " "
            cmd += "0b02"
            sessionid = str(hex(xCommissionerSessionId))[2:]

            if len(sessionid) < 4:
                sessionid = sessionid.zfill(4)

            cmd += sessionid

        return self.__executeCommand(cmd)[-1] == "Done"

    @API
    def MGMT_COMM_GET(self, Addr="ff02::1", TLVs=()):
        """send MGMT_COMM_GET command

        Returns:
            True: successful to send MGMT_COMM_GET
            False: fail to send MGMT_COMM_GET
        """
        cmd = "commissioner mgmtget"

        if len(TLVs) != 0:
            tlvs = "".join("%02x" % tlv for tlv in TLVs)
            cmd += " " + self.__replaceCommands["-x"] + " "
            cmd += tlvs

        return self.__executeCommand(cmd)[-1] == "Done"

    @API
    def MGMT_COMM_SET(
        self,
        Addr="ff02::1",
        xCommissionerSessionID=None,
        xSteeringData=None,
        xBorderRouterLocator=None,
        xChannelTlv=None,
        ExceedMaxPayload=False,
    ):
        """send MGMT_COMM_SET command

        Returns:
            True: successful to send MGMT_COMM_SET
            False: fail to send MGMT_COMM_SET
        """
        cmd = "commissioner mgmtset"

        if xCommissionerSessionID is not None:
            # use assigned session id
            cmd += " sessionid "
            cmd += str(xCommissionerSessionID)
        elif xCommissionerSessionID is None:
            # use original session id
            if self.isActiveCommissioner is True:
                cmd += " sessionid "
                cmd += self.__getCommissionerSessionId()
            else:
                pass

        if xSteeringData is not None:
            cmd += " steeringdata "
            cmd += str(hex(xSteeringData)[2:])

        if xBorderRouterLocator is not None:
            cmd += " locator "
            cmd += str(hex(xBorderRouterLocator))

        if xChannelTlv is not None:
            cmd += " " + self.__replaceCommands["-x"] + " "
            cmd += "000300" + "%04x" % xChannelTlv

        return self.__executeCommand(cmd)[-1] == "Done"

    @API
    def setPSKc(self, strPSKc):
        cmd = "dataset pskc %s" % strPSKc
        self.hasActiveDatasetToCommit = True
        return self.__executeCommand(cmd)[-1] == "Done"

    @API
    def setActiveTimestamp(self, xActiveTimestamp):
        self.activetimestamp = xActiveTimestamp
        self.hasActiveDatasetToCommit = True
        cmd = "dataset activetimestamp %s" % str(xActiveTimestamp)
        return self.__executeCommand(cmd)[-1] == "Done"

    @API
    def setUdpJoinerPort(self, portNumber):
        """set Joiner UDP Port

        Args:
            portNumber: Joiner UDP Port number

        Returns:
            True: successful to set Joiner UDP Port
            False: fail to set Joiner UDP Port
        """
        cmd = "joinerport %d" % portNumber
        return self.__executeCommand(cmd)[-1] == "Done"

    @API
    def commissionerUnregister(self):
        """stop commissioner

        Returns:
            True: successful to stop commissioner
            False: fail to stop commissioner
        """
        cmd = "commissioner stop"
        return self.__executeCommand(cmd)[-1] == "Done"

    @API
    def sendBeacons(self, sAddr, xCommissionerSessionId, listChannelMask, xPanId):
        self.__sendCommand("scan", expectEcho=False)

    @API
    def updateRouterStatus(self):
        """force update to router as if there is child id request"""
        self._update_router_status = True

    @API
    def __updateRouterStatus(self):
        cmd = "state"
        while True:
            state = self.__executeCommand(cmd)[0]
            if state == "detached":
                continue
            elif state == "child":
                break
            else:
                return False

        cmd = "state router"
        return self.__executeCommand(cmd)[-1] == "Done"

    @API
    def setRouterThresholdValues(self, upgradeThreshold, downgradeThreshold):
        self.__setRouterUpgradeThreshold(upgradeThreshold)
        self.__setRouterDowngradeThreshold(downgradeThreshold)

    @API
    def setMinDelayTimer(self, iSeconds):
        cmd = "delaytimermin %s" % iSeconds
        return self.__executeCommand(cmd)[-1] == "Done"

    @API
    def ValidateDeviceFirmware(self):
        assert (
            not self.IsBorderRouter
        ), "Method not expected to be used with border router devices"

        # As this is 1.3 and 1.4 specific THCI, 1.1, 1.2 device should not be handled here
        # if self.DeviceCapability == OT11_CAPBS:
        #     return OT11_VERSION in self.UIStatusMsg
        # elif self.DeviceCapability == OT12_CAPBS:
        #     return OT12_VERSION in self.UIStatusMsg
        if self.DeviceCapability == OT13_CAPBS:
            return [
                True if OT13_VER in self.UIStatusMsg else False
                for OT13_VER in OT13_VERSION
            ]
        else:
            return False

    @API
    def setBbrDataset(
        self, SeqNumInc=False, SeqNum=None, MlrTimeout=None, ReRegDelay=None
    ):
        """set BBR Dataset

        Args:
            SeqNumInc:  Increase `SeqNum` by 1 if True.
            SeqNum:     Set `SeqNum` to a given value if not None.
            MlrTimeout: Set `MlrTimeout` to a given value.
            ReRegDelay: Set `ReRegDelay` to a given value.

            MUST NOT set SeqNumInc to True and SeqNum to non-None value at the same time.

        Returns:
            True: successful to set BBR Dataset
            False: fail to set BBR Dataset
        """
        assert not (
            SeqNumInc and SeqNum is not None
        ), "Must not specify both SeqNumInc and SeqNum"

        if (MlrTimeout and MlrTimeout != self.bbrMlrTimeout) or (
            ReRegDelay and ReRegDelay != self.bbrReRegDelay
        ):
            if SeqNum is None:
                SeqNumInc = True

        if SeqNumInc:
            if self.bbrSeqNum in (126, 127):
                self.bbrSeqNum = 0
            elif self.bbrSeqNum in (254, 255):
                self.bbrSeqNum = 128
            else:
                self.bbrSeqNum = (self.bbrSeqNum + 1) % 256
        elif SeqNum is not None:
            self.bbrSeqNum = SeqNum

        return self.__configBbrDataset(
            SeqNum=self.bbrSeqNum, MlrTimeout=MlrTimeout, ReRegDelay=ReRegDelay
        )

    def __configBbrDataset(self, SeqNum=None, MlrTimeout=None, ReRegDelay=None):
        if MlrTimeout is not None and ReRegDelay is None:
            ReRegDelay = self.bbrReRegDelay

        cmd = "bbr config"
        if SeqNum is not None:
            cmd += " seqno %d" % SeqNum
        if ReRegDelay is not None:
            cmd += " delay %d" % ReRegDelay
        if MlrTimeout is not None:
            cmd += " timeout %d" % MlrTimeout
        ret = self.__executeCommand(cmd)[-1] == "Done"

        if SeqNum is not None:
            self.bbrSeqNum = SeqNum

        if MlrTimeout is not None:
            self.bbrMlrTimeout = MlrTimeout

        if ReRegDelay is not None:
            self.bbrReRegDelay = ReRegDelay

        cmd = self.__replaceCommands["netdata register"]
        self.__executeCommand(cmd)

        return ret

    # Low power THCI
    @API
    def setCSLtout(self, tout=30):
        self.ssedTimeout = tout
        cmd = "csl timeout %u" % self.ssedTimeout
        return self.__executeCommand(cmd)[-1] == "Done"

    @API
    def setCSLchannel(self, ch=11):
        cmd = "csl channel %u" % ch
        return self.__executeCommand(cmd)[-1] == "Done"

    @API
    def setCSLperiod(self, period=500):
        """set Csl Period
        Args:
            period: csl period in ms

        note: OT command 'csl period' accepts parameter in unit of 10 symbols,
        period is converted from unit ms to ten symbols (160us per 10 symbols).

        """
        cmd = "csl period %u" % (period * 1000)
        return self.__executeCommand(cmd)[-1] == "Done"

    @staticmethod
    def getForwardSeriesFlagsFromHexOrStr(flags):
        hexFlags = int(flags, 16) if isinstance(flags, str) else flags
        strFlags = ""
        if hexFlags == 0:
            strFlags = "X"
        else:
            if hexFlags & 0x1 != 0:
                strFlags += "l"
            if hexFlags & 0x2 != 0:
                strFlags += "d"
            if hexFlags & 0x4 != 0:
                strFlags += "r"
            if hexFlags & 0x8 != 0:
                strFlags += "a"

        return strFlags

    @staticmethod
    def mapMetricsHexToChar(metrics):
        metricsFlagMap = {
            0x40: "p",
            0x09: "q",
            0x0A: "m",
            0x0B: "r",
        }
        metricsReservedFlagMap = {0x11: "q", 0x12: "m", 0x13: "r"}
        if metricsFlagMap.get(metrics):
            return metricsFlagMap.get(metrics), False
        elif metricsReservedFlagMap.get(metrics):
            return metricsReservedFlagMap.get(metrics), True
        else:
            logging.warning(
                "Not found flag mapping for given metrics: {}".format(metrics)
            )
            return "", False

    @staticmethod
    def getMetricsFlagsFromHexStr(metrics):
        strMetrics = ""
        reserved_flag = ""

        if metrics.startswith("0x"):
            metrics = metrics[2:]
        hexMetricsArray = bytearray.fromhex(metrics)

        for metric in hexMetricsArray:
            metric_flag, has_reserved_flag = OpenThreadTHCI.mapMetricsHexToChar(metric)
            strMetrics += metric_flag
            if has_reserved_flag:
                reserved_flag = " r"

        return strMetrics + reserved_flag

    @API
    def LinkMetricsSingleReq(self, dst_addr, metrics):
        cmd = "linkmetrics query %s single %s" % (
            dst_addr,
            self.getMetricsFlagsFromHexStr(metrics),
        )
        return self.__executeCommand(cmd)[-1] == "Done"

    @API
    def LinkMetricsMgmtReq(self, dst_addr, type_, flags, metrics, series_id):
        cmd = "linkmetrics mgmt %s " % dst_addr
        if type_ == "FWD":
            cmd += "forward %d %s" % (
                series_id,
                self.getForwardSeriesFlagsFromHexOrStr(flags),
            )
            if flags != 0:
                cmd += " %s" % (self.getMetricsFlagsFromHexStr(metrics))
        elif type_ == "ENH":
            cmd += "enhanced-ack"
            if flags != 0:
                cmd += " register %s" % (self.getMetricsFlagsFromHexStr(metrics))
            else:
                cmd += " clear"
        return self.__executeCommand(cmd)[-1] == "Done"

    @API
    def LinkMetricsGetReport(self, dst_addr, series_id):
        cmd = "linkmetrics query %s forward %d" % (dst_addr, series_id)
        return self.__executeCommand(cmd)[-1] == "Done"

    # TODO: Series Id is not in this API.
    @API
    def LinkMetricsSendProbe(self, dst_addr, ack=True, size=0):
        cmd = "linkmetrics probe %s %d" % (dst_addr, size)
        return self.__executeCommand(cmd)[-1] == "Done"

    @API
    def setTxPower(self, level):
        cmd = "txpower "
        if level == "HIGH":
            cmd += "127"
        elif level == "MEDIUM":
            cmd += "0"
        elif level == "LOW":
            cmd += "-128"
        else:
            print("wrong Tx Power level")
        return self.__executeCommand(cmd)[-1] == "Done"

    @API
    def sendUdp(self, destination, port, payload="hello"):
        assert payload is not None, "payload should not be none"
        cmd = "udp send %s %d %s" % (destination, port, payload)
        return self.__executeCommand(cmd)[-1] == "Done"

    @API
    def send_udp(self, interface, destination, port, payload="12ABcd"):
        """payload hexstring"""
        assert payload is not None, "payload should not be none"
        assert interface == 0, "non-BR must send UDP to Thread interface"
        self.__udpOpen()
        time.sleep(0.5)
        cmd = "udp send %s %s -x %s" % (destination, port, payload)
        result = self.__executeCommand(cmd)[-1] == "Done"

        return result

    def __udpOpen(self):
        if not self.__isUdpOpened:
            cmd = "udp open"
            self.__executeCommand(cmd)

            # Bind to RLOC address and first dynamic port
            rlocAddr = self.getRloc()

            cmd = "udp bind %s 49152" % rlocAddr
            self.__executeCommand(cmd)

            self.__isUdpOpened = True

    @API
    def sendMACcmd(self, enh=False):
        cmd = "mac send datarequest"
        return self.__executeCommand(cmd)[-1] == "Done"

    @API
    def sendMACdata(self, enh=False):
        cmd = "mac send emptydata"
        return self.__executeCommand(cmd)[-1] == "Done"

    @API
    def setCSLsuspension(self, suspend):
        if suspend:
            self.__setPollPeriod(240 * 1000)
        else:
            self.__setPollPeriod(int(0.9 * self.ssedTimeout * 1000))

    @API
    def set_max_addrs_per_child(self, num):
        cmd = "childip max %d" % int(num)
        self.__executeCommand(cmd)

    @API
    def config_next_dua_status_rsp(self, mliid, status_code):
        if status_code >= 400:
            # map status_code to correct COAP response code
            a, b = divmod(status_code, 100)
            status_code = ((a & 0x7) << 5) + (b & 0x1F)

        cmd = "bbr mgmt dua %d" % status_code

        if mliid is not None:
            mliid = mliid.replace(":", "")
            cmd += " %s" % mliid

        self.__executeCommand(cmd)

    @API
    def getDUA(self):
        dua = self.getGUA("fd00:7d03")
        return dua

    def __addDefaultDomainPrefix(self):
        self.configBorderRouter(P_dp=1, P_stable=1, P_on_mesh=1, P_default=1)

    def __setDUA(self, sDua):
        """specify the DUA before Thread Starts."""

        iid = ipaddress.IPv6Address(sDua).packed[-8:]
        cmd = "dua iid %s" % "".join("%02x" % b for b in iid)
        return self.__executeCommand(cmd)[-1] == "Done"

    def __getMlIid(self):
        """get the Mesh Local IID."""
        # getULA64() would return the full string representation
        mleid = ModuleHelper.GetFullIpv6Address(self.getULA64()).lower()
        mliid = mleid[-19:].replace(":", "")
        return mliid

    @API
    def enable_trel_connectivity(self):
        """
        Thread Radio Encapsulation Link enables Thread devices to
        communicate directly over IPv6-based link technologies other then
        802.15.4, including WI-Fi and Ethernet.
        """

        cmd = "trel enable"
        self.__executeCommand(cmd)

    def disable_trel_connectivity(self):
        """
        Thread Radio Encapsulation Link disables Thread devices to
        communicate directly over IPv6-based link technologies other then
        802.15.4, including WI-Fi and Ethernet.
        """

        cmd = "trel disable"
        self.__executeCommand(cmd)

    @API
    def disable_enable_trel_connectivity(self):
        cmd = "trel disable"
        self.__executeCommand(cmd)
        time.sleep(2)
        cmd = "trel enable"
        self.__executeCommand(cmd)

    @API
    def getMultiradioList(self):
        cmd = "multiradio neighbor list"
        return self.__executeCommand(cmd)

    @API
    def disable_15_4_radio(self):
        cmd = "radiofilter enable"
        return self.__executeCommand(cmd)

    @API
    def enable_15_4_radio(self):
        cmd = "radiofilter disable"
        return self.__executeCommand(cmd)

    @API
    def enable_TREL_radio(self):
        cmd = "trel filter disable"
        return self.__executeCommand(cmd)

    @API
    def disable_TREL_radio(self):
        cmd = "trel filter enable"
        return self.__executeCommand(cmd)

    @API
    def mle_Discovery(self, channel):
        cmd = "discover %s" % channel
        return self.__executeCommand(cmd)

    ########################################################################################################################
    #                                       1.3 THCI Commands
    #
    ########################################################################################################################
    def record_query(self):
        """
        Sends A and/or AAAA records must be of sufficient scope to be reachable by all hosts that might query the DNS.

        Returns:
            True: successfully sends record query
            False: Fail to send the record query
        """
        pass

    def get_service_instance_name(self):
        """
        get_service_instance_name Returns the SRV value of Service Description Instruction.

        Returns:
           True: successfully retrieves the SRV value of service description instruction.
           False: Fail to retrieve the SRV value of service description instruction.
        """

        # Implement-1.3-Thread
        # srp client service
        # instance:"ins2", name:"_test2._udp,_sub1,_sub2", state:Registered, port:111, priority:1, weight:1
        # instance:"ins1", name:"_test1._udp", state:Registered, port:777, priority:0, weight:0
        # https://github.com/openthread/openthread/blob/ca450e541636692909183773beba8f55591a7185/src/cli/README_SRP_CLIENT.md

        # srp server service
        # srp-api-test-1._ipps._tcp.default.service.arpa.
        #     deleted: false
        #     subtypes: (null)
        #     port: 49152
        #     priority: 0
        #     weight: 0
        #     TXT: 0130
        #     host: srp-api-test-1.default.service.arpa.
        #     addresses: [fdde:ad00:beef:0:0:ff:fe00:fc10]
        # srp-api-test-0._ipps._tcp.default.service.arpa.
        #     deleted: false
        #     subtypes: _sub1,_sub2
        #     port: 49152
        #     priority: 0
        #     weight: 0
        #     TXT: 0130
        #     host: srp-api-test-0.default.service.arpa.
        #     addresses: [fdde:ad00:beef:0:0:ff:fe00:fc10]
        # Implement-1.3-Thread

        pass

    def mdns_query(self, service_instance_name="_meshcop._udp_.local"):
        """
        Sends Multicast DNS query for SRV value of Service Description Instruction.

        Returns:
            True: successfully sends the multicast DNS query for SRV value of service description instruction.
            False: Fail to send the multicast DNS query for SRV value of service description instruction.
        """
        pass

    def __setMlIid(self, sMlIid):
        """Set the Mesh Local IID before Thread Starts."""
        assert ":" not in sMlIid
        cmd = "mliid %s" % sMlIid
        self.__executeCommand(cmd)

    @API
    def registerDUA(self, sAddr=""):
        self.__setDUA(sAddr)

    @API
    def config_next_mlr_status_rsp(self, status_code):
        cmd = "bbr mgmt mlr response %d" % status_code
        return self.__executeCommand(cmd)[-1] == "Done"

    @API
    def setMLRtimeout(self, iMsecs):
        """Setup BBR MLR Timeout to `iMsecs` seconds."""
        self.setBbrDataset(MlrTimeout=iMsecs)

    @API
    def stopListeningToAddr(self, sAddr):
        cmd = "ipmaddr del " + sAddr
        try:
            self.__executeCommand(cmd)
        except CommandError as ex:
            if ex.code == OT_ERROR_ALREADY:
                pass
            else:
                raise

        return True

    @API
    def registerMulticast(
        self, listAddr=("ff04::1234:777a:1",), timeout=MLR_TIMEOUT_MIN
    ):
        """subscribe to the given ipv6 address (sAddr) in interface and send MLR.req OTA

        Args:
            sAddr   : str : Multicast address to be subscribed and notified OTA.
        """
        for each_sAddr in listAddr:
            self._beforeRegisterMulticast(each_sAddr, timeout)

        sAddr = " ".join(listAddr)
        cmd = "ipmaddr add " + str(sAddr)

        try:
            self.__executeCommand(cmd)
        except CommandError as ex:
            if ex.code == OT_ERROR_ALREADY:
                pass
            else:
                raise

    @API
    def getMlrLogs(self):
        return self.externalCommissioner.getMlrLogs()

    @API
    def migrateNetwork(self, channel=None, net_name=None):
        """migrate to another Thread Partition 'net_name' (could be None)
        on specified 'channel'. Make sure same Mesh Local IID and DUA
        after migration for DUA-TC-06/06b (DEV-1923)
        """
        if channel is None:
            raise Exception("channel None")

        if channel not in list(range(11, 27)):
            raise Exception("channel %d not in [11, 26] Invalid" % channel)

        print(("new partition %s on channel %d" % (net_name, channel)))

        mliid = self.__getMlIid()
        dua = self.getDUA()
        self.reset()
        deviceRole = self.deviceRole
        self.setDefaultValues()
        self.setChannel(channel)
        if net_name is not None:
            self.setNetworkName(net_name)
        self.__setMlIid(mliid)
        self.__setDUA(dua)
        return self.joinNetwork(deviceRole)

    @API
    def setParentPrio(self, prio):
        cmd = "parentpriority %u" % prio
        return self.__executeCommand(cmd)[-1] == "Done"

    @API
    def role_transition(self, role):
        cmd = "mode %s" % OpenThreadTHCI._ROLE_MODE_DICT[role]
        return self.__executeCommand(cmd)[-1] == "Done"

    @API
    def setLeaderWeight(self, iWeight=72):
        self.__executeCommand("leaderweight %d" % iWeight)

    @watched
    def isBorderRoutingEnabled(self):
        try:
            self.__executeCommand("br omrprefix")
            return True
        except CommandError:
            return False

    def __detectZephyr(self):
        """Detect if the device is running Zephyr and adapt in that case"""

        try:
            self._lineSepX = re.compile(r"\r\n|\r|\n")
            if self.__executeCommand(ZEPHYR_PREFIX + "thread version")[0].isdigit():
                self._cmdPrefix = ZEPHYR_PREFIX
        except CommandError:
            self._lineSepX = LINESEPX

    def __detectReference20200818(self):
        """Detect if the device is a Thread reference 20200818"""

        # Running `version api` in Thread reference 20200818 is equivalent to running `version`
        # It will not output an API number
        self.IsReference20200818 = not self.__executeCommand("version api")[0].isdigit()

        if self.IsReference20200818:
            self.__replaceCommands = {
                "-x": "binary",
                "allowlist": "whitelist",
                "denylist": "blacklist",
                "netdata register": "netdataregister",
                "networkkey": "masterkey",
                "partitionid preferred": "leaderpartitionid",
            }
        else:

            class IdentityDict:
                def __getitem__(self, key):
                    return key

            self.__replaceCommands = IdentityDict()

    def __discoverDeviceCapability(self):
        """Discover device capability according to version"""
        thver = self.__executeCommand("thread version")[0]
        if thver in ["1.3", "1.3.1", "1.4", "4", "5"] and not self.IsBorderRouter:
            self.log(
                "Setting capability of {}: (DevCapb.C_FTD13 | DevCapb.C_MTD13)".format(
                    self
                )
            )
            self.DeviceCapability = OT13_CAPBS
        elif thver in ["1.3", "1.3.1", "1.4", "4", "5"] and self.IsBorderRouter:
            self.log(
                "Setting capability of {}: (DevCapb.C_BR13 | DevCapb.C_Host13 | DevCapb.C_Comm13)".format(
                    self
                )
            )
            self.DeviceCapability = OT13BR_CAPBS
        elif thver in ["1.2", "3"] and not self.IsBorderRouter:
            self.log(
                "Setting capability of {}: DevCapb.L_AIO | DevCapb.C_FFD | DevCapb.C_RFD".format(
                    self
                )
            )
            self.DeviceCapability = OT12_CAPBS
        elif thver in ["1.2", "3"] and self.IsBorderRouter:
            self.log(
                "Setting capability of BR {}: DevCapb.C_BBR | DevCapb.C_Host | DevCapb.C_Comm".format(
                    self
                )
            )
            self.DeviceCapability = OT12BR_CAPBS
        elif thver in ["1.1", "2"]:
            self.log("Setting capability of {}: DevCapb.V1_1".format(self))
            self.DeviceCapability = OT11_CAPBS
        else:
            self.log("Capability not specified for {}".format(self))
            self.DeviceCapability = DevCapb.NotSpecified
            assert False, thver

    @staticmethod
    def __lstrip0x(s):
        """strip 0x at the beginning of a hex string if it exists

        Args:
            s: hex string

        Returns:
            hex string with leading 0x stripped
        """
        if s.startswith("0x"):
            s = s[2:]

        return s

    @API
    def setCcmState(self, state=0):
        assert state in (0, 1), state
        self.__executeCommand("ccm {}".format("enable" if state == 1 else "disable"))

    @API
    def setVrCheckSkip(self):
        self.__executeCommand("tvcheck disable")

    @API
    def addBlockedNodeId(self, node_id):
        cmd = "nodeidfilter deny %d" % node_id
        self.__executeCommand(cmd)

    @API
    def clearBlockedNodeIds(self):
        cmd = "nodeidfilter clear"
        self.__executeCommand(cmd)

    @API
    def get_br_omr_address(self):
        cmd = "br omrprefix"
        getomr = self.__executeCommand(cmd)
        getomr = getomr[0].split("Local: ")[1]
        print(getomr)
        cmd = "ipaddr"
        ipaddr = self.__executeCommand(cmd)
        print(ipaddr)
        result = []
        for ip in ipaddr:
            try:
                if ipaddress.ip_address(str(ip)) in ipaddress.ip_network(str(getomr)):
                    result.append(ip)
            except:
                continue
        if not result:
            raise ValueError("No OMR address found for the registered OMR prefix.")
        else:
            return result

    @API
    def get_ed_omr_address(self, br_omr_prefix):
        cmd = "ipaddr"
        ipaddr = self.__executeCommand(cmd)
        print(ipaddr)
        full_addr = []
        for i in ipaddr[:-1]:
            full_addr.append(ModuleHelper.GetFullIpv6Address(i.strip()))
        print(full_addr)
        result = [x for x in full_addr if x.startswith("".join(br_omr_prefix))]
        return result
        # return self.getGUA()

    @API
    def get_own_omr_address(self, omr_prefix):
        cmd = "ipaddr"
        ipaddr = self.__executeCommand(cmd)
        print(ipaddr)
        result = []
        for ip in ipaddr:
            try:
                if ipaddress.ip_address(str(ip)) in ipaddress.ip_network(
                    str(omr_prefix)
                ):
                    result.append(ip)
            except:
                continue
        if not result:
            raise ValueError("No OMR address found for the registered OMR prefix.")
        else:
            return result

    def srp_client_remove(self, instancename, servicename):
        cmd = "srp client service remove %s %s" % (instancename, servicename)
        self.__executeCommand(cmd)
        time.sleep(3)

    def srpCallBackEnable(self):
        cmd = "srp client callback enable"
        self.__executeCommand(cmd)

    def set_ttl(self, ttl):
        cmd = "srp client ttl %s" % ttl
        self.__executeCommand(cmd)
        self.sleep(3)

    def generate_new_ip(self, ip_addr="2001::dead:beef:cafe"):
        cmd = "ipaddr add %s" % ip_addr
        new_host_addr = self.__executeCommand(cmd)
        print(new_host_addr)
        return new_host_addr

    def host_clear(self):
        cmd = "srp client host clear"
        self.__executeCommand(cmd)

    def client_key_lease(self, keylease):
        cmd = "srp client keyleaseinterval %s" % keylease
        self.__executeCommand(cmd)

    def srpCallBackDisable(self):
        cmd = "srp client callback disable"
        self.__executeCommand(cmd)

    def srp_update(
        self,
        dst_addr=None,
        service_type="_thread-test._udp",
        tcp_service_type="_thread-test._tcp,test_subtype",
        srv=None,
        weight="0",
        priority="4",
        AAAA=None,
        port=None,
        lease_option=(),
        enable_srv_key=False,
        enable_name_compression=False,
        host_addr=None,
        client_remove=False,
        is_tcp=False,
        omr=None,
        ml_eid=None,
        txt="",
    ):
        """Send Service Registration Protocol

        Args:
            dst_addr                =  Destination device address which srp_update to be sent,
                                       if not specified, srp_update is sent to BR by default.
            srv                     =  Service Description
                                       if not specified, Default would be None.
            AAAA                    =  Host Description
                                       if not specified, Default would be Ip4 or IPv6 address of the host.
            lease_option            =  Update Lease Option contains two lease time: Lease Time and Key Lease Time.

            enable_srv_key          =  If True contains the public key corresponding to the private key that was
                                       used to sign the message.

            enable_host_key         =  If True contains the public key corresponding to the private key that was used to
                                       sign the message.

            enable_name_compression =  If True send SRP update without name compression.

        Returns:
            True: DNS Update is syntactically and semantically valid
            False: DNS Update is not syntactically and semantically valid
        """

        # Implement-1.3-Thread
        # QUERY : What should be the port number?
        # Commands mentioned in the following :
        # https://github.com/openthread/openthread/blob/ca450e541636692909183773beba8f55591a7185/src/cli/README_SRP.md
        # https://github.com/openthread/openthread/blob/ca450e541636692909183773beba8f55591a7185/src/cli/README_SRP_CLIENT.md
        # srp client keyleaseinterval 864000
        # srp client leaseinterval 864000
        # https://github.com/openthread/opentread/blob/39a1f55447f3eec0cf089d8bec29b3303f3e7f99/src/cli/README.md
        # dns compression <enable/disable>
        # Implement-1.3-Thread
        lease = None
        keylease = None
        if len(lease_option) != 0:
            lease, keylease = lease_option

        if client_remove:
            cmd = "srp client host remove"
            self.__executeCommand(cmd)
        if port == "12348":
            cmd = "srp client host clear"
            print(cmd)
            self.__executeCommand(cmd)
        # cmd = 'srp client stop'
        # self.__executeCommand(cmd)
        if enable_srv_key != False:
            cmd = "srp client service key %s" % enable_srv_key
            self.__executeCommand(cmd)

        if host_addr == None:
            # mleid = ModuleHelper.GetFullIpv6Address(self.get_ed_omr_address()).lower()
            if ml_eid == None:
                cmd = "srp client host address %s" % (omr[0])
            elif omr == None:
                cmd = "srp client host address %s" % ml_eid
            else:
                cmd = "srp client host address %s %s" % (omr[0], ml_eid)
            self.__executeCommand(cmd)
        else:
            cmd = "srp client host address %s" % (host_addr)
            self.__executeCommand(cmd)

        if enable_name_compression != False:
            cmd = "dns compression %s" % enable_name_compression
            self.__executeCommand(cmd)
        if lease != None:
            cmd = "srp client leaseinterval %s" % (lease)
            self.__executeCommand(cmd)
        if keylease != None:
            cmd = "srp client keyleaseinterval %s" % (keylease)
            self.__executeCommand(cmd)

        # Not required to execute this command for port 12346
        if port in ["12345", "12348"]:
            cmd = "srp client host name %s" % (AAAA)
            print(cmd)
            self.__executeCommand(cmd)

        if is_tcp != False:
            cmd = "srp client service add %s %s %s %s %s %s" % (
                srv,
                tcp_service_type,
                port,
                weight,
                priority,
                txt,
            )
        else:
            cmd = "srp client service add %s %s %s %s %s %s" % (
                srv,
                service_type,
                port,
                weight,
                priority,
                txt,
            )
        self.__executeCommand(cmd)
        time.sleep(3)

        if dst_addr == None:
            cmd = "srp client autostart enable"
            self.__executeCommand(cmd)
            cmd1 = "srp client service"
            self.__executeCommand(cmd1)
        else:
            cmd_to_stop_srp = "srp client stop"
            self.__executeCommand(cmd_to_stop_srp)
            cmd1 = "dns config %s 53 6000 3 1" % dst_addr[1]
            self.__executeCommand(cmd1)
            cmd = "srp client start %s" % dst_addr[0]
            self.__executeCommand(cmd)

    def client_start(self, addr=""):
        cmd = "srp client start %s 53" % addr
        self.__executeCommand(cmd)

    def client_stop(self):
        cmd = "srp client stop"
        self.__executeCommand(cmd)

    def client_key_lease(self, keylease):
        cmd = "srp client keyleaseinterval %s" % keylease
        self.__executeCommand(cmd)

    def srp_add(
        self,
        dst_addr=None,
        service_type="_thread-test._udp",
        tcp_service_type="_thread-test._tcp,test_subtype",
        srv=None,
        weight="0",
        priority="4",
        AAAA=None,
        port=None,
        lease_option=(),
        enable_srv_key=False,
        enable_name_compression=False,
        host_addr=None,
        client_remove=False,
        is_tcp=False,
        omr=None,
        ml_eid=None,
        txt="",
    ):
        """Send Service Registration Protocol

        Args:
            dst_addr                =  Destination device address which srp_update to be sent,
                                       if not specified, srp_update is sent to BR by default.
            srv                     =  Service Description
                                       if not specified, Default would be None.
            AAAA                    =  Host Description
                                       if not specified, Default would be Ip4 or IPv6 address of the host.
            lease_option            =  Update Lease Option contains two lease time: Lease Time and Key Lease Time.

            enable_srv_key          =  If True contains the public key corresponding to the private key that was
                                       used to sign the message.

            enable_host_key         =  If True contains the public key corresponding to the private key that was used to
                                       sign the message.

            enable_name_compression =  If True send SRP update without name compression.

        Returns:
            True: DNS Update is syntactically and semantically valid
            False: DNS Update is not syntactically and semantically valid
        """

        # Implement-1.3-Thread
        # QUERY : What should be the port number?
        # Commands mentioned in the following :
        # https://github.com/openthread/openthread/blob/ca450e541636692909183773beba8f55591a7185/src/cli/README_SRP.md
        # https://github.com/openthread/openthread/blob/ca450e541636692909183773beba8f55591a7185/src/cli/README_SRP_CLIENT.md
        # srp client keyleaseinterval 864000
        # srp client leaseinterval 864000
        # https://github.com/openthread/opentread/blob/39a1f55447f3eec0cf089d8bec29b3303f3e7f99/src/cli/README.md
        # dns compression <enable/disable>
        # Implement-1.3-Thread
        lease = None
        keylease = None
        if len(lease_option) != 0:
            lease, keylease = lease_option
            # cmd = 'srp client autostart disable'
            # self.__executeCommand(cmd)
        if client_remove:
            cmd = "srp client host remove"
            self.__executeCommand(cmd)
        if port == "12348":
            cmd = "srp client host clear"
            print(cmd)
            self.__executeCommand(cmd)
        # cmd = 'srp client stop'
        # self.__executeCommand(cmd)
        if enable_srv_key != False:
            cmd = "srp client service key %s" % enable_srv_key
            self.__executeCommand(cmd)

        if host_addr == None:
            # mleid = ModuleHelper.GetFullIpv6Address(self.get_ed_omr_address()).lower()
            if ml_eid == None:
                cmd = "srp client host address %s" % (omr[0])
            elif omr == None:
                cmd = "srp client host address %s" % (ml_eid)
            else:
                cmd = "srp client host address %s %s" % (omr[0], ml_eid)
            self.__executeCommand(cmd)
        else:
            cmd = "srp client host address %s" % (host_addr)
            self.__executeCommand(cmd)

        if enable_name_compression != False:
            cmd = "dns compression %s" % enable_name_compression
            self.__executeCommand(cmd)

        if lease != None:
            cmd = "srp client leaseinterval %s" % (lease)
            self.__executeCommand(cmd)
        if keylease != None:
            cmd = "srp client keyleaseinterval %s" % (keylease)
            self.__executeCommand(cmd)

        cmd = "srp client host name %s" % (AAAA)
        print(cmd)
        self.__executeCommand(cmd)

        if is_tcp != False:
            cmd = "srp client service add %s %s %s %s %s %s" % (
                srv,
                tcp_service_type,
                port,
                weight,
                priority,
                txt,
            )
        else:
            cmd = "srp client service add %s %s %s %s %s %s" % (
                srv,
                service_type,
                port,
                weight,
                priority,
                txt,
            )
        self.__executeCommand(cmd)

        if dst_addr == None:
            cmd = "srp client autostart enable"
            self.__executeCommand(cmd)
        else:
            cmd_to_stop_srp = "srp client stop"
            self.__executeCommand(cmd_to_stop_srp)
            cmd1 = "dns config %s 53 6000 3 1" % dst_addr
            self.__executeCommand(cmd1)
            cmd = "srp client start %s %s" % (dst_addr, port)
            self.__executeCommand(cmd)

    def dns_query(self, addr="", service_type="_thread-test._udp"):
        """Send unicast DNS query

        Args:
            addr: IPv6 Address to send unicast DNS query, if `None` is passed, by default reference stacks are expected
            to send unicast to its parent address

        Returns:
            True: successfully sends unicast DNS query
            False: Fail to send the unicast DNS query
        """

        # Implement-1.3-Thread
        # dns resolve <addr>
        # getParentAddress()
        # https://github.com/openthread/openthread/blob/39a1f55447f3eec0cf089d8bec29b3303f3e7f99/src/cli/README.md#dns-config
        # Implement-1.3-Thread
        cmd = "dns browse %s.default.service.arpa %s" % (service_type, addr)
        self.__executeCommand(cmd)

    def dns_resolve(self, host_name):
        cmd = "dns resolve %s.default.service.arpa" % host_name
        self.__executeCommand(cmd)

    def dns_resolve4(self, host_name):
        cmd = "dns resolve4 %s.default.service.arpa" % host_name
        self.__executeCommand(cmd)

    def dns_resolve_ip(self, host_name, server_ip=""):
        cmd = "dns resolve %s %s" % (host_name, server_ip)
        self.__executeCommand(cmd)

    def dns_resolve_ipv4(self, host_name, server_ip="", ignore_exec=False):
        cmd = "dns resolve4 %s %s" % (host_name, server_ip)
        self.__executeCommand(cmd, ignore_exec=ignore_exec)

    def dns_browse(self, service_type="_thread-test1._udp", max_attempts="3"):
        cmd = "srp client autostart enable"
        self.__executeCommand(cmd)
        self.sleep(3)
        cmd1 = "dns config"
        dns_conf = self.__executeCommand(cmd1)
        self.sleep(3)
        print(dns_conf)
        dns_server_ip = re.findall(r"\[(.*?)\]", dns_conf[0])
        # dns_server_ip = dns_conf[0][9:41]
        cmd = "dns browse %s.default.service.arpa %s 53 6000 %s 0" % (
            service_type,
            dns_server_ip[0],
            max_attempts,
        )
        self.__executeCommand(cmd)

    def get_dns_config_info(self):
        cmd = "dns config"
        dns_conf = self.__executeCommand(cmd)
        print(dns_conf)

    def dns_config(
        self,
        server_ip="::",
        port="0",
        response_time="0",
        max_tx_attempts=0,
        recursion="1",
        service_mode="",
        transport_protocol="",
    ):
        cmd = "dns config %s %s %s %s %s %s %s" % (
            server_ip,
            port,
            response_time,
            max_tx_attempts,
            recursion,
            service_mode,
            transport_protocol,
        )
        #  Command has been updated as per the esko suggestion in TSTBED-358 and
        #  TSTBED-359
        self.__executeCommand(cmd)
        self.sleep(3)

    def child_timeout(self, _time):
        cmd = "childtimeout %s" % _time
        self.__executeCommand(cmd)

    def get_ip_port(self):
        cmd = "ipaddr mleid"
        server_ip = self.__executeCommand(cmd)
        ipaddr = ipaddress.ip_address(str((server_ip[0])))
        ipaddr = str(ipaddr.exploded)
        return ipaddr

    @API
    def mdns_query_instance(self, ip_addr, service_name):
        self.bash(
            ' timeout 2s dns-sd -P %s _thread-test._udp "" 53 host-test-1.default.service.arpa %s'
            % (service_name, ip_addr)
        )

    def dns_tcp_query_instance(self, service_name, service_type):
        cmd = "dns service %s %s.default.service.arpa" % (
            service_name,
            service_type,
        )
        print(cmd)
        self.__executeCommand(cmd)

    def dns_query_instance(
        self, ip_addr, service_name, service_type="_thread-test._udp"
    ):
        cmd = "dns service %s %s.default.service.arpa %s 53 6000 3 1" % (
            service_name,
            service_type,
            ip_addr,
        )
        self.__executeCommand(cmd)

    def setMliid(self, sMlIid):
        self.__setMlIid(sMlIid)

    def set_prefix_meshlocal(self, prefix):
        cmd = "prefix meshlocal %s" % prefix
        self.__executeCommand(cmd)

    def srp_host_remove(self):
        cmd = "srp client host remove"
        self.__executeCommand(cmd)

    def add_new_service(self):
        rloc = self.__executeCommand("rloc16")[0]
        mleid = self.__executeCommand("ipaddr mleid")[0]
        mleid = ModuleHelper.GetFullIpv6Address(mleid)
        mleid = mleid.replace(":", "")
        cmd = "service add 44970 5d %sd11f" % (mleid)
        self.__executeCommand(cmd)
        self.__executeCommand("netdata register")
        time.sleep(3)

    def remove_prefix_network_data(self, prefix):
        cmd = "prefix remove %s" % prefix
        self.__executeCommand(cmd)
        cmd1 = "netdata register"
        self.__executeCommand(cmd1)
        time.sleep(3)

    def republish_prefix(self):
        cmd = "br omrprefix"
        omrprefix = self.__executeCommand(cmd)
        omrprefix = omrprefix[0].split("Local: ")[1]

        cmd1 = "prefix remove %s" % omrprefix
        self.__executeCommand(cmd1)

        cmd2 = "netdata register"
        self.__executeCommand(cmd2)
        time.sleep(3)

        cmd4 = "prefix add %s paros med " % omrprefix
        self.__executeCommand(cmd4)

        cmd5 = "netdata register"
        self.__executeCommand(cmd5)
        time.sleep(3)

    def config_prefix(self):
        cmd = "br omrprefix"
        omrprefix = self.__executeCommand(cmd)
        omrprefix = omrprefix[0].split("Local: ")[1]

        cmd1 = "prefix remove %s" % omrprefix
        self.__executeCommand(cmd1)

        cmd2 = "netdata register"
        self.__executeCommand(cmd2)
        time.sleep(3)
        omrprefix = omrprefix[:-3]
        omrprefix = omrprefix + "/64"
        cmd4 = "prefix add %s paos low" % (omrprefix)
        self.__executeCommand(cmd4)
        cmd5 = "netdata register"
        self.__executeCommand(cmd5)

    def startDTLSSession(self, addr=""):
        cmd = "coaps start"
        self.__executeCommand(cmd)
        cmd = "coaps help"
        self.__executeCommand(cmd)
        # cmd = f"coaps connect {addr}"
        # self.__executeCommand(cmd)

    @API
    def stopDTLSSession(self):
        cmd = "coaps disconnect"
        self.__executeCommand(cmd)

    # @API
    # def set_ePSKC(self, key):
    #     cmd = "ba ephemeralkey"
    #     self.__executeCommand(cmd)
    #     cmd = f"ba ephemeralkey set Z10X20g3J15w1000P60m16"
    #     self.__executeCommand(cmd)
    #     time.sleep(15)

    @API
    def add_service(self, service, server):
        self.__executeCommand("service add 44970 %s %s" % (service, server))
        self.__executeCommand("netdata register")

    ########################################################################################################################
    #                                       1.3 THCI Commands
    #
    ########################################################################################################################


class nRF_Connect_SDK_13_14(OpenThreadTHCI, IThci):
    def _connect(self):
        print(("My port is %s" % self))
        self.__lines = []
        timeout = 10
        port_error = None

        if self.port.startswith("COM"):
            for _ in range(int(timeout / 0.5)):
                time.sleep(0.5)
                try:
                    self.__handle = serial.Serial(
                        self.port, 115200, timeout=0, write_timeout=1
                    )
                    self.sleep(1)
                    buffer_clear_count = 10
                    for _ in range(buffer_clear_count):
                        self.__handle.write(bytes("\r\n", "utf-8"))
                        self.sleep(0.1)
                    self._is_net = False
                    break
                except SerialException as port_error:
                    self.log(
                        "{} port not ready, retrying to connect...".format(self.port)
                    )
            else:
                raise SerialException(
                    "Could not open {} port: {}".format(self.port, port_error)
                )
        elif ":" in self.port:
            host, port = self.port.split(":")
            self.__handle = socket.create_connection((host, port))
            self.__handle.setblocking(False)
            self._is_net = True
        else:
            raise Exception("Unknown port schema")

    def _disconnect(self):
        if self.__handle:
            self.__handle.close()
            self.__handle = None

    def _deviceBeforeReset(self):
        pass

    def _deviceAfterReset(self):
        pass

    def __restartAgentService(self):
        pass

    def _beforeRegisterMulticast(self, sAddr, timeout):
        pass

    def __socRead(self, size=512):
        if self._is_net:
            return self.__handle.recv(size)
        else:
            return self.__handle.read(size)

    def __socWrite(self, data):
        if self._is_net:
            self.__handle.sendall(data)
        else:
            self.__handle.write(bytes(data, "utf-8"))

    def _cliReadLine(self):
        if len(self.__lines) > 1:
            return self.__lines.pop(0)

        tail = ""
        if len(self.__lines) != 0:
            tail = self.__lines.pop()

        try:
            tail += str(self.__socRead(), "utf-8")
        except socket.error:
            logging.exception("%s: No new data", self)
            self.sleep(0.1)

        self.__lines += self._lineSepX.split(tail)
        if len(self.__lines) > 1:
            return self.__lines.pop(0)

    def _cliWriteLine(self, line):
        if self._cmdPrefix == ZEPHYR_PREFIX:
            if not line.startswith(self._cmdPrefix):
                line = self._cmdPrefix + line
            self.__socWrite(line + "\r")
        else:
            self.__socWrite(line + "\r\n")
