/*
 *
 *    Copyright (c) 2022 Project CHIP Authors
 *
 *    Licensed under the Apache License, Version 2.0 (the "License");
 *    you may not use this file except in compliance with the License.
 *    You may obtain a copy of the License at
 *
 *        http://www.apache.org/licenses/LICENSE-2.0
 *
 *    Unless required by applicable law or agreed to in writing, software
 *    distributed under the License is distributed on an "AS IS" BASIS,
 *    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *    See the License for the specific language governing permissions and
 *    limitations under the License.
 */

// THIS FILE IS GENERATED BY ZAP

#pragma once

#include <app/util/basic-types.h>

namespace chip
{
namespace app
{
	namespace Clusters
	{

		namespace AccessControl
		{
			namespace Events
			{

				namespace AccessControlEntryChanged
				{
					static constexpr EventId Id = 0x00000000;
				} // namespace AccessControlEntryChanged

				namespace AccessControlExtensionChanged
				{
					static constexpr EventId Id = 0x00000001;
				} // namespace AccessControlExtensionChanged

				namespace FabricRestrictionReviewUpdate
				{
					static constexpr EventId Id = 0x00000002;
				} // namespace FabricRestrictionReviewUpdate

			} // namespace Events
		} // namespace AccessControl

		namespace Actions
		{
			namespace Events
			{

				namespace StateChanged
				{
					static constexpr EventId Id = 0x00000000;
				} // namespace StateChanged

				namespace ActionFailed
				{
					static constexpr EventId Id = 0x00000001;
				} // namespace ActionFailed

			} // namespace Events
		} // namespace Actions

		namespace BasicInformation
		{
			namespace Events
			{

				namespace StartUp
				{
					static constexpr EventId Id = 0x00000000;
				} // namespace StartUp

				namespace ShutDown
				{
					static constexpr EventId Id = 0x00000001;
				} // namespace ShutDown

				namespace Leave
				{
					static constexpr EventId Id = 0x00000002;
				} // namespace Leave

				namespace ReachableChanged
				{
					static constexpr EventId Id = 0x00000003;
				} // namespace ReachableChanged

				namespace RandomNumberChanged
				{
					static constexpr EventId Id = 0x00000004;
				} // namespace RandomNumberChanged

			} // namespace Events
		} // namespace BasicInformation

		namespace OtaSoftwareUpdateRequestor
		{
			namespace Events
			{

				namespace StateTransition
				{
					static constexpr EventId Id = 0x00000000;
				} // namespace StateTransition

				namespace VersionApplied
				{
					static constexpr EventId Id = 0x00000001;
				} // namespace VersionApplied

				namespace DownloadError
				{
					static constexpr EventId Id = 0x00000002;
				} // namespace DownloadError

			} // namespace Events
		} // namespace OtaSoftwareUpdateRequestor

		namespace PowerSource
		{
			namespace Events
			{

				namespace WiredFaultChange
				{
					static constexpr EventId Id = 0x00000000;
				} // namespace WiredFaultChange

				namespace BatFaultChange
				{
					static constexpr EventId Id = 0x00000001;
				} // namespace BatFaultChange

				namespace BatChargeFaultChange
				{
					static constexpr EventId Id = 0x00000002;
				} // namespace BatChargeFaultChange

			} // namespace Events
		} // namespace PowerSource

		namespace GeneralDiagnostics
		{
			namespace Events
			{

				namespace HardwareFaultChange
				{
					static constexpr EventId Id = 0x00000000;
				} // namespace HardwareFaultChange

				namespace RadioFaultChange
				{
					static constexpr EventId Id = 0x00000001;
				} // namespace RadioFaultChange

				namespace NetworkFaultChange
				{
					static constexpr EventId Id = 0x00000002;
				} // namespace NetworkFaultChange

				namespace BootReason
				{
					static constexpr EventId Id = 0x00000003;
				} // namespace BootReason

			} // namespace Events
		} // namespace GeneralDiagnostics

		namespace SoftwareDiagnostics
		{
			namespace Events
			{

				namespace SoftwareFault
				{
					static constexpr EventId Id = 0x00000000;
				} // namespace SoftwareFault

			} // namespace Events
		} // namespace SoftwareDiagnostics

		namespace ThreadNetworkDiagnostics
		{
			namespace Events
			{

				namespace ConnectionStatus
				{
					static constexpr EventId Id = 0x00000000;
				} // namespace ConnectionStatus

				namespace NetworkFaultChange
				{
					static constexpr EventId Id = 0x00000001;
				} // namespace NetworkFaultChange

			} // namespace Events
		} // namespace ThreadNetworkDiagnostics

		namespace WiFiNetworkDiagnostics
		{
			namespace Events
			{

				namespace Disconnection
				{
					static constexpr EventId Id = 0x00000000;
				} // namespace Disconnection

				namespace AssociationFailure
				{
					static constexpr EventId Id = 0x00000001;
				} // namespace AssociationFailure

				namespace ConnectionStatus
				{
					static constexpr EventId Id = 0x00000002;
				} // namespace ConnectionStatus

			} // namespace Events
		} // namespace WiFiNetworkDiagnostics

		namespace TimeSynchronization
		{
			namespace Events
			{

				namespace DSTTableEmpty
				{
					static constexpr EventId Id = 0x00000000;
				} // namespace DSTTableEmpty

				namespace DSTStatus
				{
					static constexpr EventId Id = 0x00000001;
				} // namespace DSTStatus

				namespace TimeZoneStatus
				{
					static constexpr EventId Id = 0x00000002;
				} // namespace TimeZoneStatus

				namespace TimeFailure
				{
					static constexpr EventId Id = 0x00000003;
				} // namespace TimeFailure

				namespace MissingTrustedTimeSource
				{
					static constexpr EventId Id = 0x00000004;
				} // namespace MissingTrustedTimeSource

			} // namespace Events
		} // namespace TimeSynchronization

		namespace BridgedDeviceBasicInformation
		{
			namespace Events
			{

				namespace StartUp
				{
					static constexpr EventId Id = 0x00000000;
				} // namespace StartUp

				namespace ShutDown
				{
					static constexpr EventId Id = 0x00000001;
				} // namespace ShutDown

				namespace Leave
				{
					static constexpr EventId Id = 0x00000002;
				} // namespace Leave

				namespace ReachableChanged
				{
					static constexpr EventId Id = 0x00000003;
				} // namespace ReachableChanged

				namespace ActiveChanged
				{
					static constexpr EventId Id = 0x00000080;
				} // namespace ActiveChanged

			} // namespace Events
		} // namespace BridgedDeviceBasicInformation

		namespace Switch
		{
			namespace Events
			{

				namespace SwitchLatched
				{
					static constexpr EventId Id = 0x00000000;
				} // namespace SwitchLatched

				namespace InitialPress
				{
					static constexpr EventId Id = 0x00000001;
				} // namespace InitialPress

				namespace LongPress
				{
					static constexpr EventId Id = 0x00000002;
				} // namespace LongPress

				namespace ShortRelease
				{
					static constexpr EventId Id = 0x00000003;
				} // namespace ShortRelease

				namespace LongRelease
				{
					static constexpr EventId Id = 0x00000004;
				} // namespace LongRelease

				namespace MultiPressOngoing
				{
					static constexpr EventId Id = 0x00000005;
				} // namespace MultiPressOngoing

				namespace MultiPressComplete
				{
					static constexpr EventId Id = 0x00000006;
				} // namespace MultiPressComplete

			} // namespace Events
		} // namespace Switch

		namespace BooleanState
		{
			namespace Events
			{

				namespace StateChange
				{
					static constexpr EventId Id = 0x00000000;
				} // namespace StateChange

			} // namespace Events
		} // namespace BooleanState

		namespace OvenCavityOperationalState
		{
			namespace Events
			{

				namespace OperationalError
				{
					static constexpr EventId Id = 0x00000000;
				} // namespace OperationalError

				namespace OperationCompletion
				{
					static constexpr EventId Id = 0x00000001;
				} // namespace OperationCompletion

			} // namespace Events
		} // namespace OvenCavityOperationalState

		namespace RefrigeratorAlarm
		{
			namespace Events
			{

				namespace Notify
				{
					static constexpr EventId Id = 0x00000000;
				} // namespace Notify

			} // namespace Events
		} // namespace RefrigeratorAlarm

		namespace SmokeCoAlarm
		{
			namespace Events
			{

				namespace SmokeAlarm
				{
					static constexpr EventId Id = 0x00000000;
				} // namespace SmokeAlarm

				namespace COAlarm
				{
					static constexpr EventId Id = 0x00000001;
				} // namespace COAlarm

				namespace LowBattery
				{
					static constexpr EventId Id = 0x00000002;
				} // namespace LowBattery

				namespace HardwareFault
				{
					static constexpr EventId Id = 0x00000003;
				} // namespace HardwareFault

				namespace EndOfService
				{
					static constexpr EventId Id = 0x00000004;
				} // namespace EndOfService

				namespace SelfTestComplete
				{
					static constexpr EventId Id = 0x00000005;
				} // namespace SelfTestComplete

				namespace AlarmMuted
				{
					static constexpr EventId Id = 0x00000006;
				} // namespace AlarmMuted

				namespace MuteEnded
				{
					static constexpr EventId Id = 0x00000007;
				} // namespace MuteEnded

				namespace InterconnectSmokeAlarm
				{
					static constexpr EventId Id = 0x00000008;
				} // namespace InterconnectSmokeAlarm

				namespace InterconnectCOAlarm
				{
					static constexpr EventId Id = 0x00000009;
				} // namespace InterconnectCOAlarm

				namespace AllClear
				{
					static constexpr EventId Id = 0x0000000A;
				} // namespace AllClear

			} // namespace Events
		} // namespace SmokeCoAlarm

		namespace DishwasherAlarm
		{
			namespace Events
			{

				namespace Notify
				{
					static constexpr EventId Id = 0x00000000;
				} // namespace Notify

			} // namespace Events
		} // namespace DishwasherAlarm

		namespace OperationalState
		{
			namespace Events
			{

				namespace OperationalError
				{
					static constexpr EventId Id = 0x00000000;
				} // namespace OperationalError

				namespace OperationCompletion
				{
					static constexpr EventId Id = 0x00000001;
				} // namespace OperationCompletion

			} // namespace Events
		} // namespace OperationalState

		namespace RvcOperationalState
		{
			namespace Events
			{

				namespace OperationalError
				{
					static constexpr EventId Id = 0x00000000;
				} // namespace OperationalError

				namespace OperationCompletion
				{
					static constexpr EventId Id = 0x00000001;
				} // namespace OperationCompletion

			} // namespace Events
		} // namespace RvcOperationalState

		namespace BooleanStateConfiguration
		{
			namespace Events
			{

				namespace AlarmsStateChanged
				{
					static constexpr EventId Id = 0x00000000;
				} // namespace AlarmsStateChanged

				namespace SensorFault
				{
					static constexpr EventId Id = 0x00000001;
				} // namespace SensorFault

			} // namespace Events
		} // namespace BooleanStateConfiguration

		namespace ValveConfigurationAndControl
		{
			namespace Events
			{

				namespace ValveStateChanged
				{
					static constexpr EventId Id = 0x00000000;
				} // namespace ValveStateChanged

				namespace ValveFault
				{
					static constexpr EventId Id = 0x00000001;
				} // namespace ValveFault

			} // namespace Events
		} // namespace ValveConfigurationAndControl

		namespace ElectricalPowerMeasurement
		{
			namespace Events
			{

				namespace MeasurementPeriodRanges
				{
					static constexpr EventId Id = 0x00000000;
				} // namespace MeasurementPeriodRanges

			} // namespace Events
		} // namespace ElectricalPowerMeasurement

		namespace ElectricalEnergyMeasurement
		{
			namespace Events
			{

				namespace CumulativeEnergyMeasured
				{
					static constexpr EventId Id = 0x00000000;
				} // namespace CumulativeEnergyMeasured

				namespace PeriodicEnergyMeasured
				{
					static constexpr EventId Id = 0x00000001;
				} // namespace PeriodicEnergyMeasured

			} // namespace Events
		} // namespace ElectricalEnergyMeasurement

		namespace WaterHeaterManagement
		{
			namespace Events
			{

				namespace BoostStarted
				{
					static constexpr EventId Id = 0x00000000;
				} // namespace BoostStarted

				namespace BoostEnded
				{
					static constexpr EventId Id = 0x00000001;
				} // namespace BoostEnded

			} // namespace Events
		} // namespace WaterHeaterManagement

		namespace DemandResponseLoadControl
		{
			namespace Events
			{

				namespace LoadControlEventStatusChange
				{
					static constexpr EventId Id = 0x00000000;
				} // namespace LoadControlEventStatusChange

			} // namespace Events
		} // namespace DemandResponseLoadControl

		namespace Messages
		{
			namespace Events
			{

				namespace MessageQueued
				{
					static constexpr EventId Id = 0x00000000;
				} // namespace MessageQueued

				namespace MessagePresented
				{
					static constexpr EventId Id = 0x00000001;
				} // namespace MessagePresented

				namespace MessageComplete
				{
					static constexpr EventId Id = 0x00000002;
				} // namespace MessageComplete

			} // namespace Events
		} // namespace Messages

		namespace DeviceEnergyManagement
		{
			namespace Events
			{

				namespace PowerAdjustStart
				{
					static constexpr EventId Id = 0x00000000;
				} // namespace PowerAdjustStart

				namespace PowerAdjustEnd
				{
					static constexpr EventId Id = 0x00000001;
				} // namespace PowerAdjustEnd

				namespace Paused
				{
					static constexpr EventId Id = 0x00000002;
				} // namespace Paused

				namespace Resumed
				{
					static constexpr EventId Id = 0x00000003;
				} // namespace Resumed

			} // namespace Events
		} // namespace DeviceEnergyManagement

		namespace EnergyEvse
		{
			namespace Events
			{

				namespace EVConnected
				{
					static constexpr EventId Id = 0x00000000;
				} // namespace EVConnected

				namespace EVNotDetected
				{
					static constexpr EventId Id = 0x00000001;
				} // namespace EVNotDetected

				namespace EnergyTransferStarted
				{
					static constexpr EventId Id = 0x00000002;
				} // namespace EnergyTransferStarted

				namespace EnergyTransferStopped
				{
					static constexpr EventId Id = 0x00000003;
				} // namespace EnergyTransferStopped

				namespace Fault
				{
					static constexpr EventId Id = 0x00000004;
				} // namespace Fault

				namespace Rfid
				{
					static constexpr EventId Id = 0x00000005;
				} // namespace Rfid

			} // namespace Events
		} // namespace EnergyEvse

		namespace DoorLock
		{
			namespace Events
			{

				namespace DoorLockAlarm
				{
					static constexpr EventId Id = 0x00000000;
				} // namespace DoorLockAlarm

				namespace DoorStateChange
				{
					static constexpr EventId Id = 0x00000001;
				} // namespace DoorStateChange

				namespace LockOperation
				{
					static constexpr EventId Id = 0x00000002;
				} // namespace LockOperation

				namespace LockOperationError
				{
					static constexpr EventId Id = 0x00000003;
				} // namespace LockOperationError

				namespace LockUserChange
				{
					static constexpr EventId Id = 0x00000004;
				} // namespace LockUserChange

			} // namespace Events
		} // namespace DoorLock

		namespace PumpConfigurationAndControl
		{
			namespace Events
			{

				namespace SupplyVoltageLow
				{
					static constexpr EventId Id = 0x00000000;
				} // namespace SupplyVoltageLow

				namespace SupplyVoltageHigh
				{
					static constexpr EventId Id = 0x00000001;
				} // namespace SupplyVoltageHigh

				namespace PowerMissingPhase
				{
					static constexpr EventId Id = 0x00000002;
				} // namespace PowerMissingPhase

				namespace SystemPressureLow
				{
					static constexpr EventId Id = 0x00000003;
				} // namespace SystemPressureLow

				namespace SystemPressureHigh
				{
					static constexpr EventId Id = 0x00000004;
				} // namespace SystemPressureHigh

				namespace DryRunning
				{
					static constexpr EventId Id = 0x00000005;
				} // namespace DryRunning

				namespace MotorTemperatureHigh
				{
					static constexpr EventId Id = 0x00000006;
				} // namespace MotorTemperatureHigh

				namespace PumpMotorFatalFailure
				{
					static constexpr EventId Id = 0x00000007;
				} // namespace PumpMotorFatalFailure

				namespace ElectronicTemperatureHigh
				{
					static constexpr EventId Id = 0x00000008;
				} // namespace ElectronicTemperatureHigh

				namespace PumpBlocked
				{
					static constexpr EventId Id = 0x00000009;
				} // namespace PumpBlocked

				namespace SensorFailure
				{
					static constexpr EventId Id = 0x0000000A;
				} // namespace SensorFailure

				namespace ElectronicNonFatalFailure
				{
					static constexpr EventId Id = 0x0000000B;
				} // namespace ElectronicNonFatalFailure

				namespace ElectronicFatalFailure
				{
					static constexpr EventId Id = 0x0000000C;
				} // namespace ElectronicFatalFailure

				namespace GeneralFault
				{
					static constexpr EventId Id = 0x0000000D;
				} // namespace GeneralFault

				namespace Leakage
				{
					static constexpr EventId Id = 0x0000000E;
				} // namespace Leakage

				namespace AirDetection
				{
					static constexpr EventId Id = 0x0000000F;
				} // namespace AirDetection

				namespace TurbineOperation
				{
					static constexpr EventId Id = 0x00000010;
				} // namespace TurbineOperation

			} // namespace Events
		} // namespace PumpConfigurationAndControl

		namespace OccupancySensing
		{
			namespace Events
			{

				namespace OccupancyChanged
				{
					static constexpr EventId Id = 0x00000000;
				} // namespace OccupancyChanged

			} // namespace Events
		} // namespace OccupancySensing

		namespace TargetNavigator
		{
			namespace Events
			{

				namespace TargetUpdated
				{
					static constexpr EventId Id = 0x00000000;
				} // namespace TargetUpdated

			} // namespace Events
		} // namespace TargetNavigator

		namespace MediaPlayback
		{
			namespace Events
			{

				namespace StateChanged
				{
					static constexpr EventId Id = 0x00000000;
				} // namespace StateChanged

			} // namespace Events
		} // namespace MediaPlayback

		namespace AccountLogin
		{
			namespace Events
			{

				namespace LoggedOut
				{
					static constexpr EventId Id = 0x00000000;
				} // namespace LoggedOut

			} // namespace Events
		} // namespace AccountLogin

		namespace ContentControl
		{
			namespace Events
			{

				namespace RemainingScreenTimeExpired
				{
					static constexpr EventId Id = 0x00000000;
				} // namespace RemainingScreenTimeExpired

			} // namespace Events
		} // namespace ContentControl

		namespace CommissionerControl
		{
			namespace Events
			{

				namespace CommissioningRequestResult
				{
					static constexpr EventId Id = 0x00000000;
				} // namespace CommissioningRequestResult

			} // namespace Events
		} // namespace CommissionerControl

		namespace NordicDevKit
		{
			namespace Events
			{

				namespace UserButtonChanged
				{
					static constexpr EventId Id = 0xFFF10000;
				} // namespace UserButtonChanged

			} // namespace Events
		} // namespace NordicDevKit

		namespace UnitTesting
		{
			namespace Events
			{

				namespace TestEvent
				{
					static constexpr EventId Id = 0x00000001;
				} // namespace TestEvent

				namespace TestFabricScopedEvent
				{
					static constexpr EventId Id = 0x00000002;
				} // namespace TestFabricScopedEvent

				namespace TestDifferentVendorMeiEvent
				{
					static constexpr EventId Id = 0xFFF200EE;
				} // namespace TestDifferentVendorMeiEvent

			} // namespace Events
		} // namespace UnitTesting

		namespace SampleMei
		{
			namespace Events
			{

				namespace PingCountEvent
				{
					static constexpr EventId Id = 0x00000000;
				} // namespace PingCountEvent

			} // namespace Events
		} // namespace SampleMei

	} // namespace Clusters
} // namespace app
} // namespace chip
