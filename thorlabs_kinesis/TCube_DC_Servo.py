# -*- coding: utf-8 -*-
"""
Created on Mon Mar 30 17:47:22 2020

@author: Faure Nicolas

# Bindings for TDC100

"""

from ctypes import (
    Structure,
    cdll,
    c_bool,
    c_short,
    c_int,
    c_uint,
    c_int16,
    c_int32,
    c_char,
    c_byte,
    c_long,
    c_float,
    c_double,
    POINTER,
    CFUNCTYPE,
)

from thorlabs_kinesis._utils import (
    c_word,
    c_dword,
    bind
)


import os
os.environ['PATH'] = 'C:\Program Files\Thorlabs\Kinesis' + os.pathsep + os.environ['PATH']

lib = cdll.LoadLibrary("Thorlabs.MotionControl.TCube.DCServo.dll")

# enum FT_Status
FT_OK = c_short(0x00)
FT_InvalidHandle = c_short(0x01)
FT_DeviceNotFound = c_short(0x02)
FT_DeviceNotOpened = c_short(0x03)
FT_IOError = c_short(0x04)
FT_InsufficientResources = c_short(0x05)
FT_InvalidParameter = c_short(0x06)
FT_DeviceNotPresent = c_short(0x07)
FT_IncorrectDevice = c_short(0x08)
FT_Status = c_short

# enum MOT_MotorTypes
MOT_NotMotor = c_int(0)
MOT_DCMotor = c_int(1)
MOT_StepperMotor = c_int(2)
MOT_BrushlessMotor = c_int(3)
MOT_CustomMotor = c_int(100)
MOT_MotorTypes = c_int

# enum MOT_TravelModes
MOT_TravelModeUndefined = c_int(0x00)
MOT_Linear = c_int(0x01)
MOT_Rotational = c_int(0x02)
MOT_TravelModes = c_int

# enum MOT_TravelDirection
MOT_TravelDirectionUndefined = c_short(0x00)
MOT_Forwards = c_short(0x01)
MOT_Backwards = c_short(0x02)
MOT_TravelDirection = c_short

# enum MOT_HomeLimitSwitchDirection
MOT_LimitSwitchDirectionUndefined = c_short(0x00)
MOT_ReverseLimitSwitch = c_short(0x01)
MOT_ForwardLimitSwitch = c_short(0x04)
MOT_HomeLimitSwitchDirection = c_short

# enum MOT_DirectionSense
MOT_Normal = c_short(0x00)
MOT_Reverse = c_short(0x01)
MOT_DirectionSense = c_short

# enum MOT_JogModes
MOT_JogModeUndefined = c_short(0x00)
MOT_Continuous = c_short(0x01)
MOT_SingleStep = c_short(0x02)
MOT_JogModes = c_short

# enum MOT_StopModes
MOT_StopModeUndefined = c_short(0x00)
MOT_Immediate = c_short(0x01)
MOT_Profiled = c_short(0x02)
MOT_StopModes = c_short

# enum MOT_ButtonsModes
MOT_ButtonModeUndefined = c_word(0x00)
MOT_JogMode = c_word(0x01)
MOT_Preset = c_word(0x02)
MOT_ButtonModes = c_word

# enum MOT_LimitSwitchModes
MOT_LimitSwitchModeUndefined = c_word(0x00)
MOT_LimitSwitchIgnoreSwitch = c_word(0x01)
MOT_LimitSwitchMakeOnContact = c_word(0x02)
MOT_LimitSwitchBreakOnContact = c_word(0x03)
MOT_LimitSwitchMakeOnHome = c_word(0x04)
MOT_LimitSwitchBreakOnHome = c_word(0x05)
MOT_PMD_Reserved = c_word(0x06)
MOT_LimitSwitchIgnoreSwitchSwapped = c_word(0x81)
MOT_LimitSwitchMakeOnContactSwapped = c_word(0x82)
MOT_LimitSwitchBreakOnContactSwapped = c_word(0x83)
MOT_LimitSwitchMakeOnHomeSwapped = c_word(0x84)
MOT_LimitSwitchBreakOnHomeSwapped = c_word(0x85)
MOT_LimitSwitchModes = c_word

# enum MOT_LimitSwitchSWModes
MOT_LimitSwitchSWModeUndefined = c_word(0x00)
MOT_LimitSwitchIgnored = c_word(0x01)
MOT_LimitSwitchStopImmediate = c_word(0x02)
MOT_LimitSwitchStopProfiled = c_word(0x03)
MOT_LimitSwitchIgnored_Rotational = c_word(0x81)
MOT_LimitSwitchStopImmediate_Rotational = c_word(0x82)
MOT_LimitSwitchStopProfiled_Rotational = c_word(0x83)
MOT_LimitSwitchSWModes = c_word

# enum MOT_LimitsSoftwareApproachPolicy
DisallowIllegalMoves = c_int16(0)
AllowPartialMoves = c_int16(1)
AllowAllMoves = c_int16(2)
MOT_LimitsSoftwareApproachPolicy = c_int16


# enum MOT_MovementModes ?c_word?
LinearRange = c_word(0x00)
RotationalUnlimited = c_word(0x01)
RotationalWrapping = c_word(0x02)
MOT_MovementModes = c_word

# enum MOT_MovementDirections ?c_word?
Quickest = c_word(0x00)
Forwards = c_word(0x01)
Reverse = c_word(0x02)
MOT_MovementDirections = c_word

# enum KMOT_WheelDirectionSense
KMOT_WM_Positive = c_int16(0x01)
KMOT_WM_Negative = c_int16(0x02)
KMOT_WheelDirectionSense = c_int16

# enum KMOT_WheelMode
KMOT_WM_Velocity = c_int16(0x01)
KMOT_WM_Jog = c_int16(0x02)
KMOT_WM_MoveAbsolute = c_int16(0x03)
KMOT_WheelMode = c_int16

# enum KMOT_TriggerPortMode
KMOT_TrigDisabled = c_int16(0x00)            # Trigger Disabled
KMOT_TrigIn_GPI = c_int16(0x01)              # General purpose logic input (<see cref="CC_GetStatusBits(const char * serialNo)"> GetStatusBits</see>)
KMOT_TrigIn_RelativeMove = c_int16(0x02)     # Move relative using relative move parameters
KMOT_TrigIn_AbsoluteMove = c_int16(0x03)     # Move absolute using absolute move parameters
KMOT_TrigIn_Home = c_int16(0x04)             # Perform a Home action
KMOT_TrigIn_Stop = c_int16(0x05)             # Perform a Stop Immediate action
KMOT_TrigOut_GPO = c_int16(0x0A)             # General purpose output (<see cref="CC_SetDigitalOutputs(const char * serialNo, byte outputBits)"> SetDigitalOutputs</see>)
KMOT_TrigOut_InMotion = c_int16(0x0B)        # Set when device moving
KMOT_TrigOut_AtMaxVelocity = c_int16(0x0C)   # Set when at max velocity
KMOT_TrigOut_AtPositionSteps = c_int16(0x0D) # Set when at predefine position steps,<br />Set using wTrigStartPos, wTrigInterval, wTrigNumPulses,wTrigPulseWidth
KMOT_TrigOut_Synch = c_int16(0x0E)           # TBD ?
KMOT_TriggerPortMode = c_int16

# enum KMOT_TriggerPortPolarity
KMOT_TrigPolarityHigh = c_int16(0x01)        # Trigger Polarity high
KMOT_TrigPolarityLow = c_int16(0x02)         # Trigger Polarity Low
KMOT_TriggerPortPolarity = c_int16

# enum MOT_MovementModes
LinearRange = c_int(0x00)
RotationalUnlimited = c_int(0x01)
RotationalWrapping = c_int(0x02)
MOT_MovementModes = c_int

# enum MOT_MovementDirections
Quickest = c_int(0x00)
Forwards = c_int(0x01)
Reverse = c_int(0x02)
MOT_MovementDirections = c_int



class TLI_DeviceInfo(Structure):
    _fields_ = [("typeID", c_dword),
                ("description", (65 * c_char)),
                ("serialNo", (9 * c_char)),
                ("PID", c_dword),
                ("isKnownType", c_bool),
                ("motorType", MOT_MotorTypes),
                ("isPiezoDevice", c_bool),
                ("isLaser", c_bool),
                ("isCustomType", c_bool),
                ("isRack", c_bool),
                ("maxChannels", c_short)]

class TLI_HardwareInformation(Structure):
    _fields_ = [("serialNumber", c_dword),
                ("modelNumber", (8 * c_char)),
                ("type", c_word),
                ("firmwareVersion", c_dword),
                ("notes", (48 * c_char)),
                ("deviceDependantData", (12 * c_byte)),
                ("hardwareVersion", c_word),
                ("modificationState", c_word),
                ("numChannels", c_short)]

class MOT_VelocityParameters(Structure):
    _fields_ = [("minVelocity", c_int),
                ("acceleration", c_int),
                ("maxVelocity", c_int)]


class MOT_JogParameters(Structure):
    _fields_ = [("mode", MOT_JogModes),
                ("stepSize", c_uint),
                ("velParams", MOT_VelocityParameters),
                ("stopMode", MOT_StopModes)]


class MOT_HomingParameters(Structure):
    _fields_ = [("direction", MOT_TravelDirection),
                ("limitSwitch", MOT_HomeLimitSwitchDirection),
                ("velocity", c_uint),
                ("offsetDistance", c_uint)]


class MOT_LimitSwitchParameters(Structure):
    _fields_ = [("clockwiseHardwareLimit", MOT_LimitSwitchModes),
                ("anticlockwiseHardwareLimit", MOT_LimitSwitchModes),
                ("clockwisePosition", c_dword),
                ("anticlockwisePosition", c_dword),
                ("softLimitMode", MOT_LimitSwitchSWModes)]


class MOT_ButtonParameters(Structure):
    _fields_ = [("buttonMode", MOT_ButtonModes),
                ("leftButtonPosition", c_int),
                ("rightButtonPosition", c_int),
                ("timeout", c_word),
                ("unused", c_word)]


class MOT_PotentiometerStep(Structure):
    _fields_ = [("thresholdDeflection", c_word),
                ("velocity", c_dword)]


class MOT_PotentiometerSteps(Structure):
    _fields_ = [("potentiometerStepParameters", (4 * MOT_PotentiometerStep) ) ]



class MOT_DC_PIDParameters(Structure):
    _fields_ = [("proportionalGain", c_int),
                ("integralGain", c_int),
                ("differentialGain", c_int),
                ("integralLimit", c_int),
                ("parameterFilter", c_word)]



TLI_BuildDeviceList = bind(lib, "TLI_BuildDeviceList", None, c_short)
TLI_GetDeviceListSize = bind(lib, "TLI_GetDeviceListSize", None, c_short)
# TLI_GetDeviceList  <- TODO: Implement SAFEARRAY first. KCUBEDCSERVO_API short __cdecl TLI_GetDeviceList(SAFEARRAY** stringsReceiver);
# TLI_GetDeviceListByType  <- TODO: Implement SAFEARRAY first. KCUBEDCSERVO_API short __cdecl TLI_GetDeviceListByType(SAFEARRAY** stringsReceiver, int typeID);
# TLI_GetDeviceListByTypes  <- TODO: Implement SAFEARRAY first. KCUBEDCSERVO_API short __cdecl TLI_GetDeviceListByTypes(SAFEARRAY** stringsReceiver, int * typeIDs, int length);
TLI_GetDeviceListExt = bind(lib, "TLI_GetDeviceListExt", [POINTER(c_char), c_dword], c_short)
TLI_GetDeviceListByTypeExt = bind(lib, "TLI_GetDeviceListByTypeExt", [POINTER(c_char), c_dword, c_int], c_short)
TLI_GetDeviceListByTypesExt = bind(lib, "TLI_GetDeviceListByTypesExt", [POINTER(c_char), c_dword, POINTER(c_int), c_int], c_short)
TLI_GetDeviceInfo = bind(lib, "TLI_GetDeviceInfo", [POINTER(c_char), POINTER(TLI_DeviceInfo)], c_short)

TLI_InitializeSimulations = bind(lib, "TLI_InitializeSimulations", None, c_short)
TLI_UninitializeSimulations = bind(lib, "TLI_UninitializeSimulations", None, c_short)


CC_Open = bind(lib, "CC_Open", [POINTER(c_char)], c_short)
CC_Close = bind(lib, "CC_Close", [POINTER(c_char)], c_short)
CC_CheckConnection = bind(lib, "CC_CheckConnection", [POINTER(c_char)], c_bool)
CC_Identify = bind(lib, "CC_Identify", [POINTER(c_char)], c_short)
CC_RequestLEDswitches = bind(lib, "CC_RequestLEDswitches", [POINTER(c_char)], c_short)
CC_GetLEDswitches = bind(lib, "CC_GetLEDswitches", [POINTER(c_char)], c_word)
CC_SetLEDswitches = bind(lib, "CC_SetLEDswitches", [POINTER(c_char), c_word], c_word)
CC_GetHardwareInfo = bind(lib, "CC_GetHardwareInfo", [POINTER(c_char), POINTER(c_char), c_dword, POINTER(c_word), POINTER(c_word), POINTER(c_char), c_dword, POINTER(c_dword), POINTER(c_word), POINTER(c_word)], c_short)
CC_GetHardwareInfoBlock = bind(lib, "CC_GetHardwareInfoBlock", [POINTER(c_char), POINTER(TLI_HardwareInformation)], c_short)
CC_GetHubBay = bind(lib, "CC_GetHubBay", [POINTER(c_char)], c_short)
CC_GetSoftwareVersion = bind(lib, "CC_GetSoftwareVersion", [POINTER(c_char)], c_dword)
CC_LoadSettings = bind(lib, "CC_LoadSettings", [POINTER(c_char)], c_bool)
CC_LoadNamedSettings = bind(lib, "CC_LoadNamedSettings", [POINTER(c_char), POINTER(c_char)], c_bool)
CC_PersistSettings = bind(lib, "CC_PersistSettings", [POINTER(c_char)], c_bool)
#CC_ResetStageToDefaults = bind(lib, "CC_ResetStageToDefaults", [POINTER(c_char)], c_short)
CC_DisableChannel = bind(lib, "CC_DisableChannel", [POINTER(c_char)], c_short)
CC_EnableChannel = bind(lib, "CC_EnableChannel", [POINTER(c_char)], c_short)
#CC_CanDeviceLockFrontPanel = bind(lib, "CC_CanDeviceLockFrontPanel", [POINTER(c_char)], c_bool)
#CC_GetFrontPanelLocked = bind(lib, "CC_GetFrontPanelLocked", [POINTER(c_char)], c_bool)
#CC_RequestFrontPanelLocked = bind(lib, "CC_RequestFrontPanelLocked", [POINTER(c_char)], c_short)
#CC_SetFrontPanelLocked = bind(lib, "CC_SetFrontPanelLocked", [POINTER(c_char), c_bool], c_short)
CC_GetNumberPositions = bind(lib, "CC_GetNumberPositions", [POINTER(c_char)], c_int)
CC_MoveToPosition = bind(lib, "CC_MoveToPosition", [POINTER(c_char), c_int], c_short)
CC_GetPosition = bind(lib, "CC_GetPosition", [POINTER(c_char)], c_int)
CC_CanHome = bind(lib, "CC_CanHome", [POINTER(c_char)], c_bool)
CC_NeedsHoming = bind(lib, "CC_NeedsHoming", [POINTER(c_char)], c_bool)
CC_CanMoveWithoutHomingFirst = bind(lib, "CC_CanMoveWithoutHomingFirst", [POINTER(c_char)], c_bool)
CC_Home = bind(lib, "CC_Home", [POINTER(c_char)], c_short)
CC_ClearMessageQueue = bind(lib, "CC_ClearMessageQueue", [POINTER(c_char)], c_short)
CC_MessageQueueSize = bind(lib, "CC_MessageQueueSize", [POINTER(c_char)], c_int)
CC_GetNextMessage = bind(lib, "CC_GetNextMessage", [POINTER(c_char), POINTER(c_word), POINTER(c_word), POINTER(c_dword)], c_bool)
CC_WaitForMessage = bind(lib, "CC_WaitForMessage", [POINTER(c_char), POINTER(c_word), POINTER(c_word), POINTER(c_dword)], c_bool)
CC_RequestHomingParams = bind(lib, "CC_RequestHomingParams", [POINTER(c_char)], c_short)
CC_GetHomingVelocity = bind(lib, "CC_GetHomingVelocity", [POINTER(c_char)], c_uint)
CC_SetHomingVelocity = bind(lib, "CC_SetHomingVelocity", [POINTER(c_char), c_uint], c_short)
CC_MoveRelative = bind(lib, "CC_MoveRelative", [POINTER(c_char), c_int], c_short)
CC_RequestJogParams = bind(lib, "CC_RequestJogParams", [POINTER(c_char)], c_short)
CC_GetJogMode = bind(lib, "CC_GetJogMode", [POINTER(c_char), POINTER(MOT_JogModes), POINTER(MOT_StopModes)], c_short)
CC_SetJogMode = bind(lib, "CC_SetJogMode", [POINTER(c_char), MOT_JogModes, MOT_StopModes], c_short)
CC_GetJogStepSize = bind(lib, "CC_GetJogStepSize", [POINTER(c_char)], c_uint)
CC_SetJogStepSize = bind(lib, "CC_SetJogStepSize", [POINTER(c_char), c_uint], c_short)
CC_GetJogVelParams = bind(lib, "CC_GetJogVelParams", [POINTER(c_char), POINTER(c_int), POINTER(c_int)], c_short)
CC_SetJogVelParams = bind(lib, "CC_SetJogVelParams", [POINTER(c_char), c_int, c_int], c_short)
CC_MoveJog = bind(lib, "CC_MoveJog", [POINTER(c_char), MOT_TravelDirection], c_short)
CC_RequestVelParams = bind(lib, "CC_RequestVelParams", [POINTER(c_char)], c_short)
CC_GetVelParams = bind(lib, "CC_GetVelParams", [POINTER(c_char), POINTER(c_int), POINTER(c_int)], c_short)
CC_SetVelParams = bind(lib, "CC_SetVelParams", [POINTER(c_char), c_int, c_int], c_short)
CC_MoveAtVelocity = bind(lib, "CC_MoveAtVelocity", [POINTER(c_char), MOT_TravelDirection], c_short)
CC_SetDirection = bind(lib, "CC_SetDirection", [POINTER(c_char), c_bool], c_short)
CC_StopImmediate = bind(lib, "CC_StopImmediate", [POINTER(c_char)], c_short)
CC_StopProfiled = bind(lib, "CC_StopProfiled", [POINTER(c_char)], c_short)
CC_RequestBacklash = bind(lib, "CC_RequestBacklash", [POINTER(c_char)], c_short)
CC_GetBacklash = bind(lib, "CC_GetBacklash", [POINTER(c_char)], c_long)
CC_SetBacklash = bind(lib, "CC_SetBacklash", [POINTER(c_char), c_long], c_short)
CC_GetPositionCounter = bind(lib, "CC_GetPositionCounter", [POINTER(c_char)], c_long)
CC_SetPositionCounter = bind(lib, "CC_SetPositionCounter", [POINTER(c_char), c_long], c_short)
CC_RequestEncoderCounter = bind(lib, "CC_RequestEncoderCounter", [POINTER(c_char)], c_short)
CC_GetEncoderCounter = bind(lib, "CC_GetEncoderCounter", [POINTER(c_char)], c_long)
CC_SetEncoderCounter = bind(lib, "CC_SetEncoderCounter", [POINTER(c_char), c_long], c_short)
CC_RequestLimitSwitchParams = bind(lib, "CC_RequestLimitSwitchParams", [POINTER(c_char)], c_short)
CC_GetLimitSwitchParams = bind(lib, "CC_GetLimitSwitchParams", [POINTER(c_char), POINTER(MOT_LimitSwitchModes), POINTER(MOT_LimitSwitchModes), POINTER(c_uint), POINTER(c_uint), POINTER(MOT_LimitSwitchSWModes)], c_short)
CC_SetLimitSwitchParams = bind(lib, "CC_SetLimitSwitchParams", [POINTER(c_char), MOT_LimitSwitchModes, MOT_LimitSwitchModes, c_uint, c_uint, MOT_LimitSwitchSWModes], c_short)

CC_GetSoftLimitMode = bind(lib, "CC_GetSoftLimitMode", [POINTER(c_char)], MOT_LimitsSoftwareApproachPolicy)
CC_SetLimitsSoftwareApproachPolicy = bind(lib, "CC_SetLimitsSoftwareApproachPolicy", [POINTER(c_char), MOT_LimitsSoftwareApproachPolicy])


CC_RequestButtonParams = bind(lib, "CC_RequestButtonParams", [POINTER(c_char)], c_short)
CC_GetButtonParams = bind(lib, "CC_GetButtonParams", [POINTER(c_char), POINTER(MOT_ButtonModes), POINTER(c_int), POINTER(c_int), POINTER(c_short)], c_short)
CC_SetButtonParams = bind(lib, "CC_SetButtonParams", [POINTER(c_char), MOT_ButtonModes, c_int, c_int, c_short], c_short)
CC_RequestPotentiometerParams = bind(lib, "CC_RequestPotentiometerParams", [POINTER(c_char)], c_short)
CC_GetPotentiometerParams = bind(lib, "CC_GetPotentiometerParams", [POINTER(c_char), c_short, POINTER(c_word), POINTER(c_dword)], c_short)
CC_SetPotentiometerParams = bind(lib, "CC_SetPotentiometerParams", [POINTER(c_char), c_short, c_word, c_dword], c_short)


CC_GetVelParamsBlock = bind(lib, "CC_GetVelParamsBlock", [POINTER(c_char), POINTER(MOT_VelocityParameters)], c_short)
CC_SetVelParamsBlock = bind(lib, "CC_SetVelParamsBlock", [POINTER(c_char), POINTER(MOT_VelocityParameters)], c_short)
CC_SetMoveAbsolutePosition = bind(lib, "CC_SetMoveAbsolutePosition", [POINTER(c_char), c_int], c_short)
CC_RequestMoveAbsolutePosition = bind(lib, "CC_RequestMoveAbsolutePosition", [POINTER(c_char)], c_short)
CC_GetMoveAbsolutePosition = bind(lib, "CC_GetMoveAbsolutePosition", [POINTER(c_char)], c_int)
CC_MoveAbsolute = bind(lib, "CC_MoveAbsolute", [POINTER(c_char)], c_short)

CC_SetMoveRelativeDistance = bind(lib, "CC_SetMoveRelativeDistance", [POINTER(c_char), c_int], c_short)
CC_RequestMoveRelativeDistance = bind(lib, "CC_RequestMoveRelativeDistance", [POINTER(c_char)], c_short)
CC_GetMoveRelativeDistance = bind(lib, "CC_GetMoveRelativeDistance", [POINTER(c_char)], c_int)
CC_MoveRelativeDistance = bind(lib, "CC_MoveRelativeDistance", [POINTER(c_char)], c_short)
CC_GetHomingParamsBlock = bind(lib, "CC_GetHomingParamsBlock", [POINTER(c_char), POINTER(MOT_HomingParameters)], c_short)
CC_SetHomingParamsBlock = bind(lib, "CC_SetHomingParamsBlock", [POINTER(c_char), POINTER(MOT_HomingParameters)], c_short)
CC_GetJogParamsBlock = bind(lib, "CC_GetJogParamsBlock", [POINTER(c_char), POINTER(MOT_JogParameters)], c_short)
CC_SetJogParamsBlock = bind(lib, "CC_SetJogParamsBlock", [POINTER(c_char), POINTER(MOT_JogParameters)], c_short)

CC_GetButtonParamsBlock = bind(lib, "CC_GetButtonParamsBlock", [POINTER(c_char), POINTER(MOT_ButtonParameters)], c_short)
CC_SetButtonParamsBlock = bind(lib, "CC_SetButtonParamsBlock", [POINTER(c_char), POINTER(MOT_ButtonParameters)], c_short)
CC_GetPotentiometerParamsBlock = bind(lib, "CC_GetPotentiometerParamsBlock", [POINTER(c_char), POINTER(MOT_PotentiometerSteps)], c_short)
CC_SetPotentiometerParamsBlock = bind(lib, "CC_SetPotentiometerParamsBlock", [POINTER(c_char), POINTER(MOT_PotentiometerSteps)], c_short)

CC_GetLimitSwitchParamsBlock = bind(lib, "CC_GetLimitSwitchParamsBlock", [POINTER(c_char), POINTER(MOT_LimitSwitchParameters)], c_short)
CC_SetLimitSwitchParamsBlock = bind(lib, "CC_SetLimitSwitchParamsBlock", [POINTER(c_char), POINTER(MOT_LimitSwitchParameters)], c_short)


CC_RequestDCPIDParams = bind(lib, "CC_RequestDCPIDParams", [POINTER(c_char)], c_short)
CC_GetDCPIDParams = bind(lib, "CC_GetDCPIDParams", [POINTER(c_char), POINTER(MOT_DC_PIDParameters)], c_short)
CC_SetDCPIDParams = bind(lib, "CC_SetDCPIDParams", [POINTER(c_char), POINTER(MOT_DC_PIDParameters)], c_short)

CC_SuspendMoveMessages = bind(lib, "CC_SuspendMoveMessages", [POINTER(c_char)], c_short)
CC_ResumeMoveMessages = bind(lib, "CC_ResumeMoveMessages", [POINTER(c_char)], c_short)
CC_RequestPosition = bind(lib, "CC_RequestPosition", [POINTER(c_char)], c_short)
CC_RequestStatusBits = bind(lib, "CC_RequestStatusBits", [POINTER(c_char)], c_short)
CC_GetStatusBits = bind(lib, "CC_GetStatusBits", [POINTER(c_char)], c_dword)
CC_StartPolling = bind(lib, "CC_StartPolling", [POINTER(c_char), c_int], c_bool)
CC_PollingDuration = bind(lib, "CC_PollingDuration", [POINTER(c_char)], c_long)
CC_StopPolling = bind(lib, "CC_StopPolling", [POINTER(c_char)])

#CC_TimeSinceLastMsgReceived	KCUBEDCSERVO_API bool __cdecl CC_TimeSinceLastMsgReceived(char const * serialNo, __int64 &lastUpdateTimeMS);

CC_EnableLastMsgTimer = bind(lib, "CC_EnableLastMsgTimer", [POINTER(c_char), c_bool, c_int32])
CC_HasLastMsgTimerOverrun = bind(lib, "CC_HasLastMsgTimerOverrun", [POINTER(c_char)], c_bool)
CC_RequestSettings = bind(lib, "CC_RequestSettings", [POINTER(c_char)], c_short)
CC_GetStageAxisMinPos = bind(lib, "CC_GetStageAxisMinPos", [POINTER(c_char)], c_int)
CC_GetStageAxisMaxPos = bind(lib, "CC_GetStageAxisMaxPos", [POINTER(c_char)], c_int)
CC_SetStageAxisLimits = bind(lib, "CC_SetStageAxisLimits", [POINTER(c_char), c_int, c_int], c_short)
CC_SetMotorTravelMode = bind(lib, "CC_SetMotorTravelMode", [POINTER(c_char), MOT_TravelModes], c_short)
CC_GetMotorTravelMode = bind(lib, "CC_GetMotorTravelMode", [POINTER(c_char)], MOT_TravelModes)
CC_SetMotorParams = bind(lib, "CC_SetMotorParams", [POINTER(c_char), c_long, c_long, c_float], c_short)
CC_GetMotorParams = bind(lib, "CC_GetMotorParams", [POINTER(c_char), POINTER(c_long), POINTER(c_long), POINTER(c_float)], c_short)
CC_SetMotorParamsExt = bind(lib, "CC_SetMotorParamsExt", [POINTER(c_char), c_double, c_double, c_double], c_short)
CC_GetMotorParamsExt = bind(lib, "CC_GetMotorParamsExt", [POINTER(c_char), POINTER(c_double), POINTER(c_double), POINTER(c_double)], c_short)

CC_SetMotorVelocityLimits = bind(lib, "CC_SetMotorVelocityLimits", [POINTER(c_char), c_double, c_double], c_short)
CC_GetMotorVelocityLimits = bind(lib, "CC_GetMotorVelocityLimits", [POINTER(c_char), POINTER(c_double), POINTER(c_double)], c_short)
CC_ResetRotationModes = bind(lib, "CC_ResetRotationModes", [POINTER(c_char)], c_short)
CC_SetRotationModes = bind(lib, "CC_SetRotationModes", [POINTER(c_char), MOT_MovementModes, MOT_MovementDirections], c_short)
CC_SetMotorTravelLimits = bind(lib, "CC_SetMotorTravelLimits", [POINTER(c_char), c_double, c_double], c_short)
CC_GetMotorTravelLimits = bind(lib, "CC_GetMotorTravelLimits", [POINTER(c_char), POINTER(c_double), POINTER(c_double)], c_short)
CC_GetRealValueFromDeviceUnit = bind(lib, "CC_GetRealValueFromDeviceUnit", [POINTER(c_char), c_int, POINTER(c_double), c_int], c_short)
CC_GetDeviceUnitFromRealValue = bind(lib, "CC_GetDeviceUnitFromRealValue", [POINTER(c_char), c_double, POINTER(c_int), c_int], c_short)

