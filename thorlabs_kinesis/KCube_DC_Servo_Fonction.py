# -*- coding: utf-8 -*-
"""
Created on Wed Oct  7 11:05:34 2020

@author: fani9660
"""

#"Sample code to get information about connected controllers."
from ctypes import (
    c_int,
    c_char_p,
    byref,
    c_short,
    c_uint,
    c_double,
    c_ulong,
)

from thorlabs_kinesis._utils import (
    c_word,
)

from thorlabs_kinesis import KCube_DC_Servo as KCube


class KCubeDCServoFonction():
    """Fonctions pour contrôle du KCube
    """

    def __init___(self, serial):
        self.serial = c_char_p(bytes(serial, "utf-8"))
        self.KCube = KCube
        return self.serial

    def can_home(self):
        """Can the device perform a :meth:`home`?
        Returns
        -------
        :class:`bool`
            Whether the device can be homed.        """
        return self.KCube.CC_CanHome(self.serial)

    def can_move_without_homing_first(self):
        """Does the device need to be :meth:`home`\'d before a move can be performed?
        Returns
        -------
        :class:`bool`
            Whether the device needs to be homed.        """
        return self.KCube.CC_CanMoveWithoutHomingFirst(self.serial)

    def check_connection(self):
        """Check connection.
        Returns
        -------
        :class:`bool`
            Whether the USB is listed by the FTDI controller.        """
        return self.KCube.CC_CheckConnection(self.serial)

    def clear_message_queue(self):
        """Clears the device message queue."""
        self.KCube.CC_ClearMessageQueue(self.serial)

    def close(self):
        """Disconnect and close the device."""
        self.KCube.CC_Close(self.serial)

    def disable_channel(self):
        """Disable the channel so that motor can be moved by hand.
        When disabled, power is removed from the motor and it can be freely moved.       """
        self.KCube.CC_DisableChannel(self.serial)

    def enable_channel(self):
        """Enable channel for computer control.
        When enabled, power is applied to the motor so it is fixed in position.        """
        self.KCube.CC_EnableChannel(self.serial)

    def enable_last_msg_timer(self, enable, last_msg_timeout):
        """Enables the last message monitoring timer.
        This can be used to determine whether communications with the device is
        still good.
        Parameters
        ----------
        enable : :class:`bool`
            :data:`True` to enable monitoring otherwise :data:`False` to disable.
        last_msg_timeout : :class:`int`
            The last message error timeout in ms. Set to 0 to disable.        """
        self.KCube.CC_EnableLastMsgTimer(self.serial, enable, last_msg_timeout)

    def get_backlash(self):
        """Get the backlash distance setting (used to control hysteresis).
        See :meth:`get_real_value_from_device_unit` for converting from a
        ``DeviceUnit`` to a ``RealValue``.
        Returns
        -------
        :class:`int`
            The backlash distance in ``DeviceUnits`` (see manual).        """
        return self.KCube.CC_GetBacklash(self.serial)

    def get_encoder_counter(self):
        """Get the encoder counter.
        For devices that have an encoder, the current encoder position can be read.
        Returns
        -------
        :class:`int`
            The encoder count in encoder units.        """
        return self.KCube.CC_GetEncoderCounter(self.serial)

    def get_hardware_info(self):
        """Gets the hardware information from the device.
        Returns
        -------
        :class:`.structs.TLI_HardwareInformation`
            The hardware information.        """
        return self.KCube.CC_GetHardwareInfo

    def get_hardware_info_block(self):
        """Gets the hardware information in a block.
        Returns
        -------
        :class:`.structs.TLI_HardwareInformation`
            The hardware information.        """
        info = KCube.TLI_HardwareInformation()
        self.KCube.CC_GetHardwareInfoBlock(self.serial, byref(info))
        return info

    def get_homing_params_block(self):
        """Get the homing parameters.
        Returns
        -------
        :class:`.structs.MOT_HomingParameters`
            The homing parameters.        """
        params = KCube.MOT_HomingParameters()
        self.KCube.CC_GetHomingParamsBlock(self.serial, byref(params))
        return params

    def get_homing_velocity(self):
        """Gets the homing velocity.
        See :meth:`get_real_value_from_device_unit` for converting from a
        ``DeviceUnit`` to a ``RealValue``.
        Returns
        -------
        :class:`int`
            The homing velocity in ``DeviceUnits`` (see manual).        """
        return self.KCube.CC_GetHomingVelocity(self.serial)

    def get_jog_mode(self):
        """Gets the jog mode.
        Returns
        -------
        :class:`.enums.MOT_JogModes`
            The jog mode.
        :class:`.enums.MOT_StopModes`
            The stop mode.        """
        mode = c_short()
        stop_mode = c_short()
        self.KCube.CC_GetJogMode(self.serial, byref(mode), byref(stop_mode))
        return KCube.MOT_JogModes(mode.value), KCube.MOT_StopModes(stop_mode.value)

    def get_jog_params_block(self):
        """Get the jog parameters.
        Returns
        -------
        :class:`.structs.MOT_JogParameters`
            The jog parameters.        """
        params = KCube.MOT_JogParameters()
        self.KCube.CC_GetJogParamsBlock(self.serial, byref(params))
        return params

    def get_jog_step_size(self):
        """Gets the distance to move when jogging.
        See :meth:`get_real_value_from_device_unit` for converting from a
        ``DeviceUnit`` to a ``RealValue``.
        Returns
        -------
        :class:`int`
            The step size in ``DeviceUnits`` (see manual).        """
        return self.KCube.CC_GetJogStepSize(self.serial)

    def get_jog_vel_params(self):
        """Gets the jog velocity parameters.
        See :meth:`get_real_value_from_device_unit` for converting from a
        ``DeviceUnit`` to a ``RealValue``.
        Returns
        -------
        :class:`int`
            The maximum velocity in ``DeviceUnits`` (see manual).
        :class:`int`
            The acceleration in ``DeviceUnits`` (see manual).        """
        acceleration = c_int()
        max_velocity = c_int()
        self.KCube.CC_GetJogVelParams(self.serial, byref(acceleration), byref(max_velocity))
        return max_velocity.value, acceleration.value

    def get_led_switches(self):
        """Get the LED indicator bits on cube.
        Returns
        -------
        :class:`int`
            Sum of: 8 to indicate moving 2 to indicate end of track and 1
            to flash on identify command.        """
        return self.KCube.CC_GetLEDswitches(self.serial)

    def get_limit_switch_params(self):
        """ Gets the limit switch parameters.
        See :meth:`get_real_value_from_device_unit` for converting from a
        ``DeviceUnit`` to a ``RealValue``.
        Returns
        -------
        :class:`.enums.MOT_LimitSwitchModes`
            The clockwise hardware limit mode.
        :class:`.enums.MOT_LimitSwitchModes`
            The anticlockwise hardware limit mode.
        :class:`int`
            The position of the clockwise software limit in ``DeviceUnits`` (see manual).
        :class:`int`
            The position of the anticlockwise software limit in ``DeviceUnits`` (see manual).
        :class:`.enums.MOT_LimitSwitchSWModes`
            The soft limit mode.
        """
        cw_lim = c_word()
        ccw_lim = c_word()
        cw_pos = c_uint()
        ccw_pos = c_uint()
        soft = c_word()
        self.KCube.CC_GetLimitSwitchParams(self.serial, byref(cw_lim), byref(ccw_lim),
                                           byref(cw_pos), byref(ccw_pos), byref(soft))
        try:
            cw_mode = KCube.MOT_LimitSwitchModes(cw_lim.value)
        except ValueError:
            cw_mode = KCube.MOT_LimitSwitchModes(cw_lim.value | 0x0080)
        try:
            ccw_mode = KCube.MOT_LimitSwitchModes(ccw_lim.value)
        except ValueError:
            ccw_mode = KCube.MOT_LimitSwitchModes(ccw_lim.value | 0x0080)
        try:
            s_mode = KCube.MOT_LimitSwitchSWModes(soft.value)
        except ValueError:
            s_mode = KCube.MOT_LimitSwitchSWModes(soft.value | 0x0080)
        return cw_mode, ccw_mode, cw_pos.value, ccw_pos.value, s_mode

    def get_limit_switch_params_block(self):
        """Get the limit switch parameters.
        Returns
        -------
        :class:`.structs.MOT_LimitSwitchParameters`
            The limit switch parameters.
        """
        params = KCube.MOT_LimitSwitchParameters()
        self.KCube.CC_GetLimitSwitchParamsBlock(self.serial, byref(params))
        return params

    def get_motor_params(self):
        """Gets the motor stage parameters.
        Deprecated: calls :meth:`get_motor_params_ext`
        These parameters, when combined define the stage motion in terms of
        ``RealWorldUnits`` [millimeters or degrees]. The real-world unit
        is defined from ``steps_per_rev * gear_box_ratio / pitch``.
        Returns
        ----------
        :class:`float`
            The steps per revolution.
        :class:`float`
            The gear box ratio.
        :class:`float`
            The pitch.
        """
        return self.get_motor_params_ext()

    def get_motor_params_ext(self):
        """Gets the motor stage parameters.
        These parameters, when combined define the stage motion in terms of
        ``RealWorldUnits`` [millimeters or degrees]. The real-world unit
        is defined from ``steps_per_rev * gear_box_ratio / pitch``.
        Returns
        ----------
        :class:`float`
            The steps per revolution.
        :class:`float`
            The gear box ratio.
        :class:`float`
            The pitch.
        """
        steps_per_rev = c_double()
        gear_box_ratio = c_double()
        pitch = c_double()
        self.KCube.CC_GetMotorParamsExt(self.serial, byref(steps_per_rev),
                                        byref(gear_box_ratio), byref(pitch))
        return steps_per_rev.value, gear_box_ratio.value, pitch.value

    def get_motor_travel_limits(self):
        """Gets the motor stage min and max position.
        Returns
        -------
        :class:`float`
            The minimum position in ``RealWorldUnits`` [millimeters or degrees].
        :class:`float`
            The maximum position in ``RealWorldUnits`` [millimeters or degrees].
        """
        min_position = c_double()
        max_position = c_double()
        self.KCube.CC_GetMotorTravelLimits(self.serial, byref(min_position),
                                           byref(max_position))
        return min_position.value, max_position.value


    def get_motor_travel_mode(self):
        """Get the motor travel mode.
        Returns
        -------
        :class:`.enums.MOT_TravelModes`
            The travel mode.
        """
        return KCube.MOT_TravelModes(self.KCube.CC_GetMotorTravelMode(self.serial))

    def get_motor_velocity_limits(self):
        """Gets the motor stage maximum velocity and acceleration.
        Returns
        -------
        :class:`float`
            The maximum velocity in ``RealWorldUnits`` [millimeters or degrees].
        :class:`float`
            The maximum acceleration in ``RealWorldUnits`` [millimeters or degrees].
        """
        max_velocity = c_double()
        max_acceleration = c_double()
        self.KCube.CC_GetMotorVelocityLimits(self.serial, byref(max_velocity),
                                             byref(max_acceleration))
        return max_velocity.value, max_acceleration.value

    def get_move_absolute_position(self):
        """Gets the move absolute position.
        See :meth:`get_real_value_from_device_unit` for converting from a
        ``DeviceUnit`` to a ``RealValue``.
        Returns
        -------
        :class:`int`
            The move absolute position in ``DeviceUnits`` (see manual).
        """
        return self.KCube.CC_GetMoveAbsolutePosition(self.serial)

    def get_move_relative_distance(self):
        """Gets the move relative distance.
        See :meth:`get_real_value_from_device_unit` for converting from a
        ``DeviceUnit`` to a ``RealValue``.
        Returns
        -------
        :class:`int`
            The move relative position in ``DeviceUnits`` (see manual).
        """
        return self.KCube.CC_GetMoveRelativeDistance(self.serial)

    def get_next_message(self):
        """Get the next Message Queue item. See :mod:`.messages`.
        Returns
        -------
        :class:`int`
            The message type.
        :class:`int`
            The message ID.
        :class:`int`
            The message data.
        """
        message_type = c_word()
        message_id = c_word()
        message_data = c_ulong()
        self.KCube.CC_GetNextMessage(self.serial, byref(message_type),
                                     byref(message_id), byref(message_data))
        return message_type.value, message_id.value, message_data.value

    def get_number_positions(self):
        """Get the number of positions.
        This function will get the maximum position reachable by the device.
        The motor may need to be set to its :meth:`home` position before this
        parameter can be used.
        Returns
        -------
        :class:`int`
            The number of positions.
        """
        return self.KCube.CC_GetNumberPositions(self.serial)

    def get_position(self):
        """Get the current position.
        See :meth:`get_real_value_from_device_unit` for converting from a
        ``DeviceUnit`` to a ``RealValue``.
        Returns
        -------
        index : :class:`int`
            The position in ``DeviceUnits`` (see manual).
        """
        return self.KCube.CC_GetPosition(self.serial)

    def get_position_counter(self):
        """Get the position counter.
        The position counter is identical to the position parameter.
        The position counter is set to zero when homing is complete.
        See :meth:`get_real_value_from_device_unit` for converting from a
        ``DeviceUnit`` to a ``RealValue``.
        Returns
        -------
        :class:`int`
            The position counter in ``DeviceUnits`` (see manual).
        """
        return self.KCube.CC_GetPositionCounter(self.serial)

    def get_soft_limit_mode(self):
        """Gets the software limits mode.
        Returns
        -------
        :class:`.enums.MOT_LimitsSoftwareApproachPolicy`
            The software limits mode.
        """
        return KCube.MOT_LimitsSoftwareApproachPolicy(self.KCube.CC_GetSoftLimitMode(self.serial))

    def get_software_version(self):
        """Gets version number of the device software.
        Returns
        -------
        :class:`str`
            The device software version.
        """
        return self.KCube.CC_GetSoftwareVersion(self.serial)

    def get_stage_axis_max_pos(self):
        """Gets the Stepper Motor maximum stage position.
        See :meth:`get_real_value_from_device_unit` for converting from a
        ``DeviceUnit`` to a ``RealValue``.
        Returns
        -------
        :class:`int`
            The maximum position in ``DeviceUnits`` (see manual).
        """
        return self.KCube.CC_GetStageAxisMaxPos(self.serial)

    def get_stage_axis_min_pos(self):
        """Gets the Stepper Motor minimum stage position.
        See :meth:`get_real_value_from_device_unit` for converting from a
        ``DeviceUnit`` to a ``RealValue``.
        Returns
        -------
        :class:`int`
            The minimum position in ``DeviceUnits`` (see manual).
        """
        return self.KCube.CC_GetStageAxisMinPos(self.serial)

    def get_status_bits(self):
        """Get the current status bits.
        This returns the latest status bits received from the device.
        To get new status bits, use :meth:`request_status_bits` or use
        the polling functions, :meth:`start_polling`.
        Returns
        -------
        :class:`int`
            The status bits from the device.
        """
        return self.KCube.CC_GetStatusBits(self.serial)

    def get_vel_params(self):
        """Gets the move velocity parameters.
        See :meth:`get_real_value_from_device_unit` for converting from a
        ``DeviceUnit`` to a ``RealValue``.
        Returns
        -------
        max_velocity : :class:`int`
            The maximum velocity in ``DeviceUnits`` (see manual).
        acceleration : :class:`int`
            The acceleration in ``DeviceUnits`` (see manual).
        """
        acceleration = c_int()
        max_velocity = c_int()
        self.KCube.CC_GetVelParams(self.serial, byref(acceleration), byref(max_velocity))
        return max_velocity.value, acceleration.value

    def get_vel_params_block(self):
        """Get the move velocity parameters.
        Returns
        -------
        :class:`.structs.MOT_VelocityParameters`
            The velocity parameters.
        """
        params = KCube.MOT_VelocityParameters()
        self.KCube.CC_GetVelParamsBlock(self.serial, byref(params))
        return params

    def has_last_msg_timer_overrun(self):
        """Queries if the time since the last message has exceeded the
        ``lastMsgTimeout`` set by :meth:`.enable_last_msg_timer`.
        This can be used to determine whether communications with the device is
        still good.
        Returns
        -------
        :class:`bool`
            :data:`True` if last message timer has elapsed or
            :data:`False` if monitoring is not enabled or if time of last message
            received is less than ``lastMsgTimeout``.
        """
        return self.KCube.CC_HasLastMsgTimerOverrun(self.serial)

    def home(self):
        """Home the device.
        Homing the device will set the device to a known state and determine
        the home position.
        """
        return self.KCube.CC_Home(self.serial)

    def identify(self):
        """Sends a command to the device to make it identify itself."""
        self.KCube.CC_Identify(self.serial)

    def load_settings(self):
        """Update device with stored settings.
        The settings are read from ``ThorlabsDefaultSettings.xml``, which
        gets created when the Kinesis software is installed.
        """
        self.KCube.CC_LoadSettings(self.serial)

    def load_named_settings(self, settings_name):
        """Update device with named settings.
        Parameters
        ----------
        settings_name : :class:`str`
            The name of the device to load the settings for. Examples for the value
            of `setting_name` can be found in `ThorlabsDefaultSettings.xml``, which
            gets created when the Kinesis software is installed.
        """
        self.KCube.CC_LoadNamedSettings(self.serial, settings_name.encode())

    def message_queue_size(self):
        """Gets the size of the message queue.
        Returns
        -------
        :class:`int`
            The number of messages in the queue.
        """
        return self.KCube.CC_MessageQueueSize(self.serial)

    def move_absolute(self):
        """Moves the device to the position defined in :meth:`set_move_absolute_position`.
        """
        self.KCube.CC_MoveAbsolute(self.serial)

    def move_at_velocity(self, direction):
        """Start moving at the current velocity in the specified direction.
        Parameters
        ----------
        direction : :class:`.enums.MOT_TravelDirection`
            The required direction of travel as a :class:`.enums.MOT_TravelDirection`
            enum value or member name.
        """
        self.KCube.CC_MoveAtVelocity(self.serial, direction)


    def move_relative(self, displacement):
        """Move the motor by a relative amount.
        See :meth:`get_device_unit_from_real_value`:meth:`get_device_unit_from_real_value` for
        converting from a ``RealValue`` to a ``DeviceUnit``.
        Parameters
        ----------
        displacement : :class:`int`
            Signed displacement in ``DeviceUnits`` (see manual).
        """
        self.KCube.CC_MoveRelative(self.serial, displacement)

    def move_relative_distance(self):
        """Moves the device by a relative distance defined by :meth:`set_move_relative_distance`.
        """
        self.KCube.CC_MoveRelativeDistance(self.serial)

    def move_to_position(self, index):
        """Move the device to the specified position (index).
        The motor may need to be set to its :meth:`home` position before a
        position can be set.
        See :meth:`get_device_unit_from_real_value` for converting from a
        ``RealValue`` to a ``DeviceUnit``.
        Parameters
        ----------
        index : :class:`int`
            The position in ``DeviceUnits`` (see manual).
        """
        self.KCube.CC_MoveToPosition(self.serial, index)

    def needs_homing(self):
        """Does the device need to be :meth:`home`\'d before a move can be performed?
        Deprecated: calls :meth:`can_move_without_homing_first` instead.
        Returns
        -------
        :class:`bool`
            Whether the device needs to be homed.
        """
        return self.can_move_without_homing_first()

    def open(self):
        """Open the device for communication.
        """
        return self.KCube.CC_Open(self.serial)

    def persist_settings(self):
        """Persist the devices current settings.
        """
        self.KCube.CC_PersistSettings(self.serial)

    def polling_duration(self):
        """Gets the polling loop duration.
        Returns
        -------
        :class:`int`
            The time between polls in milliseconds or 0 if polling is not active.
        """
        return self.KCube.CC_PollingDuration(self.serial)

    def register_message_callback(self, callback):
        """Registers a callback on the message queue.
        Parameters
        ----------
        callback : :class:`~msl.equipment.resources.thorlabs.kinesis.callbacks.MotionControlCallback`
            A function to be called whenever messages are received.
        """
        self.KCube.CC_RegisterMessageCallback(self.serial, callback)

    def request_backlash(self):
        """Requests the backlash.
        """
        self.KCube.CC_RequestBacklash(self.serial)

    def request_encoder_counter(self):
        """Requests the encoder counter.
        """
        self.KCube.CC_RequestEncoderCounter(self.serial)

    def request_front_panel_locked(self):
        """Ask the device if its front panel is locked.
        """
        self.KCube.CC_RequestFrontPanelLocked(self.serial)

    def request_homing_params(self):
        """Requests the homing parameters.
        """
        self.KCube.CC_RequestHomingParams(self.serial)

    def request_jog_params(self):
        """Requests the jog parameters.
        """
        self.KCube.CC_RequestJogParams(self.serial)

    def request_led_switches(self):
        """Requests the LED indicator bits on the cube.
        """
        self.KCube.CC_RequestLEDswitches(self.serial)

    def request_limit_switch_params(self):
        """Requests the limit switch parameters.
        """
        self.KCube.CC_RequestLimitSwitchParams(self.serial)

    def request_move_absolute_position(self):
        """Requests the position of next absolute move.
        """
        self.KCube.CC_RequestMoveAbsolutePosition(self.serial)

    def request_move_relative_distance(self):
        """Requests the relative move distance.
        """
        self.KCube.CC_RequestMoveRelativeDistance(self.serial)

    def request_position(self):
        """Requests the current position.
        This needs to be called to get the device to send it's current position.
        Note, this is called automatically if ``Polling`` is enabled for the device
        using :meth:`start_polling`.
        """
        self.KCube.CC_RequestPosition(self.serial)

    def request_settings(self):
        """Requests that all settings are downloaded from the device.
        This function requests that the device upload all it's settings to the
        DLL.
        """
        self.KCube.CC_RequestSettings(self.serial)

    def request_status_bits(self):
        """Request the status bits which identify the current motor state.
        This needs to be called to get the device to send it's current status bits.
        Note, this is called automatically if ``Polling`` is enabled for the device
        using :meth:`start_polling`.
        """
        self.KCube.CC_RequestStatusBits(self.serial)

    def request_vel_params(self):
        """Requests the velocity parameters.
        """
        self.KCube.CC_RequestVelParams(self.serial)

    def reset_rotation_modes(self):
        """Reset the rotation modes for a rotational device.
        """
        self.KCube.CC_ResetRotationModes(self.serial)

    def reset_stage_to_defaults(self):
        """Reset the stage settings to defaults.
        """
        self.KCube.CC_ResetStageToDefaults(self.serial)

    def resume_move_messages(self):
        """Resume suspended move messages.
        """
        self.KCube.CC_ResumeMoveMessages(self.serial)

    def set_backlash(self, distance):
        """Sets the backlash distance (used to control hysteresis).
        See :meth:`get_device_unit_from_real_value` for converting from a
        ``RealValue`` to a ``DeviceUnit``.
        Parameters
        ----------
        distance : :class:`int`
            The backlash distance in ``DeviceUnits`` (see manual).
        """
        self.KCube.CC_SetBacklash(self.serial, distance)

    def set_direction(self, reverse):
        """Sets the motor direction sense.
        This function is used because some actuators have directions of motion
        reversed. This parameter will tell the system to reverse the direction sense
        when moving, jogging etc.
        Parameters
        ----------
        reverse : :class:`bool`
            If :data:`True` then directions will be swapped on these moves.
        """
        self.KCube.CC_SetDirection(self.serial, reverse)

    def set_homing_velocity(self, velocity):
        """Sets the homing velocity.
        See :meth:`get_device_unit_from_real_value` for converting from a
        ``RealValue`` to a ``DeviceUnit``.
        Parameters
        ----------
        velocity : :class:`int`
            The homing velocity in ``DeviceUnits`` (see manual).
        """
        self.KCube.CC_SetHomingVelocity(self.serial, velocity)

    def set_jog_step_size(self, step_size):
        """Sets the distance to move on jogging.
        See :meth:`get_device_unit_from_real_value` for converting from a
        ``RealValue`` to a ``DeviceUnit``.
        Parameters
        ----------
        step_size : :class:`int`
            The step size in ``DeviceUnits`` (see manual)
        """
        self.KCube.CC_SetJogStepSize(self.serial, step_size)

    def set_jog_vel_params(self, max_velocity, acceleration):
        """Sets jog velocity parameters.
        See :meth:`get_device_unit_from_real_value` for converting from a
        ``RealValue`` to a ``DeviceUnit``.
        Parameters
        ----------
        max_velocity : :class:`int`
            The maximum velocity in ``DeviceUnits`` (see manual).
        acceleration : :class:`int`
            The acceleration in ``DeviceUnits`` (see manual).
        """
        self.KCube.CC_SetJogVelParams(self.serial, acceleration, max_velocity)

    def set_led_switches(self, led_switches):
        """Set the LED indicator bits on the cube.
        Parameters
        ----------
        led_switches : :class:`int`
            Sum of: 8 to indicate moving 2 to indicate end of track and
            1 to flash on identify command.
        Raises
        ------
        ~msl.equipment.exceptions.ThorlabsError
            If not successful.
        """
        self.KCube.CC_SetLEDswitches(self.serial, led_switches)

    def set_motor_params(self, steps_per_rev, gear_box_ratio, pitch):
        """Sets the motor stage parameters.
        Deprecated: calls :meth:`set_motor_params_ext`
        These parameters, when combined, define the stage motion in terms of
        ``RealWorldUnits`` [millimeters or degrees]. The real-world unit
        is defined from ``steps_per_rev * gear_box_ratio / pitch``.
        See :meth:`get_real_value_from_device_unit` for converting from a
        ``DeviceUnit`` to a ``RealValue``.
        Parameters
        ----------
        steps_per_rev : :class:`float`
            The steps per revolution.
        gear_box_ratio : :class:`float`
            The gear box ratio.
        pitch : :class:`float`
            The pitch.
        Raises
        ------
        ~msl.equipment.exceptions.ThorlabsError
            If not successful.
        """
        self.set_motor_params_ext(steps_per_rev, gear_box_ratio, pitch)

    def set_motor_params_ext(self, steps_per_rev, gear_box_ratio, pitch):
        """Sets the motor stage parameters.
        These parameters, when combined, define the stage motion in terms of
        ``RealWorldUnits`` [millimeters or degrees]. The real-world unit
        is defined from ``steps_per_rev * gear_box_ratio / pitch``.
        See :meth:`get_real_value_from_device_unit` for converting from a
        ``DeviceUnit`` to a ``RealValue``.
        Parameters
        ----------
        steps_per_rev : :class:`float`
            The steps per revolution.
        gear_box_ratio : :class:`float`
            The gear box ratio.
        pitch : :class:`float`
            The pitch.
        Raises
        ------
        ~msl.equipment.exceptions.ThorlabsError
            If not successful.
        """
        self.KCube.CC_SetMotorParamsExt(self.serial, steps_per_rev, gear_box_ratio, pitch)

    def set_motor_travel_limits(self, min_position, max_position):
        """Sets the motor stage min and max position.
        These define the range of travel for the stage.
        See :meth:`get_real_value_from_device_unit` for converting from a
        ``DeviceUnit`` to a ``RealValue``.
        Parameters
        ----------
        min_position : :class:`float`
            The minimum position in ``RealWorldUnits`` [millimeters or degrees].
        max_position : :class:`float`
            The maximum position in ``RealWorldUnits`` [millimeters or degrees].
        Raises
        ------
        ~msl.equipment.exceptions.ThorlabsError
            If not successful.
        """
        self.KCube.CC_SetMotorTravelLimits(self.serial, min_position, max_position)

    def set_motor_velocity_limits(self, max_velocity, max_acceleration):
        """Sets the motor stage maximum velocity and acceleration.
        See :meth:`get_real_value_from_device_unit` for converting from a
        ``DeviceUnit`` to a ``RealValue``.
        Parameters
        ----------
        max_velocity : :class:`float`
            The maximum velocity in ``RealWorldUnits`` [millimeters or degrees].
        max_acceleration : :class:`float`
            The maximum acceleration in ``RealWorldUnits`` [millimeters or degrees].
        Raises
        ------
        ~msl.equipment.exceptions.ThorlabsError
            If not successful.
        """
        self.KCube.CC_SetMotorVelocityLimits(self.serial, max_velocity, max_acceleration)

    def set_move_absolute_position(self, position):
        """Sets the move absolute position.
        See :meth:`get_device_unit_from_real_value` for converting from a
        ``RealValue`` to a ``DeviceUnit``.
        Parameters
        ----------
        position : :class:`int`
            The absolute position in ``DeviceUnits`` (see manual).
        Raises
        ------
        ~msl.equipment.exceptions.ThorlabsError
            If not successful.
        """
        self.KCube.CC_SetMoveAbsolutePosition(self.serial, position)

    def set_move_relative_distance(self, distance):
        """Sets the move relative distance.
        See :meth:`get_device_unit_from_real_value` for converting from a
        ``RealValue`` to a ``DeviceUnit``.
        Parameters
        ----------
        distance : :class:`int`
            The relative position in ``DeviceUnits`` (see manual).
        Raises
        ------
        ~msl.equipment.exceptions.ThorlabsError
            If not successful.
        """
        self.KCube.CC_SetMoveRelativeDistance(self.serial, distance)

    def set_position_counter(self, count):
        """Set the position counter.
        Setting the position counter will locate the current position.
        Setting the position counter will effectively define the home position
        of a motor.
        See :meth:`get_device_unit_from_real_value` for converting from a
        ``RealValue`` to a ``DeviceUnit``.
        Parameters
        ----------
        count : :class:`int`
            The position counter in ``DeviceUnits`` (see manual).
        """
        self.KCube.CC_SetPositionCounter(self.serial, count)

    def set_stage_axis_limits(self, min_position, max_position):
        """Sets the stage axis position limits.
        See :meth:`get_device_unit_from_real_value` for converting from a
        ``RealValue`` to a ``DeviceUnit``.
        Parameters
        ----------
        min_position : :class:`int`
            The minimum position in ``DeviceUnits`` (see manual).
        max_position : :class:`int`
            The maximum position in ``DeviceUnits`` (see manual).
        """
        self.KCube.CC_SetStageAxisLimits(self.serial, min_position, max_position)

    def set_vel_params(self, max_velocity, acceleration):
        """Sets the move velocity parameters.
        See :meth:`get_device_unit_from_real_value` for converting from a
        ``RealValue`` to a ``DeviceUnit``.
        Parameters
        ----------
        max_velocity : :class:`int`
            The maximum velocity in ``DeviceUnits`` (see manual).
        acceleration : :class:`int`
            The acceleration in ``DeviceUnits`` (see manual).
        """
        self.KCube.CC_SetVelParams(self.serial, acceleration, max_velocity)

    def set_vel_params_block(self, min_velocity, max_velocity, acceleration):
        """Set the move velocity parameters.
        Parameters
        ----------
        min_velocity : :class:`int`
            The minimum velocity.
        max_velocity : :class:`int`
            The maximum velocity.
        acceleration : :class:`int`
            The acceleration.
        """
        params = KCube.MOT_VelocityParameters()
        params.minVelocity = min_velocity
        params.acceleration = acceleration
        params.maxVelocity = max_velocity
        self.KCube.CC_SetVelParamsBlock(self.serial, byref(params))

    def start_polling(self, milliseconds):
        """Starts the internal polling loop.
        This function continuously requests position and status messages.
        Parameters
        ----------
        milliseconds : :class:`int`
            The polling rate, in milliseconds.
        """
        self.KCube.CC_StartPolling(self.serial, milliseconds)

    def stop_immediate(self):
        """Stop the current move immediately (with the risk of losing track of the position).
        """
        self.KCube.CC_StopImmediate(self.serial)

    def stop_polling(self):
        """Stops the internal polling loop."""
        self.KCube.CC_StopPolling(self.serial)

    def stop_profiled(self):
        """Stop the current move using the current velocity profile.
        """
        self.KCube.CC_StopProfiled(self.serial)

    def suspend_move_messages(self):
        """Suspend automatic messages at ends of moves.
        Useful to speed up part of real-time system with lots of short moves.
        """
        self.KCube.CC_SuspendMoveMessages(self.serial)

    def wait_for_message(self):
        """Wait for next Message Queue item. See :mod:`.messages`.
        Returns
        -------
        :class:`int`
            The message type.
        :class:`int`
            The message ID.
        :class:`int`
            The message data.
        """
        message_type = c_word()
        message_id = c_word()
        message_data = c_word()
        self.KCube.CC_WaitForMessage(self.serial, byref(message_type), byref(message_id), byref(message_data))
        return message_type.value, message_id.value, message_data.value

    def TLI_BuildDeviceList(self):
        """Sends a command to the device to make it identify itself."""
        self.KCube.TLI_BuildDeviceList()
