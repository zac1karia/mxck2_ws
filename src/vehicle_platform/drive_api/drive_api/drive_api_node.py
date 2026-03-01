"""Sends PWM data to the car controller.

This module (ROS node) is doing the publishing part
as it is publishing to /drive_pwm, /cmd_vel, and /commands/motor/speed topics.

It listens to topics /drive_api/command (see drive_api.msg/drive_api_values message for more)
and /command (see commands_msgs.msg/CommandArrayStamped message for more),
which are used to give velocity / steering commands to the Drive-API.

Drive-API can publish several types of messages depending on selected settings:
BASIC (simulation=False, use_vesc=False)
 In BASIC mode, Drive-API is publishing to /drive_pwm topic which is subscribed by Teensy board.
 Both speed and steering is sent this way, specified in PWM duty.
BASIC+VESC (simulation=False, use_vesc=True)
 In BASIC+VESC mode, Drive-API is publishing steering to /drive_pwm topic (to be handled by Teensy),
 and speed to /commands/motor/speed topic, which is subscribed by VESC.
SIMULATION (simulation=True)
 In SIMULATION mode, Drive-API is publishing both speed and steering to /cmd_vel topic which
 is used as a standard for various simulators.

For controlling the speed and steering, following control modes are available:
LEGACY
 In LEGACY control mode, speed/steering value is specified in range from 0 to 1 (both included)
 and a direction. Boundary values have unusual meaning -- 0 represents lowest possible
 speed/steering, 1 represents highest possible speed/steering. Using values lower than 0 results
 in stopping the vehicle/resetting the steering. Values larger than 1 are ignored.
JOINT
 In JOINT control mode, speed/steering value is specified in range from -1 to 1 (both included).
 Boundary values have ordinary meaning -- 1 is highest speed (largest left steering) and -1 is
 highest backward speed (largest right steering). Requesting value 0 results in stopping
 the vehicle/resetting the steering. Values outside the range are ignored.
METRIC
 In METRIC control mode, only speed can be passed. It is specified in unlimited range in [m.s^-1].
 An attempt to pass steering in this control mode results in an error.
ANGULAR
 In ANGULAR control mode, only steering can be passed. It is specified in [rad] and in a range
 limited by 'SERVO_LEFT_MAX' and 'SERVO_RIGHT_MAX'. LEGACY steering parameters are required while
 using this control mode. An attempt to pass speed in this control mode results in an error.
"""

import sys

from typing import List, Tuple, Dict, Any

# ROS 2 Python Client API
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import \
    ParameterDescriptor, \
    ParameterType, \
    Parameter, \
    SetParametersResult, \
    FloatingPointRange, \
    IntegerRange

# Computation engine (less memory consuming than numpy)
import math

from enum import IntEnum

# Message types
from std_msgs.msg import Bool
from std_msgs.msg import Float64

try:
    from command_msgs.msg import Command
    from command_msgs.msg import CommandArrayStamped
    from command_msgs.msg import CommandParameter

    USE_COMMANDS = True
except ImportError:
    print('Unable to find Command* messages. Subscriber will be disabled.', file=sys.stderr)
    USE_COMMANDS = False

from teensy_drive_msgs.msg import DriveValues

try:
    from drive_api_msgs.msg import DriveApiValues

    USE_DAPI_COMMANDS = True
except ImportError:
    print('Unable to find drive_api_values message. Subscriber will be disabled.', file=sys.stderr)
    USE_DAPI_COMMANDS = False

# Twist expresses velocity in free space broken into its linear and angular parts:
#   Vector3 linear
#   Vector3 angular
# Vector3 represents a vector in free space:
#   float64 x
#   float64 y
#   float64 z
from geometry_msgs.msg import Twist


class SteeringDirection(IntEnum):
    LEFT = 2
    RIGHT = 3


class ThrottleDirection(IntEnum):
    FORWARD = 0
    BACKWARD = 1


class RunMode(IntEnum):
    BASIC = 0
    BASIC_VESC = 1
    SIMULATION = 2

    @classmethod
    def has_name(cls, name: str) -> bool:
        return name in cls._member_names_


class ControlMode(IntEnum):
    LEGACY = 0
    JOINT = 1
    METRIC = 2
    ANGULAR = 3


class InitError(Exception):
    pass


class DriveApiNode(Node):

    def __init__(self):
        super().__init__(node_name='drive_api')

        self.get_logger().info(f'initializing...')

        # configuration
        self.config_initialized: bool = False
        # config map
        # initialized from parameters in add_on_set_parameters_callback
        self.config: Dict[str, Any] = {}
        # first register parameters callbacks
        # (so they are triggered also for the initial invocation when declaring the parameters)
        # note: the order matters!
        #   callbacks are added in front to the list of callbacks
        #   so we need to add them in the reverse order
        self.add_on_set_parameters_callback(self.set_parameters_copy_and_recalculate_callback)
        self.add_on_set_parameters_callback(self.set_parameters_validate_callback)
        # then declare all parameters and let their values automatically get populated with CLI or files overrides)
        self.setup_parameters()
        # set flag to indicate that all parameters are loaded
        # and that set_parameters_copy_and_recalculate_callback should invoke recalculate_derived_configs
        # for any future updates
        self.config_initialized = True
        # manually invoke recalculate_derived_configs for the first time
        self.recalculate_derived_configs()

        # state
        self.msg = DriveValues()
        self.msg_vesc = Float64()
        self.msg_cmd_vel = Twist()
        self.eStop: bool = True
        self.run_mode: RunMode = RunMode[self.get_parameter('run_mode').value.upper()]

        # publishers
        self.pub = None  # self.create_publisher(msg_type=DriveValues, topic='drive_pwm', qos_profile=1)
        self.pub_vesc = None  # self.create_publisher(msg_type=Float64, topic='commands/motor/speed', qos_profile=1)
        self.pub_cmd_vel = None  # self.create_publisher(msg_type=Twist, topic='cmd_vel', qos_profile=1)

        if self.run_mode == RunMode.BASIC:
            self.pub = self.create_publisher(msg_type=DriveValues, topic='drive_pwm', qos_profile=1)
            pub_function = self.publish
        elif self.run_mode == RunMode.BASIC_VESC:
            self.pub = self.create_publisher(msg_type=DriveValues, topic='drive_pwm', qos_profile=1)
            self.pub_vesc = self.create_publisher(msg_type=Float64, topic='commands/motor/speed', qos_profile=1)
            pub_function = self.publish_with_vesc
        elif self.run_mode == RunMode.SIMULATION:
            self.pub_cmd_vel = self.create_publisher(msg_type=Twist, topic='cmd_vel', qos_profile=1)
            pub_function = self.publish_sim
        else:
            raise InitError('unknown run mode')

        # subscriptions
        self.dapi_cmds_subscription = None
        self.cmds_subscription = None

        if USE_DAPI_COMMANDS:
            self.dapi_cmds_subscription = self.create_subscription(
                msg_type=DriveApiValues,
                topic='/drive_api/command',
                callback=self.api_callback,
                qos_profile=1,
            )

        if USE_COMMANDS:
            self.cmds_subscription = self.create_subscription(
                msg_type=CommandArrayStamped,
                topic='/command',
                callback=self.command_callback,
                qos_profile=1,
            )

        # Register to 2-way /eStop
        self.estop_subscription = self.create_subscription(
            msg_type=Bool,
            topic='/eStop',
            callback=self.estop_callback,
            qos_profile=1,
        )

        # fixed rate publish loop
        # TODO: https://answers.ros.org/question/358343/rate-and-sleep-function-in-rclpy-library-for-ros2/
        # TODO: https://nicolovaligi.com/concurrency-and-parallelism-in-ros1-and-ros2-application-apis.html
        # TODO: https://yuzhangbit.github.io/tools/several-ways-of-writing-a-ros-node/
        # Function rate() creates rate object to loop at the desired rate in Hz
        # if rospy.has_param('~rate'):
        #     rate = rospy.Rate(rospy.get_param('~rate'))
        #     self.get_logger().info(
        #         f'Setting the publish rate of the node to the requested {rospy.get_param("~rate"):d} Hz.'
        #     )
        # else:
        #     rate = rospy.Rate(10)
        #     self.get_logger().info('Publishing the messages with default rate 10 Hz.')
        # TODO: configurable rate (via param)
        # self.pub_rate = self.create_rate(frequency=10)
        # Function is_shutdown() reacts to exit flag (Ctrl+C, etc.)
        # TODO: rewrite
        # while rclpy.ok():
        #     self.get_logger().info('calling pub_function()')
        #     pub_function()
        #     self.pub_rate.sleep()
        self.timer = self.create_timer(1.0 / self.config['publish_rate'], pub_function)

        self.get_logger().info(f'run_mode={self.run_mode.name.lower()}')

        self.get_logger().info('constructor exit')

        pass

    def setup_parameters(self):

        # TODO: uninitialized values for statically typed params do not cause failure?

        # (name, value, descriptor)
        params_def: List[Tuple[str, Any, ParameterDescriptor]] = [
            (
                'drive_battery.cells',
                None,
                ParameterDescriptor(
                    type=ParameterType.PARAMETER_INTEGER,
                    description='number of cells of the drive battery (the one for the motor and the servo)',
                    integer_range=[IntegerRange(
                        from_value=1,
                        to_value=5,
                        step=1,
                    )],
                )
            ),
            (
                'drive_battery.cell_voltage',
                None,
                ParameterDescriptor(
                    type=ParameterType.PARAMETER_DOUBLE,
                    description='voltage of a cell of the battery [V]',
                    floating_point_range=[FloatingPointRange(
                        from_value=1.0,
                        to_value=10.0,
                        step=0.1,
                    )],
                )
            ),
            # (
            #     # TODO
            #     'motor.to_erpm',
            #     None,
            #     ParameterDescriptor(
            #         type=ParameterType.PARAMETER_DOUBLE,
            #         description='TODO',
            #     )
            # ),
            # (
            #     # TODO
            #     'motor.erpm_max',
            #     None,
            #     ParameterDescriptor(
            #         type=ParameterType.PARAMETER_DOUBLE,
            #         # TODO: better description, correct unit
            #         description='TODO',
            #     )
            # ),
            (
                'motor.back_emf',
                None,
                ParameterDescriptor(
                    type=ParameterType.PARAMETER_INTEGER,
                    # TODO: better description, correct unit
                    description='constant velocity of the motor in [Kv]',
                )
            ),
            (
                'motor.poles',
                None,
                ParameterDescriptor(
                    type=ParameterType.PARAMETER_INTEGER,
                    description='number of poles in the motor',
                )
            ),
            (
                'motor.pinion',
                None,
                ParameterDescriptor(
                    type=ParameterType.PARAMETER_INTEGER,
                    description='number of teeth on the gear of the motor',
                )
            ),
            (
                'differential.spur',
                None,
                ParameterDescriptor(
                    type=ParameterType.PARAMETER_INTEGER,
                    description='number of teeth on the gear connected with motor gear',
                )
            ),
            (
                'differential.pinion',
                None,
                ParameterDescriptor(
                    type=ParameterType.PARAMETER_INTEGER,
                    description='number of teeth on the gear before the wheels',
                )
            ),
            (
                'differential.ring',
                None,
                ParameterDescriptor(
                    type=ParameterType.PARAMETER_INTEGER,
                    description='number of teeth on the wheel gear connected with differential gear',
                )
            ),
            (
                'wheels.radius',
                None,
                ParameterDescriptor(
                    type=ParameterType.PARAMETER_DOUBLE,
                    description='radius of the wheels [m]',
                )
            ),
            (
                'pwm.throttle.calm_value',
                None,
                ParameterDescriptor(
                    type=ParameterType.PARAMETER_INTEGER,
                    description='TODO',
                )
            ),
            (
                'pwm.throttle.forward.min',
                None,
                ParameterDescriptor(
                    type=ParameterType.PARAMETER_INTEGER,
                    description='TODO',
                )
            ),
            (
                'pwm.throttle.forward.max',
                None,
                ParameterDescriptor(
                    type=ParameterType.PARAMETER_INTEGER,
                    description='TODO',
                )
            ),
            (
                'pwm.throttle.backward.min',
                None,
                ParameterDescriptor(
                    type=ParameterType.PARAMETER_INTEGER,
                    description='TODO',
                )
            ),
            (
                'pwm.throttle.backward.max',
                None,
                ParameterDescriptor(
                    type=ParameterType.PARAMETER_INTEGER,
                    description='TODO',
                )
            ),
            (
                'pwm.steering.calm_value',
                None,
                ParameterDescriptor(
                    type=ParameterType.PARAMETER_INTEGER,
                    description='TODO',
                )
            ),
            (
                'pwm.steering.left.min',
                None,
                ParameterDescriptor(
                    type=ParameterType.PARAMETER_INTEGER,
                    description='TODO',
                )
            ),
            (
                'pwm.steering.left.max',
                None,
                ParameterDescriptor(
                    type=ParameterType.PARAMETER_INTEGER,
                    description='TODO',
                )
            ),
            (
                'pwm.steering.right.min',
                None,
                ParameterDescriptor(
                    type=ParameterType.PARAMETER_INTEGER,
                    description='TODO',
                )
            ),
            (
                'pwm.steering.right.max',
                None,
                ParameterDescriptor(
                    type=ParameterType.PARAMETER_INTEGER,
                    description='TODO',
                )
            ),
            (
                'angular_steering.left_max',
                None,
                ParameterDescriptor(
                    type=ParameterType.PARAMETER_DOUBLE,
                    description='in degrees',
                    floating_point_range=[FloatingPointRange(
                        from_value=0.0,
                        to_value=90.0,
                        step=0.0,
                    )],
                )
            ),
            (
                'angular_steering.right_max',
                None,
                ParameterDescriptor(
                    type=ParameterType.PARAMETER_DOUBLE,
                    description='in degrees',
                    floating_point_range=[FloatingPointRange(
                        from_value=0.0,
                        to_value=90.0,
                        step=0.0,
                    )],
                )
            ),
            (
                'simulation.throttle_modifier',
                1.0,
                ParameterDescriptor(
                    type=ParameterType.PARAMETER_DOUBLE,
                    description='Throttle modifier for simulation mode.',
                )
            ),
            (
                'simulation.steering_modifier',
                1.0,
                ParameterDescriptor(
                    type=ParameterType.PARAMETER_DOUBLE,
                    description='Steering modifier for simulation mode.',
                )
            ),
            (
                'run_mode',
                None,
                ParameterDescriptor(
                    type=ParameterType.PARAMETER_STRING,
                    read_only=True,
                    description='Drive API mode',
                    additional_constraints='allowed values are basic, basic_vesc, simulation',
                )
            ),
            (
                'publish_rate',
                10,
                ParameterDescriptor(
                    type=ParameterType.PARAMETER_INTEGER,
                    read_only=True,
                    description='Drive API publish rate [Hz]',
                )
            ),
        ]

        self.declare_parameters(
            namespace='',
            parameters=params_def,
        )

        pass

    def set_parameters_validate_callback(self, parameters: List[Parameter]) -> SetParametersResult:
        """Called when there is a request to set one or multiple parameters

        Called even for the initial parameter declaration (if registered before the declaration).

        Registered using self.add_on_set_parameters_callback(self.reconfigure_callback).

        Called for each parameter separately (i.e. len(parameters) == 1),
        unless multiple parameters are set using set_parameters_atomically (then len(parameters) >= 1).

        Before this callback is called, parameters' values are validated against their specified constraints (if any).
        If type or constraints validation fails, this callback will not be called at all.

        If this callback returns SetParametersResult(successful=False), the values will not be set.

        """

        # self.get_logger().info('set_parameters_validate_callback')

        # validate parameters
        for param in parameters:
            # print(f'  param={param.name} value={param.value}')
            if param.value is None:
                return SetParametersResult(
                    successful=False,
                    reason=f'missing value for {param.name}'
                )
            # validate run_mode
            if param.name == 'run_mode':
                if not RunMode.has_name(param.value.upper()):
                    return SetParametersResult(
                        successful=False,
                        reason=f'invalid run_mode=\'{param.value}\''
                    )
            pass

        # if we pass successful=False, parameter value will not be set
        # if parameter set was attempted using self.set_parameter*, node will exit with an error
        # if parameter set was attempted remotely, the remote caller is just passed the result
        # (failure and the optional configurable reason='why it was unsuccessful')
        return SetParametersResult(successful=True)

        pass

    def recalculate_derived_configs(self):
        self.get_logger().info('recalculate_derived_configs')
        self.config['pwm.throttle.forward.range'] = \
            self.config['pwm.throttle.forward.max'] - self.config['pwm.throttle.forward.min']
        self.config['pwm.throttle.backward.range'] = \
            self.config['pwm.throttle.backward.max'] - self.config['pwm.throttle.backward.min']
        self.config['pwm.steering.left.range'] = \
            self.config['pwm.steering.left.max'] - self.config['pwm.steering.left.min']
        self.config['pwm.steering.right.range'] = \
            self.config['pwm.steering.right.max'] - self.config['pwm.steering.right.min']
        # TODO: differing calculation for erpm_max in the original drive_api_node.py
        self.config['vesc.throttle.erpm_max'] = \
            (self.config['motor.poles'] / 2.0) * self.config['motor.back_emf'] \
            * self.config['drive_battery.cells'] * self.config['drive_battery.cell_voltage']
        self.config['vesc.throttle.to_erpm'] = \
            (self.config['motor.poles'] / 2.0) \
            * (1.0 * self.config['differential.spur'] / self.config['motor.pinion']) \
            * (1.0 * self.config['differential.ring'] / self.config['differential.pinion']) \
            / (2.0 * math.pi * self.config['wheels.radius']) \
            * 60
        pass

    def set_parameters_copy_and_recalculate_callback(self, parameters: List[Parameter]) -> SetParametersResult:
        """Called when there is a request to set one or multiple parameters

        Called even for the initial parameter declaration (if registered before the declaration).

        Registered using self.add_on_set_parameters_callback(self.reconfigure_callback).

        Called for each parameter separately (i.e. len(parameters) == 1),
        unless multiple parameters are set using set_parameters_atomically (then len(parameters) >= 1).

        Before this callback is called, parameters' values are validated against their specified constraints (if any).
        If type or constraints validation fails, this callback will not be called at all.

        If this callback returns SetParametersResult(successful=False), the values will not be set.

        """

        # self.get_logger().info('set_parameters_copy_and_recalculate_callback')

        # copy parameters to config
        for param in parameters:
            # print(f'  param={param.name} value={param.value}')
            if param.name == 'angular_steering.left_max' or param.name == 'angular_steering.right_max':
                self.config[param.name] = math.radians(param.value)  # convert degrees to radians
            else:
                self.config[param.name] = param.value  # use the value as it is
            pass

        if self.config_initialized:
            self.recalculate_derived_configs()

        return SetParametersResult(successful=True)

    def publish(self):
        """Publish currently stored variables.

        Note: Used only in BASIC mode.
        """
        self.pub.publish(self.msg)

    def publish_with_vesc(self):
        """Publish currently stored variables.

        Note: Used only in BASIC_VESC mode.
        """
        self.pub.publish(DriveValues(pwm_drive=self.config['pwm.throttle.calm_value'], pwm_angle=self.msg.pwm_angle))
        self.pub_vesc.publish(self.msg_vesc)

    def publish_sim(self):
        """Publish currently stored variables.

        Note: Used only in SIMULATION mode.
        """
        self.pub_cmd_vel.publish(self.msg_cmd_vel)

    def set_speed(self, speed: float, direction: ThrottleDirection, control_mode: ControlMode = ControlMode.LEGACY):
        """Set the car speed at desired value with given direction.

        Arguments:
        speed -- speed
        direction -- direction of movement

        Returns:
        success -- False if encountered errors, otherwise True

        ControlMode option:
        LEGACY -- speed is given from 0 to 1 (both included), direction
                  is specified via 'direction'
        JOINT -- speed is given from -1 to 1 (both included), direction
                 is expected to be 'FORWARD', otherwise it is flipped
        METRIC -- speed is given from unlimited range in [m.s^-1], direction
                  is expected to be 'FORWARD' otherwise it is flipped
        ANGULAR -- not supported
        """

        # skip if stopped
        if self.eStop:
            return False

        # validate arguments
        if not isinstance(control_mode, ControlMode):
            return False
        if not isinstance(direction, ThrottleDirection):
            return False

        # continue according to the selected mode
        if control_mode == ControlMode.LEGACY:

            # validate the speed
            if speed < 0 or speed > 1:
                return False

            if direction == ThrottleDirection.BACKWARD:
                if self.run_mode == RunMode.BASIC:
                    self.msg.pwm_drive = int(
                        self.config['pwm.throttle.backward.min']
                        + speed * self.config['pwm.throttle.backward.range']
                    )
                    return True

                if self.run_mode == RunMode.SIMULATION:
                    self.msg_cmd_vel.linear.x = -speed * self.config['simulation.throttle_modifier']
                    return True

                if self.run_mode == RunMode.BASIC_VESC:
                    self.msg_vesc.data = -speed * self.config['vesc.throttle.erpm_max']
                    return True

                return False

            if direction == ThrottleDirection.FORWARD:
                if self.run_mode == RunMode.BASIC:
                    self.msg.pwm_drive = int(
                        self.config['pwm.throttle.forward.min'] + speed
                        * self.config['pwm.throttle.forward.range']
                    )
                    return True

                if self.run_mode == RunMode.SIMULATION:
                    self.msg_cmd_vel.linear.x = speed * self.config['simulation.throttle_modifier']
                    return True

                if self.run_mode == RunMode.BASIC_VESC:
                    self.msg_vesc.data = speed * self.config['vesc.throttle.erpm_max']
                    return True

            return False

        elif control_mode == ControlMode.JOINT:

            # validate the speed
            if speed < -1 or speed > 1:
                return False

            if speed == 0:
                return self.stop()

            if (
                (direction == ThrottleDirection.FORWARD and speed > 0)
                or (direction == ThrottleDirection.BACKWARD and speed < 0)
            ):
                if self.run_mode == RunMode.BASIC:
                    self.msg.pwm_drive = int(
                        self.config['pwm.throttle.forward.min']
                        + abs(speed) * self.config['pwm.throttle.forward.range']
                    )
                    return True

                if self.run_mode == RunMode.SIMULATION:
                    self.msg_cmd_vel.linear.x = abs(speed) * self.config['simulation.throttle_modifier']
                    return True

                elif self.run_mode == RunMode.BASIC_VESC:
                    self.msg_vesc.data = abs(speed) * self.config['vesc.throttle.erpm_max']
                    return True

                return False

            if (
                (direction == ThrottleDirection.BACKWARD and speed > 0)
                or (direction == ThrottleDirection.FORWARD and speed < 0)
            ):
                if self.run_mode == RunMode.BASIC:
                    self.msg.pwm_drive = int(
                        self.config['pwm.throttle.backward.min']
                        + abs(speed) * self.config['pwm.throttle.backward.range']
                    )
                    return True

                if self.run_mode == RunMode.SIMULATION:
                    self.msg_cmd_vel.linear.x = -abs(speed) * self.config['simulation.throttle_modifier']
                    return True

                if self.run_mode == RunMode.BASIC_VESC:
                    self.msg_vesc.data = -abs(speed) * self.config['vesc.throttle.erpm_max']
                    return True

                return False

            return False

        if control_mode == ControlMode.METRIC:

            # ControlMode.METRIC is not available in RunMode.BASIC
            if self.run_mode == RunMode.BASIC:
                self.get_logger().info('set_speed: Unsupported run_mode for data in \'METRIC\' format.')
                return False

            if speed == 0:
                return self.stop()

            if self.run_mode == RunMode.SIMULATION:
                self.msg_cmd_vel.linear.x = speed
                return True

            if self.run_mode == RunMode.BASIC_VESC:
                self.msg_vesc.data = speed * self.config['vesc.throttle.to_erpm']
                return True

            return False

        return False

    def set_forward_speed(self, speed: float, control_mode: ControlMode = ControlMode.LEGACY):
        """Trampoline function for 'setSpeed()'."""
        return self.set_speed(speed, ThrottleDirection.FORWARD, control_mode)

    def set_backward_speed(self, speed: float, control_mode: ControlMode = ControlMode.LEGACY):
        """Trampoline function for 'setSpeed()'."""
        return self.set_speed(speed, ThrottleDirection.BACKWARD, control_mode)

    def stop(self):
        """Send a calm state speed. It means that the car stops."""

        self.get_logger().info(f'stop')

        self.msg.pwm_drive = self.config['pwm.throttle.calm_value']
        self.msg_cmd_vel.linear.x = 0.0
        self.msg_vesc.data = 0.0

        return True

    def set_steer(self, steer: float, direction: SteeringDirection, control_mode: ControlMode = ControlMode.LEGACY):
        """Set the car steering at desired value with given direction.

        Arguments:
        steer -- steer
        direction -- direction of movement

        Returns:
        success -- False if encountered errors, otherwise True

        ControlMode option:
        LEGACY -- steer is given from 0 to 1 (both included), direction
                  is specified via 'direction'
        JOINT -- steer is given from -1 to 1 (both included), direction
                 is expected to be 'LEFT', otherwise it is flipped
        METRIC -- not supported
        ANGULAR -- steer is given from unlimited range in [rad], but limited
                   by 'SERVO_LEFT_MAX' and 'SERVO_RIGHT_MAX' parameters,
                   direction is expected to be 'LEFT', otherwise it is flipped
        """

        # skip if stopped
        if self.eStop:
            return False

        # validate arguments
        if not isinstance(control_mode, ControlMode):
            return False
        if not isinstance(direction, SteeringDirection):
            return False

        # continue according to the selected mode
        if control_mode == ControlMode.LEGACY:

            # validate the steering
            if steer < 0 or steer > 1:
                return False

            # simulation mode
            if self.run_mode == RunMode.SIMULATION:
                if steer == 0:
                    self.reset_steer()
                else:
                    self.msg_cmd_vel.angular.z = \
                        steer * self.config['simulation.steering_modifier'] \
                        * (1.0 if direction == SteeringDirection.LEFT else -1.0)
                return True

            # other modes

            if direction == SteeringDirection.LEFT:
                self.msg.pwm_angle = int(
                    self.config['pwm.steering.left.min']
                    + steer * self.config['pwm.steering.left.range']
                )
                return True

            if direction == SteeringDirection.RIGHT:
                self.msg.pwm_angle = int(
                    self.config['pwm.steering.right.min']
                    + steer * self.config['pwm.steering.right.range']
                )
                return True

            return False

        elif control_mode == ControlMode.JOINT:

            # validate the steering
            if steer < -1 or steer > 1:
                return False

            # simulation mode
            if self.run_mode == RunMode.SIMULATION:
                if steer == 0:
                    self.reset_steer()
                else:
                    self.msg_cmd_vel.angular.z = \
                        abs(steer) * self.config['simulation.steering_modifier'] \
                        * (1.0 if direction == SteeringDirection.LEFT else -1.0)
                return True

            # Set corresponding variable
            if steer == 0:
                self.reset_steer()
                return True

            if (
                (direction == SteeringDirection.LEFT and steer > 0)
                or (direction == SteeringDirection.RIGHT and steer < 0)
            ):
                self.msg.pwm_angle = int(
                    self.config['pwm.steering.left.min']
                    + abs(steer) * self.config['pwm.steering.left.range']
                )
                return True

            if (
                (direction == SteeringDirection.RIGHT and steer > 0)
                or (direction == SteeringDirection.LEFT and steer < 0)
            ):
                self.msg.pwm_angle = int(
                    self.config['pwm.steering.right.min']
                    + abs(steer) * self.config['pwm.steering.right.range']
                )
                return True

            return False

        elif control_mode == ControlMode.ANGULAR:

            # simulation mode
            if self.run_mode == RunMode.SIMULATION:
                if steer == 0:
                    self.reset_steer()
                else:
                    self.msg_cmd_vel.angular.z = steer if direction == SteeringDirection.LEFT else -steer
                return True

            if steer == 0:
                self.reset_steer()
                return True

            if (
                (direction == SteeringDirection.LEFT and steer > 0)
                or (direction == SteeringDirection.RIGHT and steer < 0)
            ):
                self.msg.pwm_angle = int(
                    self.config['pwm.steering.left.min']
                    + min(abs(steer) / self.config['angular_steering.left_max'], 1.0)
                    * self.config['pwm.steering.left.range'])
                return True

            if (
                (direction == SteeringDirection.RIGHT and steer > 0)
                or (direction == SteeringDirection.LEFT and steer < 0)
            ):
                self.msg.pwm_angle = int(
                    self.config['pwm.steering.right.min']
                    + min(abs(steer) / self.config['angular_steering.right_max'], 1.0)
                    * self.config['pwm.steering.right.range']
                )
                return True

            return False

        return False

    def set_left_steer(self, steer: float, control_mode: ControlMode = ControlMode.LEGACY):
        """Trampoline function for 'setSteer()'."""
        return self.set_steer(steer, SteeringDirection.LEFT, control_mode)

    def set_right_steer(self, steer: float, control_mode: ControlMode = ControlMode.LEGACY):
        """Trampoline function for 'setSteer()'."""
        return self.set_steer(steer, SteeringDirection.RIGHT, control_mode)

    def reset_steer(self):
        """Send a calm state steer. It means that the car turns the wheels to go straight."""

        self.get_logger().info(f'reset_steer')

        self.msg.pwm_angle = self.config['pwm.steering.calm_value']
        self.msg_cmd_vel.angular.z = 0.0

        pass

    def api_callback(self, data: DriveApiValues):
        """Obtain requested speed/steering from ROS topic.

        Arguments:
        data -- structure received on topic /drive_api/command, defined by drive_api_values
        """

        if data.velocity < 0:
            self.stop()
        elif data.forward:
            self.set_forward_speed(data.velocity)
        else:
            self.set_backward_speed(data.velocity)

        if data.steering < 0:
            self.reset_steer()
        elif data.right:
            self.set_right_steer(data.steering)
        else:
            self.set_left_steer(data.steering)

        pass

    def command_callback(self, data: CommandArrayStamped):
        """Obtain requested speed/steering from ROS topic.

        Arguments:
        data -- structure received on topic /command, defined by CommandArrayStamped
        """

        # skip if stopped
        if self.eStop:
            return

        if len(data.commands) <= 0:
            return

        for c in data.commands:
            if c.command == 'speed' and len(c.parameters) > 0:
                for p in c.parameters:
                    success = False

                    # LEGACY
                    if p.parameter == 'forward':
                        if p.value < 0:
                            success = self.stop()
                        else:
                            success = self.set_forward_speed(p.value, ControlMode.LEGACY)
                    elif p.parameter == 'backward':
                        if p.value < 0:
                            success = self.stop()
                        else:
                            success = self.set_backward_speed(p.value, ControlMode.LEGACY)
                    # JOINT
                    elif p.parameter == 'norm' or p.parameter == 'joint':
                        success = self.set_forward_speed(p.value, ControlMode.JOINT)
                    # METRIC
                    elif p.parameter == 'metric' or p.parameter == 'm/s':
                        success = self.set_forward_speed(p.value, ControlMode.METRIC)

                    if not success:
                        self.get_logger().info(
                            f'Unable to process \'{c.command}\' parameter: {p.parameter} ({p.value:f})'
                        )
                        continue

                    break
            elif c.command == 'steer' and len(c.parameters) > 0:
                for p in c.parameters:
                    success = False

                    # LEGACY
                    if p.parameter == 'left':
                        if p.value < 0:
                            success = self.stop()
                        else:
                            success = self.set_left_steer(p.value, ControlMode.LEGACY)
                    elif p.parameter == 'right':
                        if p.value < 0:
                            success = self.stop()
                        else:
                            success = self.set_right_steer(p.value, ControlMode.LEGACY)
                    # JOINT
                    elif p.parameter == 'norm' or p.parameter == 'joint':
                        success = self.set_left_steer(p.value, ControlMode.JOINT)
                    # ANGULAR
                    elif p.parameter == 'deg':
                        success = self.set_left_steer(math.radians(p.value), ControlMode.ANGULAR)
                    elif p.parameter == 'rad':
                        success = self.set_left_steer(p.value, ControlMode.ANGULAR)

                    if not success:
                        self.get_logger().info(
                            f'Unable to process \'{c.command}\' parameter: {p.parameter} ({p.value:f})'
                        )
                        continue

                    break
            else:
                self.get_logger().info(f'command_callback: Unable to process command: \'{c.command}\'')

        pass

    def estop_callback(self, data: Bool):
        """Obtain emergency stop message from ROS topic.

        This message can be also used to stop/start simulation.

        Arguments:
        data -- structure received on topic /eStop, defined by std_msgs.msg/Bool
        """

        self.eStop = data.data

        self.get_logger().info(f'received eStop == {self.eStop}')

        if self.eStop:
            # Set calm states
            self.stop()
            self.reset_steer()

        pass


pass


# NOTE: See more info the the `main()` in the project README.md (the top-level one)
#       (section [Python entrypoints `main()` inconsistencies])
def main(args=None):
    if args is None:
        args = sys.argv

    rclpy.init(args=args)

    print(f'main args = {args}')

    node = None

    try:
        node = DriveApiNode()
        rclpy.spin(node)
    except InitError as e:
        print(f'InitError: {e}', file=sys.stderr)
        pass
    except KeyboardInterrupt:
        pass

    if node is not None:
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
