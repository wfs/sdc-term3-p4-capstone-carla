# comment-out next lines when generating api docs
# *****
import rospy
# *****

from yaw_controller import YawController
import pid

# GAS_DENSITY = 2.858
# ONE_MPH = 0.44704
MAX_ACCEL_DECEL_TORQUE = 200.0
PREDICTIVE_STEERING = 1.0  # from 0.0 to 1.0


class Controller(object):
    """
    *** STEP 4 ***

    Adjusts vehicles steering to minimise Cross Track Error and align to predicted path.
    """

    def __init__(self, wheel_base_from_ros_param_server, steer_ratio_from_ros_param_server,
                 max_lat_accel_from_ros_param_server, max_steer_angle_from_ros_param_server):
        """
        Take twist data as input to initialise pid and yaw_controller.
        """
        # Create yaw controller
        wb = wheel_base_from_ros_param_server
        sr = steer_ratio_from_ros_param_server
        mla = max_lat_accel_from_ros_param_server
        msa = max_steer_angle_from_ros_param_server

        self.yaw_controller = YawController(wb, sr, 0.0, mla, msa)

        # Create / fine-tune PID steering controller
        # self.steering_correction_pid = pid.PID(kp=0.2, ki=0.004, kd=3.0, mn=-msa, mx=msa)
        # self.steering_correction_pid = pid.PID(kp=0.3, ki=0.004, kd=1.0, mn=-msa, mx=msa)
        # self.steering_correction_pid = pid.PID(kp=0.5, ki=0.004, kd=0.5, mn=-msa, mx=msa)
        self.steering_correction_pid = pid.PID(kp=0.5, ki=0.004, kd=0.25, mn=-msa, mx=msa)

        self.timestamp = rospy.get_time()

    def reset(self):
        self.steering_correction_pid.reset()

    def control(self, cte, dbw_enabled, twist_cmd_linear_velocity, twist_cmd_angular_velocity,
                current_linear_velocity):
        new_timestamp = rospy.get_time()
        duration = new_timestamp - self.timestamp

        self.timestamp = new_timestamp
        if dbw_enabled:  # aka self-driving
            # calculate new steering angle
            corrected_steering_angle = self.steering_correction_pid.step(cte, duration)

            # Get predicted steering angle from waypoints curve
            yaw_steer = self.yaw_controller.get_steering(twist_cmd_linear_velocity, twist_cmd_angular_velocity,
                                                         current_linear_velocity)

            steering_angle = corrected_steering_angle + PREDICTIVE_STEERING * yaw_steer

            return steering_angle
        else:  # aka manual driving
            self.steering_correction_pid.reset()  # reset pid

        return 0.0

    def control_speed_based_on_torque(self, target_linear_velocity, current_velocity, transition_time,
                                      vehicle_mass, wheel_and_tire_radius):
        """
        Manipulate inputs to return throttle, brake and steering values to dbw_node.
        """
        velocity_change_required = target_linear_velocity - current_velocity
        # adjust acceleration within timebox
        acceleration = velocity_change_required / transition_time
        # check torque, where torque is vehicle mass * acceleration * (wheel radius, includes tire)
        torque = vehicle_mass * acceleration * wheel_and_tire_radius

        throttle, brake = 0, 0
        if torque > 0:
            # ensure there is at least 1 KPH velocity
            throttle, brake = min(1.0, torque / MAX_ACCEL_DECEL_TORQUE), 0.0
        else:
            # apply brake within torque limit
            throttle, brake = 0.0, min(abs(torque), MAX_ACCEL_DECEL_TORQUE)

        return throttle, brake
