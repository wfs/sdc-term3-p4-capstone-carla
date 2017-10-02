import rospy

# import lowpass  # smooths PID reaction
import pid
from yaw_controller import YawController

GAS_DENSITY = 2.858
ONE_MPH = 0.44704
MAX_ACCEL_DECEL_TORQUE = 200.0


class Controller(object):
    """
    3. DWB node - Task 2:

    Fill out twist_controller.py, import PID, yaw_controller.

    Taking information from top level DBW to pass to PID and yaw_controller.
    """

    def __init__(self, wheel_base_from_ros_param_server, steer_ratio_from_ros_param_server,
                 max_lat_accel_from_ros_param_server, max_steer_angle_from_ros_param_server):
        """
        Take twist data as input to initialise pid and yaw_controller.
        """
        # Create PID steering correction controller
        max_steer_angle = max_steer_angle_from_ros_param_server
        self.steering_correction_pid = pid.PID(kp=0.2, ki=0.004, kd=3.0, mn=-max_steer_angle,
                                               mx=max_steer_angle)

        # Create yaw controller
        wb = wheel_base_from_ros_param_server
        sr = steer_ratio_from_ros_param_server
        mla = max_lat_accel_from_ros_param_server
        msa = max_steer_angle_from_ros_param_server

        self.yaw_controller = YawController(wb, sr, 0.0, mla, msa)

        self.timestamp = 0

    def control(self, cte, dbw_enabled):
        new_timestamp = rospy.get_time()
        duration = new_timestamp - self.timestamp
        sample_time = duration + 1e-6  # to avoid division by zero

        self.timestamp = new_timestamp
        if dbw_enabled:  # aka self-driving
            # calculate new steering angle
            steering_angle = self.steering_correction_pid.step(cte, sample_time)
            return steering_angle
        else:  # aka manual driving
            self.steering_correction_pid.reset()  # reset pids and filters
            return 0.0

    def control_speed_based_on_torque(self, target_linear_velocity, target_angular_velocity, current_velocity, transition_time,
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

        # Calc steering
        steer = self.yaw_controller.get_steering(throttle, target_angular_velocity, current_velocity)

        rospy.logwarn("twist_controller returning throttle %s, brake %s, steer %s adjustments to dbw_node",
                      throttle, brake, steer)

        return throttle, brake, steer
