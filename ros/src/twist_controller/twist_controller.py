import rospy

#import lowpass  # smooths PID reaction
import pid  # for acceleration
import yaw_controller  # for steering

GAS_DENSITY = 2.858
ONE_MPH = 0.44704
MAX_ACCEL_DECEL_TORQUE = 200.0


class Controller(object):
    """
    3. DWB node - Task 2:

    Fill out twist_controller.py, import PID, yaw_controller.

    Taking information from top level DBW to pass to PID and yaw_controller.
    """
    #see dbw_node.py for list e.g. car_mass, fuel_remaining, etc.
    def __init__(self,
                 maxiumum_steering_angle_from_ros_param_server,
                 wheel_base_from_ros_param_server,
                 steer_ratio_from_ros_param_server,
                 max_lat_accel_from_ros_param_server,
                 max_steer_angle_from_ros_param_server
                 ):
        """
        Take twist data as input to initialise pid and yaw_controller.
        """
        # Create PID steering correction controller
        max_steer_angle = maxiumum_steering_angle_from_ros_param_server
        self.steering_correction_pid = pid.PID(kp=0.2, ki=0.004, kd=3.0, mn=-max_steer_angle,
                                               mx=max_steer_angle)

        # Create yaw controller
        wb = wheel_base_from_ros_param_server
        sr = steer_ratio_from_ros_param_server
        mla = max_lat_accel_from_ros_param_server
        msa = max_steer_angle_from_ros_param_server

        self.yaw_controller = yaw_controller.YawController(wheel_base=wb, steer_ratio=sr, min_speed=0.0,
                                                           max_lat_accel=mla, max_steer_angle=msa)

        self.timestamp = 0


    def control(self,
                target_linear_velocity,
                target_angular_velocity,
                current_velocity,
                transition_time):
        """
        Manipulate inputs to return throttle, brake and steering values to dbw_node.
        """
        # Calc time differential
        time = rospy.get_time()
        delta_time = time - self.timestamp
        self.timestamp = delta_time

        # Calc throttle and brake :
        # how much do we need to adjust car velocity?
        velocity_change_required = target_linear_velocity - current_velocity
        # adjust acceleration within timebox
        acceleration = velocity_change_required / transition_time
        # check torque, where torque is vehicle mass * acceleration * (wheel radius, includes tire)
        torque = self.vehicle_mass * acceleration * self.wheel_radius

        throttle, brake = 0, 0
        if torque > 0:
            # ensure there is at least 1 KPH velocity
            throttle, brake = min(1.0, torque / MAX_ACCEL_DECEL_TORQUE), 0.0
        else:
            # apply brake within torque limit
            throttle, brake = 0.0, min(abs(torque), MAX_ACCEL_DECEL_TORQUE)

        # Calc steering
        steer = self.yaw_controller.get_steering(throttle, target_angular_velocity, current_velocity)

        # Return throttle, brake, steer
        #return 1., 0., 0.
        return throttle, brake, steer
