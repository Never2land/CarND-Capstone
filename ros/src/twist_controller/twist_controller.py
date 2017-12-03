from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController

GAS_DENSITY = 2.858
ONE_MPH = 0.44704

#Pamaters to be tuned
PID_KP = 0.7
PID_KI = 0.0007
PID_KD = 0.1

LPF_TAU = 0.1
LPF_TS = 0.2

YAW_MIN_SPEED = 1.0

class Controller(object):
    def __init__(self, **kwargs):
        # TODO: Implement

        self.params = kwargs
        self.torque = (self.params['vehicle_mass'] + self.params['fuel_capacity']*GAS_DENSITY) * self.params['wheel_radius']

        self.pid = PID(PID_KP, PID_KI, PID_KD, self.params['decel_limit'], self.params['accel_limit'])
        self.lpf= LowPassFilter(LPF_TAU, LPF_TS)
        self.yaw_controller = YawController(self.params['wheel_base'],
                                            self.params['steer_ratio'],
                                            YAW_MIN_SPEED,
                                            self.params['max_lat_accel'],
                                            self.params['max_steer_angle'])

        pass

    def control(self, dbw_enabled, twist_cmd_msg, current_velocity_msg, sample_time):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        if not dbw_enabled:
            self.pid.reset()
            return 0.0, 0.0, 0.0

        proposed_linear = twist_cmd_msg.twist.linear.x
        proposed_angular = twist_cmd_msg.twist.angular.z
        cur_linear = current_velocity_msg.twist.linear.x
        #cur_angular = current_velocity_msg.twist.angular.z

        linear_error = proposed_linear - cur_linear

        #rospy.loginfo('Linear error: %d', linear_error)
        print ("Linear error ", linear_error)

        #print("proposed_linear", proposed_linear, "current_linear", cur_linear)
        #print("linear_error", linear_error, "sample_time", sample_time);
        throttle = self.pid.step(linear_error, sample_time)
        #print("throttle after pid", throttle)
        throttle = self.lpf.filt(throttle)
        #print("throttle after lowpass", throttle)

        brake = 0.0
        if throttle < 0.0:
            if -throttle > self.params['brake_deadband']:
                brake = -self.torque * throttle
            throttle = 0.0

        steer = self.yaw_controller.get_steering(proposed_linear, proposed_angular, cur_linear)
        #return 1, 0 ,0
        return throttle, brake, steer
