#!/usr/bin/env python3

"""
2D Controller Class to be used for the CARLA waypoint follower demo.
"""

import cutils
import numpy as np
import math

class Controller2D(object):
    def __init__(self, waypoints, control_method):
        self.vars                = cutils.CUtils()
        self._current_x          = 0
        self._current_y          = 0
        self._current_yaw        = 0
        self._current_speed      = 0
        self._desired_speed      = 0
        self._current_frame      = 0
        self._current_timestamp  = 0
        self._start_control_loop = False
        self._set_throttle       = 0
        self._set_brake          = 0
        self._set_steer          = 0
        self._waypoints          = waypoints
        self._conv_rad_to_steer  = 180.0 / 70.0 / np.pi
        self._pi                 = np.pi
        self._2pi                = 2.0 * np.pi
        self._kP                 = 0.8
        self._kI                 = 0.5
        self._kD                 = 0.0
        self._Kpp                = 4.5
        self._Kvf                = 2.5
        self._Kcte               = 0.5
        self._Kmpc               = 0.3
        self._eps_lookahead      = 10e-3
        self._closest_distance   = 0
        self._wheelbase          = 3.0
        self._control_method     = control_method
        self._steering_diff      = np.linspace(-10, 10, 21, endpoint = True)


    def update_values(self, x, y, yaw, speed, timestamp, frame, closest_distance):
        self._current_x         = x
        self._current_y         = y
        self._current_yaw       = yaw
        self._current_speed     = speed
        self._current_timestamp = timestamp
        self._current_frame     = frame
        self._closest_distance  = closest_distance
        if self._current_frame:
            self._start_control_loop = True

    def update_desired_speed(self):
        min_idx       = 0
        min_dist      = float("inf")
        desired_speed = 0
        for i in range(len(self._waypoints)):
            dist = np.linalg.norm(np.array([
                    self._waypoints[i][0] - self._current_x,
                    self._waypoints[i][1] - self._current_y]))
            if dist < min_dist:
                min_dist = dist
                min_idx = i
        if min_idx < len(self._waypoints)-1:
            desired_speed = self._waypoints[min_idx][2]
        else:
            desired_speed = self._waypoints[-1][2]
        self._desired_speed = desired_speed

    def update_waypoints(self, new_waypoints):
        self._waypoints = new_waypoints

    def get_commands(self):
        return self._set_throttle, self._set_steer, self._set_brake

    def set_throttle(self, input_throttle):
        # Clamp the throttle command to valid bounds
        throttle           = np.fmax(np.fmin(input_throttle, 1.0), 0.0)
        self._set_throttle = throttle

    def set_steer(self, input_steer_in_rad):
        # Covnert radians to [-1, 1]
        input_steer = self._conv_rad_to_steer * input_steer_in_rad

        # Clamp the steering command to valid bounds
        steer           = np.fmax(np.fmin(input_steer, 1.0), -1.0)
        self._set_steer = steer

    def set_brake(self, input_brake):
        # Clamp the steering command to valid bounds
        brake           = np.fmax(np.fmin(input_brake, 1.0), 0.0)
        self._set_brake = brake

    def calculate_throttle(self, t, v, v_desired):
        # Using PI Controller
        time_step = t - self.vars.t_previous
        speed_error = v_desired - v
        k_term = self._kP*speed_error
        i_term = self.vars.i_term_previous + self._kI*time_step*speed_error
        self.vars.i_term_previous = i_term
        return k_term + i_term

    def get_shifted_coordinate(self, x, y, yaw, length):
        x_shifted = x + length*np.cos(yaw)
        y_shifted = y + length*np.sin(yaw)
        return x_shifted, y_shifted

    def get_lookahead_dis(self, v):
        return self._Kvf*v

    def get_distance(self, x1, y1, x2, y2):
        return np.sqrt((x1 - x2)**2 + (y1 - y2)**2)

    def get_goal_waypoint_index(self, x, y, waypoints, lookahead_dis):
        for i in range(len(waypoints)):
            dis = self.get_distance(x, y, waypoints[i][0], waypoints[i][1])
            if abs(dis - lookahead_dis) <= self._eps_lookahead:
                return i
        return len(waypoints)-1

    def get_alpha(self, v1, v2, lookahead_dis):
        inner_prod = v1[0]*v2[0] + v1[1]*v2[1]
        return np.arccos(inner_prod/lookahead_dis)

    def get_steering_direction(self, v1, v2):
        corss_prod = v1[0]*v2[1] - v1[1]*v2[0]
        if corss_prod >= 0 :
            return -1
        return 1

    def get_heading_error(self, waypoints, current_yaw):
        waypoint_delta_x = waypoints[1][0] - waypoints[0][0]
        waypoint_delta_y = waypoints[1][1] - waypoints[0][1]
        waypoint_heading = np.arctan(waypoint_delta_y/waypoint_delta_x)
        heading_error_mod = divmod((waypoint_heading - current_yaw), np.pi)[1]
        if heading_error_mod > np.pi/2 and heading_error_mod < np.pi:
            heading_error_mod -= np.pi
        return heading_error_mod

    def get_cte_heading_error(self, v):
        proportional_cte_error = self._Kcte * self._closest_distance
        cte_heading_error = np.arctan(proportional_cte_error/v)
        cte_heading_error_mod = divmod(cte_heading_error, np.pi)[1]
        if cte_heading_error_mod > np.pi/2 and cte_heading_error_mod < np.pi:
            cte_heading_error_mod -= np.pi
        return cte_heading_error_mod

    def get_predicted_wheel_location(self, x, y, steering_angle, yaw, v):
        wheel_heading = yaw + steering_angle
        wheel_traveled_dis = v * (self._current_timestamp - self.vars.t_previous)
        return [x + wheel_traveled_dis * np.cos(wheel_heading), y + wheel_traveled_dis * np.sin(wheel_heading)]

    def calculate_steering(self, x, y, yaw, waypoints, v):
        if self._control_method == 'PurePursuit':
            lookahead_dis = self.get_lookahead_dis(v)
            idx = self.get_goal_waypoint_index(x, y, waypoints, lookahead_dis)
            v1 = [waypoints[idx][0] - x, waypoints[idx][1] - y]
            v2 = [np.cos(yaw), np.sin(yaw)]
            alpha = self.get_alpha(v1, v2, lookahead_dis)
            if math.isnan(alpha):
                alpha = self.vars.alpha_previous
            if not math.isnan(alpha):
                self.vars.alpha_previous = alpha
            steering = self.get_steering_direction(v1, v2)*np.arctan((2*self._wheelbase*np.sin(alpha))/(self._Kpp*v))
            if math.isnan(steering):
                steering = self.vars.steering_previous
            if not math.isnan(steering):
                self.vars.steering_previous = steering
            return steering
        elif self._control_method == 'Stanley':
            v1 = [waypoints[0][0] - x, waypoints[0][1] - y]
            v2 = [np.cos(yaw), np.sin(yaw)]
            heading_error = self.get_heading_error(waypoints, yaw)
            cte_error = self.get_steering_direction(v1, v2)*self.get_cte_heading_error(v)
            steering =  heading_error + cte_error
            return steering
        elif self._control_method == 'MPC':
            steering_list = self.vars.steering_previous + self._steering_diff * self._pi/180
            last_waypoint = [waypoints[int(self._Kmpc*len(waypoints))][0], waypoints[int(self._Kmpc*len(waypoints))][1]]
            min_dis = float("inf")
            steering = self.vars.steering_previous
            for i in range(len(steering_list)):
                predicted_wheel_location = self.get_predicted_wheel_location(x, y, steering_list[i], yaw, v)
                dis_to_last_waypoint = self.get_distance(predicted_wheel_location[0], predicted_wheel_location[1], last_waypoint[0], last_waypoint[1])
                if dis_to_last_waypoint < min_dis:
                    min_dis = dis_to_last_waypoint
                    steering = steering_list[i]
            return steering
        else:
            return 0

    def update_controls(self):
        ######################################################
        # RETRIEVE SIMULATOR FEEDBACK
        ######################################################
        x               = self._current_x
        y               = self._current_y
        yaw             = self._current_yaw
        v               = self._current_speed
        self.update_desired_speed()
        v_desired       = self._desired_speed
        t               = self._current_timestamp
        waypoints       = self._waypoints
        throttle_output = 0
        steer_output    = 0
        brake_output    = 0

        ######################################################
        ######################################################
        # MODULE 7: DECLARE USAGE VARIABLES HERE
        ######################################################
        ######################################################
        """
            Use 'self.vars.create_var(<variable name>, <default value>)'
            to create a persistent variable (not destroyed at each iteration).
            This means that the value can be stored for use in the next
            iteration of the control loop.

            Example: Creation of 'v_previous', default value to be 0
            self.vars.create_var('v_previous', 0.0)

            Example: Setting 'v_previous' to be 1.0
            self.vars.v_previous = 1.0

            Example: Accessing the value from 'v_previous' to be used
            throttle_output = 0.5 * self.vars.v_previous
        """
        self.vars.create_var('v_previous', 0.0)
        self.vars.create_var('t_previous',0.0)
        self.vars.create_var('i_term_previous',0.0)
        self.vars.create_var('alpha_previous',0.0)
        self.vars.create_var('steering_previous',0.0)

        # Skip the first frame to store previous values properly
        if self._start_control_loop:
            """
                Controller iteration code block.

                Controller Feedback Variables:
                    x               : Current X position (meters)
                    y               : Current Y position (meters)
                    yaw             : Current yaw pose (radians)
                    v               : Current forward speed (meters per second)
                    t               : Current time (seconds)
                    v_desired       : Current desired speed (meters per second)
                                      (Computed as the speed to track at the
                                      closest waypoint to the vehicle.)
                    waypoints       : Current waypoints to track
                                      (Includes speed to track at each x,y
                                      location.)
                                      Format: [[x0, y0, v0],
                                               [x1, y1, v1],
                                               ...
                                               [xn, yn, vn]]
                                      Example:
                                          waypoints[2][1]:
                                          Returns the 3rd waypoint's y position

                                          waypoints[5]:
                                          Returns [x5, y5, v5] (6th waypoint)

                Controller Output Variables:
                    throttle_output : Throttle output (0 to 1)
                    steer_output    : Steer output (-1.22 rad to 1.22 rad)
                    brake_output    : Brake output (0 to 1)
            """

            ######################################################
            ######################################################
            # MODULE 7: IMPLEMENTATION OF LONGITUDINAL CONTROLLER HERE
            ######################################################
            ######################################################
            """
                Implement a longitudinal controller here. Remember that you can
                access the persistent variables declared above here. For
                example, can treat self.vars.v_previous like a "global variable".
            """

            # Change these outputs with the longitudinal controller. Note that
            # brake_output is optional and is not required to pass the
            # assignment, as the car will naturally slow down over time.

            throttle_output = self.calculate_throttle(t, v, v_desired)
            brake_output    = 0

            ######################################################
            ######################################################
            # MODULE 7: IMPLEMENTATION OF LATERAL CONTROLLER HERE
            ######################################################
            ######################################################
            """
                Implement a lateral controller here. Remember that you can
                access the persistent variables declared above here. For
                example, can treat self.vars.v_previous like a "global variable".
            """
            #print("Goal point idx =",idx)
            # Change the steer output with the lateral controller.
            steer_output    = self.calculate_steering(x, y, yaw, waypoints, v)

            ######################################################
            # SET CONTROLS OUTPUT
            ######################################################
            self.set_throttle(throttle_output)  # in percent (0 to 1)
            self.set_steer(steer_output)        # in rad (-1yaw
            self.set_brake(brake_output)        # in percent (0 to 1)

        ######################################################
        ######################################################
        # MODULE 7: STORE OLD VALUES HERE (ADD MORE IF NECESSARY)
        ######################################################
        ######################################################
        """
            Use this block to store old values (for example, we can store the
            current x, y, and yaw values here using persistent variables for use
            in the next iteration)
        """
        self.vars.v_previous = v  # Store forward speed to be used in next step
        self.vars.t_previous = t


