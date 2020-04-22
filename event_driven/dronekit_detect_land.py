
# Copyright (c) Prokura Innovations. All rights reserved.
# Licensed under the MIT license. See LICENSE file in the project root for full license information.

import argparse
from dronekit import connect, VehicleMode
from pymavlink import mavutil
import exceptions
import socket
from enum import Enum
import numpy as np
import time
# import cv2
# del imports
import sys
from math import cos, sin, radians
import json
# flight state machine


class States(Enum):
    MANUAL = 0
    GUIDED = 1
    ARMING = 2
    TAKEOFF = 3
    WAYPOINT = 4
    LANDING = 5
    DISARMING = 6
    STOP = 7


class BackyardFlyer():
    def __init__(self, vehicle):
        print("inside initialization")
        self.target_position = np.array([0.0, 0.0, 0.0])
        self.all_waypoints = []
        self.in_mission = True
        self.check_state = {}
        self.vehicle = vehicle
        print(self.vehicle)

        # initial state
        self.vehicle.mode = VehicleMode('STABILIZE')
        # time.sleep(5)
        print(self.vehicle.mode)
        # while(vehicle.mode.name != 'STABILIZE'):
        #     pass
        self.flight_state = States.MANUAL

        # register callbacks
        self.vehicle.add_attribute_listener(
            'location.local_frame', self.local_location_callback)
        self.vehicle.add_attribute_listener('velocity', self.velocity_callback)
        self.vehicle.add_attribute_listener('armed', self.state_callback)
        self.vehicle.add_attribute_listener('mode', self.state_callback)

        self.vehicle.mode = VehicleMode('GUIDED')
        # time.sleep(5)
        print(self.vehicle.is_armable)
        print(self.vehicle.mode)

    def goto_position_target_local_ned(self, north, east, down):
        """
        Send SET_POSITION_TARGET_LOCAL_NED command to request the vehicle fly to a specified
        location in the North, East, Down frame.

        It is important to remember that in this frame, positive altitudes are entered as negative
        "Down" values. So if down is "10", this will be 10 metres below the home altitude.

        Starting from AC3.3 the method respects the frame setting. Prior to that the frame was
        ignored. For more information see:
        http://dev.ardupilot.com/wiki/copter-commands-in-guided-mode/#set_position_target_local_ned

        See the above link for information on the type_mask (0=enable, 1=ignore).
        At time of writing, acceleration and yaw bits are ignored.

        """
        # print (self)
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,       # time_boot_ms (not used)
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
            0b0000111111111000,  # type_mask (only positions enabled)
            # x, y, z positions (or North, East, Down in the MAV_FRAME_BODY_NED frame
            north, east, down,
            0, 0, 0,  # x, y, z velocity in m/s  (not used)
            # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0, 0,
            0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
        # send command to vehicle
        self.vehicle.send_mavlink(msg)

    def send_ned_velocity(self, velocity_x, velocity_y, velocity_z, duration):
        """
        Move vehicle in direction based on specified velocity vectors and
        for the specified duration.

        This uses the SET_POSITION_TARGET_LOCAL_NED command with a type mask enabling only
        velocity components
        (http://dev.ardupilot.com/wiki/copter-commands-in-guided-mode/#set_position_target_local_ned).

        Note that from AC3.3 the message should be re-sent every second (after about 3 seconds
        with no message the velocity will drop back to zero). In AC3.2.1 and earlier the specified
        velocity persists until it is canceled. The code below should work on either version
        (sending the message multiple times does not cause problems).

        See the above link for information on the type_mask (0=enable, 1=ignore).
        At time of writing, acceleration and yaw bits are ignored.
        """
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,       # time_boot_ms (not used)
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
            0b0000111111000111,  # type_mask (only speeds enabled)
            0, 0, 0,  # x, y, z positions (not used)
            velocity_x, velocity_y, velocity_z,  # x, y, z velocity in m/s
            # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0, 0,
            0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

        # send command to vehicle on 1 Hz cycle
        for x in range(0, duration):
            self.vehicle.send_mavlink(msg)
            time.sleep(1)

    def send_ned_wo_duration(self, velocity_x, velocity_y, velocity_z):
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,       # time_boot_ms (not used)
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
            0b0000111111000111,  # type_mask (only speeds enabled)
            0, 0, 0,  # x, y, z positions (not used)
            velocity_x, velocity_y, velocity_z,  # x, y, z velocity in m/s
            # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0, 0,
            0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
        self.vehicle.send_mavlink(msg)

    def local_location_callback(self, _, attr_name, value):
        if self.flight_state == States.TAKEOFF:
            altitude = -1.0 * value.down
            if altitude > 0.95 * self.target_position[2]:
                self.all_waypoints = [
                    (0.0, 0.0, 15.0, 0.0), (5.0, 0.0, 15.0, 0), (5.0, 5.0, 15.0, 0), (0.0, 5.0, 15.0, 0.0)]
                self.waypoint_transition()
        if self.flight_state == States.WAYPOINT:
            north = value.north
            east = value.east
            if abs(north - self.target_position[0]) < 1.0 and abs(east - self.target_position[1]) < 1.0:
                print('I reached waypoint.')
                print(self.check_state[0])
                if self.check_state[0] == True:
                    # print('check state is true')
                    self.landing_transition()
                else:
                    self.waypoint_transition()

    def velocity_callback(self, _, attr_name, value):
        # pass
        # print("inside velocity_callback")
        # print('**************')
        if self.flight_state == States.LANDING:
            print(self.vehicle.location.global_frame.alt, self.vehicle.home_location.alt,
                  self.vehicle.location.global_relative_frame.alt)
            if ((self.vehicle.location.global_frame.alt - self.vehicle.home_location.alt < 0.8) and
                    abs(self.vehicle.location.local_frame.down) < 0.8):
                self.disarming_transition()
            else:
                self.landing_transition()
        # quit_me = input("ghcvghf")
        # if quit_me == "q":
        #     self.disarming_transition()
        #     return
        # cmds = self.vehicle.commands
        # cmds.download()
        # cmds.wait_ready()

    def state_callback(self, _, attr_name, value):
        print("inside state_callback")
        # print(self.flight_state)
        if not self.in_mission:
            print('not in mission')
            if self.vehicle.mode.name == 'STABILIZE':
                self.stop_transition()
        elif self.flight_state == States.MANUAL and self.vehicle.is_armable:
            # print('babu bhaiya')
            self.arming_transition()
        elif self.flight_state == States.ARMING:
            if self.vehicle.armed and self.vehicle.mode.name == 'GUIDED':
                self.takeoff_transition()
        elif self.flight_state == States.DISARMING:
            if not self.vehicle.armed:
                print('i am now going manual')
                self.manual_transition()

    def calculate_box(self):
        print("calculate_box")
        return self.all_waypoints.pop()

    def arming_transition(self):
        print("arming transition")
        self.vehicle.home_location = self.vehicle.location.global_frame
        self.flight_state = States.ARMING
        self.vehicle.mode = VehicleMode('GUIDED')
        self.vehicle.armed = True

    def takeoff_transition(self):
        print('takeoff transition')
        print(self.vehicle.location.local_frame)
        target_altitude = 15.0
        self.target_position[2] = target_altitude
        self.vehicle.simple_takeoff(target_altitude)
        self.flight_state = States.TAKEOFF

    def waypoint_transition(self):
        print('waypoint transition')
        if len(self.all_waypoints) != 0:
            self.check_state[0] = False
            box_cord = self.calculate_box()
            self.target_position[:] = box_cord[:3]
            print(self.target_position)
            self.goto_position_target_local_ned(
                north=self.target_position[0], east=self.target_position[1], down=-1.0 * self.target_position[2])
            self.flight_state = States.WAYPOINT
        else:
            print('mission complete')
            self.check_state[0] = True
            # self.flight_state = States.WAYPOINT

    def pid_controller(self,
                       z_target,
                       z_actual,
                       z_dot_target,
                       z_dot_actual,
                       dt=0.1,
                       z_dot_dot_ff=0.0):

        err = z_target - z_actual
        err_dot = z_dot_target - z_dot_actual
        self.integrated_error += err * dt

        p = self.k_p * err
        i = self.integrated_error * self.k_i
        d = self.k_d * err_dot

        u_bar = p + i + d + z_dot_dot_ff
        u = self.vehicle_mass * (self.g - u_bar)
        return u

    def find_vel(self, angle, curr_vel_x, cur_vel_y):
        pass

    def landing_transition(self):
        print('landing transition')
        self.flight_state = States.LANDING
        # self.vehicle.mode = VehicleMode('LAND')
        # print(self.vehicle.mode)
        down_speed = 0.5  # should be greater than zero to descend
        duration = 5
        angle = 0
        velocity = 2
        while True:
            try:
                with open('test.txt') as f:
                    json_data = json.load(f)
                    angle = int(json_data)
                    print(json_data)
                    break
            except:
                continue
        angle_rad = radians(angle)
        vel_x = velocity*cos(angle_rad)
        vel_y = velocity*sin(angle_rad)
        print('vel_x:'+str(vel_x), 'vel_y:'+str(vel_y))
        self.send_ned_wo_duration(velocity_x=velocity*cos(angle_rad), velocity_y=velocity*sin(angle_rad),
                                  velocity_z=0)
        # self.send_ned_wo_duration(0, 0, 0, 1)

    def disarming_transition(self):
        print('disarm transition')
        # print(self.vehicle.location.local_frame)
        self.flight_state = States.DISARMING
        self.vehicle.armed = False
        cmds = vehicle.commands
        cmds.download()
        cmds.wait_ready()
        print('*********')
        print(self.vehicle.armed)
        # print(self.vehicle.armed)
        # if self.vehicle.armed == True:
        #     self.vehicle.armed = False
        #     print(self.vehicle.armed)
        #     self.disarming_transition()
        # else:
        #     return
        # print(self.vehicle)

    def manual_transition(self):
        print('manual transition')
        self.flight_state = States.MANUAL
        self.in_mission = False
        print(self.vehicle.mode)
        self.vehicle.mode = VehicleMode('STABILIZE')
        # time.sleep(2)
        # while self.vehicle.mode.name != 'STABILIZE':
        #     print(self.vehicle.mode.name)
        #     pass

    def stop_transition(self):
        print('stop transition')
        self.flight_state = States.STOP
        # self.vehicle.close()
        print(bool(self.vehicle))


# Set up option parsing to get connection string
parser = argparse.ArgumentParser(
    description='Control Copter and send commands in GUIDED mode ')
parser.add_argument('--connect',
                    help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect
sitl = None


# Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()

# Connect to the Vehicle
try:
    print('Connecting to vehicle on: %s' % connection_string)
    vehicle = connect(connection_string, wait_ready=True)
    # vehicle = connect('tcp:127.0.0.1:5762', wait_ready=True)

    # print(bool(vehicle))
# Bad TCP connection
except socket.error:
    print 'No server exists!'

# Bad TTY connection
except exceptions.OSError as e:
    print 'No serial exists!'

# API Error
except dronekit.APIException:
    print 'Timeout!'

# Other error
except:
    print 'Some other error!'

# print(vehicle.mode)
# print(vehicle.location.local_frame)
# print(vehicle.armed)
# print(vehicle.velocity)
# print(vehicle)
drone = BackyardFlyer(vehicle)

# sys.exit(0)
# ho ya nira kasari BackyardFlyer class ko code execute hune banunu
# print('*************\t', drone.flight_state)
# img = cv2.imread('a.jpg')
# cv2.imshow('dummy', img)
while drone.flight_state != States.STOP:
    pass

    # if cv2.waitKey(1) & 0xFF == ord('q'):
    #     break


sys.exit(0)
# time.sleep(1000)
