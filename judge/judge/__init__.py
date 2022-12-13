#!/usr/bin/env python

# Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

# Allows controlling a vehicle with a keyboard. For a simpler and more
# documented example, please take a look at tutorial.py.

"""
Welcome to CARLA manual control.

Use ARROWS or WASD keys for control.

    W            : throttle
    S            : brake
    A/D          : steer left/right
    Q            : toggle reverse
    Space        : hand-brake
    P            : toggle autopilot
    M            : toggle manual transmission
    ,/.          : gear up/down
    CTRL + W     : toggle constant velocity mode at 60 km/h

    L            : toggle next light type
    SHIFT + L    : toggle high beam
    Z/X          : toggle right/left blinker
    I            : toggle interior light

    TAB          : change sensor position
    ` or N       : next sensor
    [1-9]        : change to sensor [1-9]
    G            : toggle radar visualization
    C            : change weather (Shift+C reverse)
    Backspace    : change vehicle

    O            : open/close all doors of vehicle
    T            : toggle vehicle's telemetry

    V            : Select next map layer (Shift+V reverse)
    B            : Load current selected map layer (Shift+B to unload)

    R            : toggle recording images to disk

    CTRL + R     : toggle recording of simulation (replacing any previous)
    CTRL + P     : start replaying last recorded simulation
    CTRL + +     : increments the start time of the replay by 1 second (+SHIFT = 10 seconds)
    CTRL + -     : decrements the start time of the replay by 1 second (+SHIFT = 10 seconds)

    F1           : toggle HUD
    H/?          : toggle help
    ESC          : quit
"""


from .main import main

if __name__ == "__main__":
    main()


# from carla import Client, Location, Rotation, Transform, Vector3D, World
# import random
# from argparse import ArgumentParser
# from enum import Enum
# import time
# from . import controller
# import math
# import os
# import pygame
# from pygame.locals import *


# class State(Enum):
#     RUNNING = 1
#     COLLISION = 2
#     FINISH = 3


# class Vehicle:
#     def __init__(self, world: World, init_trans: Transform):
#         # Create the vehicle
#         blu = world.get_blueprint_library().find("vehicle.tesla.model3")
#         vehicle = world.spawn_actor(blu, init_trans)

#         ## Add a collision sensor on the car
#         blu = world.get_blueprint_library().find("sensor.other.collision")
#         collision_detector = world.spawn_actor(blu, Transform(), attach_to=vehicle)
#         collision_detector.listen(on_collision)

#         ## Add a lidar on the student car
#         blu = world.get_blueprint_library().find("sensor.lidar.ray_cast")
#         blu.set_attribute("channels", "32")
#         blu.set_attribute("points_per_second", "600000")
#         blu.set_attribute("rotation_frequency", "10")
#         blu.set_attribute("range", str(LIDAR_RANGE_M))
#         lidar = world.spawn_actor(blu, Transform(), attach_to=vehicle)

#         ## Add a RGB camera on the car
#         blu = world.get_blueprint_library().find("sensor.camera.rgb")
#         blu.set_attribute("image_size_x", "1920")
#         blu.set_attribute("image_size_y", "1080")
#         blu.set_attribute("fov", "110")
#         blu.set_attribute(
#             "sensor_tick", "1.0"
#         )  # Set the time in seconds between sensor captures
#         camera = world.spawn_actor(blu, Transform(), attach_to=vehicle)

#         # assign fields
#         self.vehicle = vehicle
#         self.lidar = lidar
#         self.camera = camera
#         self.collision_detector = collision_detector

#         # Start listening to sensor events
#         # NOTE: It MUST be done after assigning self.xxx = xxx to avoid racing
#         lidar.listen(lambda event: self.on_lidar_event(event))
#         camera.listen(lambda event: self.on_camera_event(event))

#     def stop(self):
#         control = self.vehicle.get_control()
#         control.brake = 0.1
#         self.vehicle.apply_control(control)

#     def on_lidar_event(self, event):
#         controller.on_lidar_data(event, self.vehicle)

#     def on_camera_event(self, event):
#         print(event)
#         controller.on_camera_data(event, self.vehicle)


# ## Constant global variables
# WORLD = "Town07"
# SPEC_HEIGHT = 3
# MAX_TICK_COUNT = 600
# SPECTATOR_TRANS = Transform(
#     Location(74.6, -11.9, SPEC_HEIGHT), Rotation(-8.17, -73.632591, 0.0)
# )
# INIT_VEHICLE_TRANS = [
#     Transform(Location(74.6, -11.9, 3.0), Rotation(-8.17, -73.632591, 0.0)),
#     # Transform(Location(0, 0, 0.1), Rotation(0.0, 180.0, 0.0)),
# ]
# INIT_SPEED_KMPH = 20.0
# INIT_SPEED_MPS = INIT_SPEED_KMPH * 1000 / 3600
# SPEED_THRESH_KMPH = 25.0
# SPEED_THRESH_MPS = SPEED_THRESH_KMPH * 1000 / 3600
# VELOCIDY_THRESH = 1e-3
# LIDAR_RANGE_M = 30


# # Global variables to be initialized
# STATE = None
# VEHICLES = None
# SPECTATOR = None


# def main():
#     parser = ArgumentParser()
#     parser.add_argument("--addr", default="localhost", help="CARLA server address")
#     parser.add_argument("--port", default=2000, help="CARLA server port")
#     parser.add_argument("--no-follow-car", action="store_true")
#     args = parser.parse_args()

#     # Initialize pygame
#     pygame.init()
#     pygame.font.init()

#     ## Connect to the client and retrieve the world object
#     client = Client(args.addr, int(args.port))
#     client.load_world(WORLD)
#     world = client.get_world()

#     # Initialize the world
#     init_world(world)

#     try:
#         phase_1(world)
#         phase_2(world)
#         phase_3(world)
#     except KeyboardInterrupt:
#         print("INTERRUPTED")
#         return


# def init_world(world: World):
#     global STATE
#     global VEHICLES
#     global SPECTATOR

#     ## Enable synchronous mode
#     settings = world.get_settings()
#     # settings.synchronous_mode = True  # Enables synchronous mode
#     settings.fixed_delta_seconds = 0.05
#     world.apply_settings(settings)

#     # while True:
#     #     world.tick()
#     #     spectator = world.get_spectator()
#     #     print(spectator.get_transform())

#     # exit()

#     # Set initial state
#     STATE = State.RUNNING

#     ## Configure spectator
#     SPECTATOR = world.get_spectator()
#     SPECTATOR.set_transform(SPECTATOR_TRANS)

#     # Initialize vehicles
#     VEHICLES = list(init_vehicle(world, trans) for trans in INIT_VEHICLE_TRANS)


# def init_vehicle(world: World, init_trans: Transform):
#     vehicle = Vehicle(world, init_trans)
#     return vehicle


# def on_collision(event):
#     global STATE
#     if STATE == State.FINISH:
#         pass
#     else:
#         STATE = State.COLLISION


# def on_success():
#     global STATE
#     STATE = State.FINISH


# def on_fail():
#     global STATE
#     global VEHICLES

#     for veh in VEHICLES:
#         veh.stop()

#     STATE = State.FINISH


# def handle_lidar_event(event, vehicle):
#     controller.on_lidar_data(event, vehicle)


# def handle_camera_event(event, vehicle):
#     print(event)
#     controller.on_camera_data(event, vehicle)


# def check():
#     global STATE
#     global VEHICLES

#     ## Check speed limit
#     for veh in VEHICLES:
#         vel = veh.vehicle.get_velocity().length()
#         if vel > SPEED_THRESH_MPS:
#             print("FAIL: Speed exceeds {} km/s".format(SPEED_THRESH_KMPH))
#             on_fail()
#             return


# def phase_1(world: World):
#     ## Skip 10 frames (~1s for 20fps)
#     for _ in range(20):
#         world.tick()


# def phase_2(world: World):
#     global STATE
#     global VEHICLES

#     # for tick_count in range(MAX_TICK_COUNT):
#     while True:
#         for veh in VEHICLES:
#             controller.step(veh.vehicle)
#         world.tick()

#         ## Stick the spectator to the student car
#         # if not args.no_follow_car:
#         #     stu_trans = stu_car.get_transform()
#         #     spec_loc = Location(stu_trans.location.x, stu_trans.location.y, SPEC_HEIGHT)
#         #     spec_rot = stu_trans.rotation
#         #     spec_trans = Transform(spec_loc, spec_rot)
#         #     spec.set_transform(spec_trans)

#         ## Run state transition
#         if STATE == State.RUNNING:
#             check()

#         if STATE == State.RUNNING:
#             pass
#         elif STATE == State.COLLISION:
#             for veh in VEHICLES:
#                 veh.stop()
#             print("FAIL: Collision occurred")
#             STATE = State.FINISH

#         elif STATE == State.FINISH:
#             break

#         else:
#             assert False

#     ## Check timeout
#     if STATE == State.RUNNING:
#         print("FAIL: Timeout")
#         on_fail()
#         return


# def phase_3(world: World):
#     ## Tick forever until keyboard interrupt
#     try:
#         while True:
#             world.tick()
#     except KeyboardInterrupt:
#         pass


# if __name__ == "__main__":
#     main()
