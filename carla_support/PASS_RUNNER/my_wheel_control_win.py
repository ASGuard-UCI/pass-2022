#!/usr/bin/env python

# Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

# Allows controlling a vehicle with a Wheel. For a simpler and more
# documented example, please take a look at tutorial.py.

"""
Welcome to wheel control.

Use ARROWS or WASD keys for control.

    right pedal  : throttle
    left pedal   : brake
    wheel        : steer left/right
    L1/R1        : toggle reverse
    TRIANGLE     : respawn car

    L            : toggle next light type
    SHIFT + L    : toggle high beam
    Z/X          : toggle right/left blinker
    I            : toggle interior light

    up           : change camera position

    R            : toggle recording images to disk

    CTRL + R     : toggle recording of simulation (replacing any previous)
    CTRL + P     : start replaying last recorded simulation
    CTRL + +     : increments the start time of the replay by 1 second (+SHIFT = 10 seconds)
    CTRL + -     : decrements the start time of the replay by 1 second (+SHIFT = 10 seconds)

    F1           : toggle HUD
    H/?          : toggle help
    ESC          : quit
"""

from __future__ import print_function

# ==============================================================================
# -- imports -------------------------------------------------------------------
# ==============================================================================
import io
import ntpath

import carla
from carla import ColorConverter as cc

import argparse
import os
import sys
import time
import collections
import datetime
import logging
import math
import weakref

import xml.etree.ElementTree as ET

from srunner.tools.scenario_parser import ScenarioConfigurationParser
from configparser import ConfigParser
# from evdev import ecodes, InputDevice, ff

# for windows usb devices
# import hid

try:
    import pygame
    import pygame.gfxdraw
    from pygame.locals import K_ESCAPE
    from pygame.locals import K_F1
    from pygame.locals import KMOD_CTRL
    from pygame.locals import KMOD_SHIFT
    from pygame.locals import K_BACKSPACE
    from pygame.locals import K_TAB
    from pygame.locals import K_SPACE
    from pygame.locals import K_UP
    from pygame.locals import K_DOWN
    from pygame.locals import K_LEFT
    from pygame.locals import K_RIGHT
    from pygame.locals import K_w
    from pygame.locals import K_a
    from pygame.locals import K_s
    from pygame.locals import K_d
    from pygame.locals import K_q
    from pygame.locals import K_m
    from pygame.locals import K_COMMA
    from pygame.locals import K_PERIOD
    from pygame.locals import K_p
    from pygame.locals import K_i
    from pygame.locals import K_l
    from pygame.locals import K_z
    from pygame.locals import K_x
    from pygame.locals import K_r
    from pygame.locals import K_MINUS
    from pygame.locals import K_EQUALS
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

try:
    import numpy as np
except ImportError:
    raise RuntimeError('cannot import numpy, make sure numpy package is installed')


# ==============================================================================
# -- Global functions ----------------------------------------------------------
# ==============================================================================

def get_actor_display_name(actor, truncate=250):
    name = ' '.join(actor.type_id.replace('_', '.').title().split('.')[1:])
    return (name[:truncate - 1] + u'\u2026') if len(name) > truncate else name


# ==============================================================================
# -- World ---------------------------------------------------------------------
# ==============================================================================

class World(object):

    restarted = False

    def __init__(self, carla_world, hud, args):
        self.world = carla_world
        try:
            self.map = self.world.get_map()
        except RuntimeError as error:
            print('RuntimeError: {}'.format(error))
            print('  The server could not send the OpenDRIVE (.xodr) file:')
            print('  Make sure it exists, has the same name of your town, and is correct.')
            sys.exit(1)
        self.hud = hud
        self.player = None
        self.collision_sensor = None
        self.lane_invasion_sensor = None
        self.gnss_sensor = None
        self.imu_sensor = None
        self.radar_sensor = None
        self.camera_manager = None
        self.restart()
        self.world.on_tick(hud.on_world_tick)
        self.recording_enabled = False
        self.recording_start = 0

        # scenario object from leaderboard via Pipe connection
        self.c_conn = args.c_conn

    def restart(self):

        if self.restarted:
            return
        self.restarted = True

        self.player_max_speed = 1.589
        self.player_max_speed_fast = 3.713

        # Keep same camera config if the camera manager exists.
        cam_index = self.camera_manager.index if self.camera_manager is not None else 2
        cam_pos_index = self.camera_manager.transform_index if self.camera_manager is not None else 2

        # Get the ego vehicle
        while self.player is None:
            print("Waiting for the ego vehicle...")
            time.sleep(1)
            possible_vehicles = self.world.get_actors().filter('vehicle.lincoln.*')
            for vehicle in possible_vehicles:
                if vehicle.attributes['role_name'] == 'hero':
                    print("Ego vehicle found")
                    self.player = vehicle
                    break
        
        self.player_name = self.player.type_id

        # Set up the sensors.
        self.collision_sensor = CollisionSensor(self.player, self.hud)
        self.lane_invasion_sensor = LaneInvasionSensor(self.player, self.hud)
        self.gnss_sensor = GnssSensor(self.player)
        self.imu_sensor = IMUSensor(self.player)
        self.camera_manager = CameraManager(self.player, self.hud)
        self.camera_manager.transform_index = cam_pos_index
        self.camera_manager.set_sensor(cam_index, notify=False)
        actor_type = get_actor_display_name(self.player)
        self.hud.notification(actor_type)

        self.world.wait_for_tick()

    def tick(self, clock):
        if len(self.world.get_actors().filter(self.player_name)) < 1:
            return False

        self.hud.tick(self, clock)
        return True

    def render(self, display):
        self.camera_manager.render(display)
        # self.hud.render(display)
        self.hud.render_gauge(display, self.c_conn.recv())

    def destroy_sensors(self):
        self.camera_manager.sensor.destroy()
        self.camera_manager.sensor = None
        self.camera_manager.index = None

    def destroy(self):
        sensors = [
            self.camera_manager.sensor,
            self.collision_sensor.sensor,
            self.lane_invasion_sensor.sensor,
            self.gnss_sensor.sensor,
            self.imu_sensor.sensor]
        for sensor in sensors:
            if sensor is not None:
                sensor.stop()
                sensor.destroy()
        if self.player is not None:
            self.player.destroy()

        # write history waypoints, for route generation
        waypoints = ET.Element('waypoints')
        for wp in self.hud._passed_waypoints:
            waypoint = ET.SubElement(waypoints, 'waypoint')
            waypoint.set('pitch', '0.0')
            waypoint.set('roll', '0.0')
            waypoint.set('yaw', '{}'.format(wp.transform.rotation.yaw))
            waypoint.set('x', '{}'.format(wp.transform.location.x))
            waypoint.set('y', '{}'.format(wp.transform.location.y))
            waypoint.set('z', '{}'.format(wp.transform.location.z))
        # wp = self.hud._passed_waypoints[-1]
        # waypoint = ET.SubElement(waypoints, 'waypoint')
        # waypoint.set('pitch', '0.0')
        # waypoint.set('roll', '0.0')
        # waypoint.set('yaw', '{}'.format(wp.transform.rotation.yaw))
        # waypoint.set('x', '{}'.format(wp.transform.location.x))
        # waypoint.set('y', '{}'.format(wp.transform.location.y))
        # waypoint.set('z', '{}'.format(wp.transform.location.z))
        data_string = ET.tostring(waypoints, encoding='unicode')
        with open('_out/wheel_passed_waypoints.xml', 'w') as _f:
            _f.write(data_string)


# ==============================================================================
# -- WheelControl -----------------------------------------------------------
# ==============================================================================


class WheelControl(object):
    """Class that handles Wheel input."""
    def __init__(self, world, start_in_autopilot, args):
        self._autopilot_enabled = start_in_autopilot
        self._control = carla.VehicleControl()
        self._lights = carla.VehicleLightState.NONE
        self._steer_cache = 0.0
        self._args = args
        world.player.set_autopilot(self._autopilot_enabled)
        world.player.set_light_state(self._lights)
        world.hud.notification("Press 'H' or '?' for help.", seconds=4.0)


        # initialize steering wheel
        pygame.joystick.init()
        joystick_count = pygame.joystick.get_count()
        if joystick_count > 1:
            raise ValueError("Please Connect Just One Wheel joystick")
        if joystick_count < 1:
            raise ValueError("No wheel connected")

        self._joystick = pygame.joystick.Joystick(0)
        self._joystick.init()

        # evdev references to the steering wheel (force feedback)
        # self._device = evdev.list_devices()[0]
        # self._evtdev = InputDevice(self._device)
        # self._evtdev.write(ecodes.EV_FF, ecodes.FF_AUTOCENTER, int(65535 * .75))
        # time.sleep(1)

        # self._vid, self._pid = 0x044f, 0xb66e  # this id can be found in device manager
        # self._device = hid.device()
        # self._device.open(self._vid, self._pid)

        # read wheel config
        self._parser = ConfigParser()
        self._parser.read('wheel_config.ini')
        self._steer_idx = int(
            self._parser.get('Thrustmaster FFB Wheel', 'steering_wheel'))
        self._throttle_idx = int(
            self._parser.get('Thrustmaster FFB Wheel', 'throttle'))
        self._brake_idx = int(self._parser.get('Thrustmaster FFB Wheel', 'brake'))
        self._reverse_idx = int(self._parser.get('Thrustmaster FFB Wheel', 'reverse'))
        self._reverse2_idx = int(self._parser.get('Thrustmaster FFB Wheel', 'reverse2'))
        self._handbrake_idx = int(self._parser.get('Thrustmaster FFB Wheel', 'handbrake'))
        self._respawn_idx = int(self._parser.get('Thrustmaster FFB Wheel', 'respawn'))

        # sound effects
        self._snd_dir = os.path.join('map_resources', 'sound')
        # self.car_sound_array_idle, self.sample_rate = sf.read(os.path.join(self._snd_dir, 'Engine_Speed_01_Loop.WAV'))
        # self.car_sound_array_low, self.sample_rate = sf.read(os.path.join(self._snd_dir, 'Engine_Speed_03_Loop.WAV'))
        # self.car_sound_array_high, self.sample_rate = sf.read(os.path.join(self._snd_dir, 'Engine_Speed_05_Loop.WAV'))
        _car_sound = pygame.mixer.Sound(os.path.join(self._snd_dir, 'Engine_Speed_01_Loop.WAV'))
        self.car_sound_idle = _car_sound
        self.car_sound_array_idle = np.frombuffer(_car_sound.get_raw(), dtype=np.float32)
        _car_sound = pygame.mixer.Sound(os.path.join(self._snd_dir, 'Engine_Speed_03_Loop.WAV'))
        self.car_sound_array_low = np.frombuffer(_car_sound.get_raw(), dtype=np.float32)
        _car_sound = pygame.mixer.Sound(os.path.join(self._snd_dir, 'Engine_Speed_05_Loop.WAV'))
        self.car_sound_array_high = np.frombuffer(_car_sound.get_raw(), dtype=np.float32)
        self.sound_played_time = 0
        self._bgm_dir = os.path.join(self._snd_dir, 'Cycles.mp3')
        self.speed_jump = False

    def parse_events(self, client, world, clock):
        current_lights = self._lights
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return True
            # reset cars
            elif event.type == pygame.JOYBUTTONDOWN:
                if event.button == self._respawn_idx:
                    # scenario_configurations = ScenarioConfigurationParser.parse_scenario_configuration(
                    #     self._args.scenario, '')
                    # start_transform = None
                    # for config in scenario_configurations:
                    #     for ego_vehicle in config.ego_vehicles:
                    #         if 'lincoln' in ego_vehicle.model:
                    #             start_transform = ego_vehicle.transform
                        # teleport
                        # world.player.set_transform(start_transform)
                    # teleport to the npc
                    vehicles = world.world.get_actors().filter('vehicle.*')
                    npc_vehicle = [x for x in vehicles if x.id != world.player.id][0]
                    tele_transform = npc_vehicle.get_transform()
                    tele_transform.location -= 10 * tele_transform.get_forward_vector()
                    world.player.set_transform(tele_transform)


                elif event.button == self._reverse_idx or event.button == self._reverse2_idx:
                    self._control.gear = 1 if self._control.reverse else -1
                elif event.button == self._handbrake_idx:
                    self._control.gear = 1 if self._control.reverse else -1
            # toggle camera
            elif event.type == pygame.JOYHATMOTION:
                world.camera_manager.toggle_camera()


            elif event.type == pygame.KEYUP:
                if self._is_quit_shortcut(event.key):
                    return True
                elif event.key == K_BACKSPACE:
                    if self._autopilot_enabled:
                        world.player.set_autopilot(False)
                        world.restart()
                        world.player.set_autopilot(True)
                    else:
                        world.restart()
                elif event.key == K_F1:
                    world.hud.toggle_info()
                elif event.key == K_TAB:
                    world.camera_manager.toggle_camera()
                elif event.key == K_r and not (pygame.key.get_mods() & KMOD_CTRL):
                    world.camera_manager.toggle_recording()
                elif event.key == K_r and (pygame.key.get_mods() & KMOD_CTRL):
                    if (world.recording_enabled):
                        client.stop_recorder()
                        world.recording_enabled = False
                        world.hud.notification("Recorder is OFF")
                    else:
                        client.start_recorder("manual_recording.rec")
                        world.recording_enabled = True
                        world.hud.notification("Recorder is ON")
                elif event.key == K_p and (pygame.key.get_mods() & KMOD_CTRL):
                    # stop recorder
                    client.stop_recorder()
                    world.recording_enabled = False
                    # work around to fix camera at start of replaying
                    current_index = world.camera_manager.index
                    world.destroy_sensors()
                    # disable autopilot
                    self._autopilot_enabled = False
                    world.player.set_autopilot(self._autopilot_enabled)
                    world.hud.notification("Replaying file 'manual_recording.rec'")
                    # replayer
                    client.replay_file("manual_recording.rec", world.recording_start, 0, 0)
                    world.camera_manager.set_sensor(current_index)
                elif event.key == K_MINUS and (pygame.key.get_mods() & KMOD_CTRL):
                    if pygame.key.get_mods() & KMOD_SHIFT:
                        world.recording_start -= 10
                    else:
                        world.recording_start -= 1
                    world.hud.notification("Recording start time is %d" % (world.recording_start))
                elif event.key == K_EQUALS and (pygame.key.get_mods() & KMOD_CTRL):
                    if pygame.key.get_mods() & KMOD_SHIFT:
                        world.recording_start += 10
                    else:
                        world.recording_start += 1
                    world.hud.notification("Recording start time is %d" % (world.recording_start))
                elif event.key == K_q:
                    self._control.gear = 1 if self._control.reverse else -1
                elif event.key == K_m:
                    self._control.manual_gear_shift = not self._control.manual_gear_shift
                    self._control.gear = world.player.get_control().gear
                    world.hud.notification('%s Transmission' %
                                            ('Manual' if self._control.manual_gear_shift else 'Automatic'))
                elif self._control.manual_gear_shift and event.key == K_COMMA:
                    self._control.gear = max(-1, self._control.gear - 1)
                elif self._control.manual_gear_shift and event.key == K_PERIOD:
                    self._control.gear = self._control.gear + 1
                elif event.key == K_p and not pygame.key.get_mods() & KMOD_CTRL:
                    self._autopilot_enabled = not self._autopilot_enabled
                    world.player.set_autopilot(self._autopilot_enabled)
                    world.hud.notification(
                        'Autopilot %s' % ('On' if self._autopilot_enabled else 'Off'))
                elif event.key == K_l and pygame.key.get_mods() & KMOD_CTRL:
                    current_lights ^= carla.VehicleLightState.Special1
                elif event.key == K_l and pygame.key.get_mods() & KMOD_SHIFT:
                    current_lights ^= carla.VehicleLightState.HighBeam
                elif event.key == K_l:
                    # Use 'L' key to switch between lights:
                    # closed -> position -> low beam -> fog
                    if not self._lights & carla.VehicleLightState.Position:
                        world.hud.notification("Position lights")
                        current_lights |= carla.VehicleLightState.Position
                    else:
                        world.hud.notification("Low beam lights")
                        current_lights |= carla.VehicleLightState.LowBeam
                    if self._lights & carla.VehicleLightState.LowBeam:
                        world.hud.notification("Fog lights")
                        current_lights |= carla.VehicleLightState.Fog
                    if self._lights & carla.VehicleLightState.Fog:
                        world.hud.notification("Lights off")
                        current_lights ^= carla.VehicleLightState.Position
                        current_lights ^= carla.VehicleLightState.LowBeam
                        current_lights ^= carla.VehicleLightState.Fog
                elif event.key == K_i:
                    current_lights ^= carla.VehicleLightState.Interior
                elif event.key == K_z:
                    current_lights ^= carla.VehicleLightState.LeftBlinker
                elif event.key == K_x:
                    current_lights ^= carla.VehicleLightState.RightBlinker

        if not self._autopilot_enabled:
            self._parse_vehicle_keys(pygame.key.get_pressed(), clock.get_time())
            self._parse_vehicle_wheel()
            self._parse_speedToWheel(world)
            self._parse_speedToSound(world, clock)
            self._control.reverse = self._control.gear < 0
            # Set automatic control-related vehicle lights
            if self._control.brake:
                current_lights |= carla.VehicleLightState.Brake
            else: # Remove the Brake flag
                current_lights &= ~carla.VehicleLightState.Brake
            if self._control.reverse:
                current_lights |= carla.VehicleLightState.Reverse
            else: # Remove the Reverse flag
                current_lights &= ~carla.VehicleLightState.Reverse
            if current_lights != self._lights: # Change the light state only if necessary
                self._lights = current_lights
                world.player.set_light_state(carla.VehicleLightState(self._lights))
            world.player.apply_control(self._control)

    def _parse_vehicle_wheel(self):
        numAxes = self._joystick.get_numaxes()
        jsInputs = [float(self._joystick.get_axis(i)) for i in range(numAxes)]
        self._joystick.rumble(100,100,5)
        jsButtons = [float(self._joystick.get_button(i)) for i in
                     range(self._joystick.get_numbuttons())]

        # Steering command
        # Custom function to map range of inputs [1, -1] to outputs [0, 1] i.e 1 from inputs means nothing is pressed
        # For the steering, it seems fine as it is
        K1 = 1.0  # 0.55
        # limiting wheel rotation to +/- 0.65
        steerPos = jsInputs[self._steer_idx]
        if (steerPos > .65): steerPos = .65
        if (steerPos < -.65): steerPos = -.65
        steerCmd = K1 * math.tan(1.1 * steerPos)

        # speed related commands
        K2 = 1.6
        if (jsInputs[self._throttle_idx] == 0.0):
            throttleCmd = 0
        else:
            throttleCmd = K2 + (2.05 * math.log10(-0.7 * jsInputs[self._throttle_idx] + 1.4) - 1.2) / 0.92
            # throttleCmd = K2 + (2.05 * math.log10(0.7 * jsInputs[self._throttle_idx] + 1.4) - 1.2) / 0.92
        if throttleCmd <= 0:
            throttleCmd = 0
        elif throttleCmd > 1:
            throttleCmd = 1
        if (jsInputs[self._brake_idx] == 0.0):
            brakeCmd = 0
        else:
            brakeCmd = 1.6 + (2.05 * math.log10(-0.7 * jsInputs[self._brake_idx] + 1.4) - 1.2) / 0.92
        if brakeCmd <= 0:
            brakeCmd = 0
        elif brakeCmd > 1:
            brakeCmd = 1

        self._control.steer = steerCmd
        self._control.brake = brakeCmd
        self._control.throttle = throttleCmd


    def _parse_speedToWheel(self, world):
        # adjusts steering wheel autocenter using speed

        v = world.player.get_velocity()
        speed = (3.6 * math.sqrt(v.x ** 2 + v.y ** 2 + v.z ** 2))

        # speed limit that influences the autocenter
        S2W_THRESHOLD = 90
        if (speed > S2W_THRESHOLD):
            speed = S2W_THRESHOLD
        # autocenterCmd  \in [0,65535]
        autocenterCmd = 60000 * math.sin(speed / S2W_THRESHOLD)

        # send autocenterCmd to the steeringwheel
        # self._evtdev.write(ecodes.EV_FF, ecodes.FF_AUTOCENTER, int(autocenterCmd))

    def _parse_speedToSound(self, world, clock):
        # play sound
        v = world.player.get_velocity()
        self.sound_played_time += clock.get_time()  # milliseconds since last tick; len(array)/samplerate = 1/2 audio time
        speed = (3.6 * math.sqrt(v.x ** 2 + v.y ** 2 + v.z ** 2))
        self.car_sound = pygame.mixer.Sound(self.car_sound_array_idle)
        self.car_sound.set_volume(0.6)
        # from pysndfx import AudioEffectsChain
        if self.sound_played_time > 2e2:
            # every 0.5 seconds, reset
            if speed < 180 and speed > 5:
                _max_speed_sound = 30
                _factor = float(0.15 * speed)
                # fx = (AudioEffectsChain().speed(_factor))
                speeded_array = self.speedx(self.car_sound_array_idle, _factor)
                # print('original len:{}, speeded len:{}'.format(len(self.car_sound_array_idle), len(speeded_array)))
                # sf.write('_temp.wav', speeded_array, self.sample_rate, 'PCM_16')
                # self.car_sound = pygame.mixer.Sound('_temp.wav')
                # continue play
                self.car_sound = pygame.mixer.Sound(speeded_array[min(len(speeded_array), int(self.sound_played_time / 2 / 1e3 * 44100)):])
            # elif speed < 80 and speed > 5:
            #     _max_speed_sound = 80
            #     _factor = float(0.1 * speed)
            #     speeded_array = self.speedx(self.car_sound_array_low, _factor)
            #     self.car_sound = pygame.mixer.Sound(speeded_array[min(len(speeded_array), int(self.sound_played_time / 2 / 1e3 * 44100)):])
            # elif speed >= 80 and speed > 5:
            #     _max_speed_sound = 180
            #     _factor = float(0.05 * speed)
            #     speeded_array = self.speedx(self.car_sound_array_high, _factor)
            #     self.car_sound = pygame.mixer.Sound(speeded_array[min(len(speeded_array), int(self.sound_played_time / 2 / 1e3 * 44100)):])
            self.car_sound.play(maxtime=300)
            self.sound_played_time = 0
        if self._control.throttle == 0:
            self.car_sound.stop()
            self.car_sound_idle.set_volume(0.4)
            self.car_sound_idle.play(maxtime=500)



    def _parse_vehicle_keys(self, keys, milliseconds):
        if keys[K_UP] or keys[K_w]:
            self._control.throttle = min(self._control.throttle + 0.1, 1.00)
        else:
            self._control.throttle = 0.0

        if keys[K_DOWN] or keys[K_s]:
            self._control.brake = min(self._control.brake + 0.2, 1)
        else:
            self._control.brake = 0

        steer_increment = 5e-4 * milliseconds
        if keys[K_LEFT] or keys[K_a]:
            if self._steer_cache > 0:
                self._steer_cache = 0
            else:
                self._steer_cache -= steer_increment
        elif keys[K_RIGHT] or keys[K_d]:
            if self._steer_cache < 0:
                self._steer_cache = 0
            else:
                self._steer_cache += steer_increment
        else:
            self._steer_cache = 0.0
        self._steer_cache = min(0.7, max(-0.7, self._steer_cache))
        self._control.steer = round(self._steer_cache, 1)
        self._control.hand_brake = keys[K_SPACE]

    @staticmethod
    def _is_quit_shortcut(key):
        return (key == K_ESCAPE) or (key == K_q and pygame.key.get_mods() & KMOD_CTRL)
    @staticmethod
    def speedx(sound_array, factor):
        """ Multiplies the sound's speed by some `factor` """
        indices = np.round(np.arange(0, len(sound_array), factor))
        indices = indices[indices < len(sound_array)].astype(int)
        return sound_array[indices.astype(int)]
    # @staticmethod
    # def stretch(sound_array, f, window_size, h):
    #     """ Stretches the sound by a factor `f` """
    #     phase = np.zeros(window_size)
    #     hanning_window = np.hanning(window_size)
    #     result = np.zeros(int(len(sound_array) / f + window_size))
    #     for i in np.round(np.arange(0, len(sound_array) - (window_size + h), h * f)).astype(int):
    #         # two potentially overlapping subarrays
    #         a1 = sound_array[i: i + window_size]
    #         a2 = sound_array[i + h: i + window_size + h]
    #         # resynchronize the second array on the first
    #         s1 = np.fft.fft(hanning_window * a1)
    #         s2 = np.fft.fft(hanning_window * a2)
    #         phase = (phase + np.angle(s2 / s1)) % 2 * np.pi
    #         a2_rephased = np.fft.ifft(np.abs(s2) * np.exp(1j * phase))
    #         # add to result
    #         i2 = int(i / f)
    #         result[i2: i2 + window_size] += (hanning_window * a2_rephased).astype('float64')
    #     result = ((2 ** (16 - 4)) * result / result.max())  # normalize (16bit)
    #     return result.astype('int16')


# ==============================================================================
# -- HUD -----------------------------------------------------------------------
# ==============================================================================


class HUD(object):
    def __init__(self, width, height):
        self.dim = (width, height)
        font = pygame.font.Font(pygame.font.get_default_font(), 20)
        font_name = 'courier' if os.name == 'nt' else 'mono'
        fonts = [x for x in pygame.font.get_fonts() if font_name in x]
        default_font = 'ubuntumono'
        mono = default_font if default_font in fonts else fonts[0]
        mono = pygame.font.match_font(mono)
        self._font_mono = pygame.font.Font(mono, 12 if os.name == 'nt' else 14)
        self._notifications = FadingText(font, (width, 40), (0, height - 40))
        self.help = HelpText(pygame.font.Font(mono, 16), width, height)
        self.server_fps = 0
        self.frame = 0
        self.simulation_time = 0
        self._show_info = True
        self._info_text = []
        self._gauge_display = {}
        self._server_clock = pygame.time.Clock()
        self._passed_waypoints = []

        self.score_from_pengci = 0
        self.deduct_score = 0

    def on_world_tick(self, timestamp):
        self._server_clock.tick()
        self.server_fps = self._server_clock.get_fps()
        self.frame = timestamp.frame
        self.simulation_time = timestamp.elapsed_seconds

    def tick(self, world, clock):
        self._notifications.tick(world, clock)
        if not self._show_info:
            return
        t = world.player.get_transform()
        wp = world.map.get_waypoint(t.location)
        self._passed_waypoints.append(wp)
        v = world.player.get_velocity()
        c = world.player.get_control()
        compass = world.imu_sensor.compass
        heading = 'N' if compass > 270.5 or compass < 89.5 else ''
        heading += 'S' if 90.5 < compass < 269.5 else ''
        heading += 'E' if 0.5 < compass < 179.5 else ''
        heading += 'W' if 180.5 < compass < 359.5 else ''
        colhist = world.collision_sensor.get_collision_history()
        collision = [colhist[x + self.frame - 200] for x in range(0, 200)]
        max_col = max(1.0, max(collision))
        collision = [x / max_col for x in collision]
        vehicles = world.world.get_actors().filter('vehicle.*')
        self._gauge_display['Speed'] = 3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2)
        self._info_text = [
            'Server:  % 16.0f FPS' % self.server_fps,
            'Client:  % 16.0f FPS' % clock.get_fps(),
            '',
            'Vehicle: % 20s' % get_actor_display_name(world.player, truncate=20),
            'Map:     % 20s' % world.map.name.split('/')[-1],
            'Simulation time: % 12s' % datetime.timedelta(seconds=int(self.simulation_time)),
            '',
            'Speed:   % 15.0f km/h' % (3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2)),
            u'Compass:% 17.0f\N{DEGREE SIGN} % 2s' % (compass, heading),
            'Accelero: (%5.1f,%5.1f,%5.1f)' % (world.imu_sensor.accelerometer),
            'Gyroscop: (%5.1f,%5.1f,%5.1f)' % (world.imu_sensor.gyroscope),
            'Location:% 20s' % ('(% 5.1f, % 5.1f)' % (t.location.x, t.location.y)),
            'GNSS:% 24s' % ('(% 2.6f, % 3.6f)' % (world.gnss_sensor.lat, world.gnss_sensor.lon)),
            'Height:  % 18.0f m' % t.location.z,
            '']
        self._info_text += [
            ('Throttle:', c.throttle, 0.0, 1.0),
            ('Steer:', c.steer, -1.0, 1.0),
            ('Brake:', c.brake, 0.0, 1.0),
            ('Reverse:', c.reverse),
            ('Hand brake:', c.hand_brake),
            ('Manual:', c.manual_gear_shift),
            'Gear:        %s' % {-1: 'R', 0: 'N'}.get(c.gear, c.gear)]
        self._info_text += [
            '',
            'Collision:',
            collision,
            '',
            'Number of vehicles: % 8d' % len(vehicles)]
        if len(vehicles) > 1:
            self._info_text += ['Nearby vehicles:']
            distance = lambda l: math.sqrt((l.x - t.location.x)**2 + (l.y - t.location.y)**2 + (l.z - t.location.z)**2)
            vehicles = [(distance(x.get_location()), x) for x in vehicles if x.id != world.player.id]
            for d, vehicle in sorted(vehicles, key=lambda vehicles: vehicles[0]):
                if d > 200.0:
                    break
                vehicle_type = get_actor_display_name(vehicle, truncate=22)
                self._info_text.append('% 4dm %s' % (d, vehicle_type))

    def toggle_info(self):
        self._show_info = not self._show_info

    def notification(self, text, seconds=2.0):
        self._notifications.set_text(text, seconds=seconds)

    def error(self, text):
        self._notifications.set_text('Error: %s' % text, (255, 0, 0))

    def render(self, display):
        if self._show_info:
            info_surface = pygame.Surface((220, self.dim[1]))
            info_surface.set_alpha(100)
            display.blit(info_surface, (0, 0))
            v_offset = 4
            bar_h_offset = 100
            bar_width = 106
            for item in self._info_text:
                if v_offset + 18 > self.dim[1]:
                    break
                if isinstance(item, list):
                    if len(item) > 1:
                        points = [(x + 8, v_offset + 8 + (1.0 - y) * 30) for x, y in enumerate(item)]
                        pygame.draw.lines(display, (255, 136, 0), False, points, 2)
                    item = None
                    v_offset += 18
                elif isinstance(item, tuple):
                    if isinstance(item[1], bool):
                        rect = pygame.Rect((bar_h_offset, v_offset + 8), (6, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect, 0 if item[1] else 1)
                    else:
                        rect_border = pygame.Rect((bar_h_offset, v_offset + 8), (bar_width, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect_border, 1)
                        f = (item[1] - item[2]) / (item[3] - item[2])
                        if item[2] < 0.0:
                            rect = pygame.Rect((bar_h_offset + f * (bar_width - 6), v_offset + 8), (6, 6))
                        else:
                            rect = pygame.Rect((bar_h_offset, v_offset + 8), (f * bar_width, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect)
                    item = item[0]
                if item:  # At this point has to be a str.
                    surface = self._font_mono.render(item, True, (255, 255, 255))
                    display.blit(surface, (8, v_offset))
                v_offset += 18
        self._notifications.render(display)
        self.help.render(display)

    def render_gauge(self, display, checkpoint_num):
        gauge_surface = pygame.Surface((300, 300))
        gauge_surface.set_alpha(100)
        gauge_center = (int(gauge_surface.get_width()/2), int(gauge_surface.get_height()/2))
        speed_angle = int(self._gauge_display['Speed'] * 270 / 160)
        speed_color = [int(speed_angle), int(255-speed_angle), 0]
        for i in range(len(speed_color)):
            if speed_color[i] > 255:
                speed_color[i] = 255
            if speed_color[i] < 0:
                speed_color[i] = 0
        arc_thickness = 50
        _font = pygame.font.SysFont('Franklin Gothic Heavy', 60)
        speed_text = _font.render('%15.0f km/h' % self._gauge_display['Speed'], True, speed_color)
        text_rect = speed_text.get_rect(center=gauge_center)
        display.blit(speed_text, text_rect)  # show speed number
        for i in range(0, arc_thickness):
            # plot outline
            pygame.gfxdraw.arc(gauge_surface, gauge_center[0], gauge_center[1], 100 - i, -255, 270-255, (55, 77, 91))
            # plot speed
            pygame.gfxdraw.arc(gauge_surface, gauge_center[0], gauge_center[1], 100 - i, -255, speed_angle-255, speed_color)
        display.blit(gauge_surface, (0, 0))  # show speed index

        # for music credit
        # license_surface = pygame.Surface((800, 50))
        # license_center = (int(license_surface.get_width()/2), int(license_surface.get_height()/2))
        _font = pygame.font.SysFont('Ubuntu', 18)
        license_text = _font.render('music by audionautix.com', True, [0, 0, 0])
        text_rect = speed_text.get_rect(center=(1800, 960))
        display.blit(license_text, text_rect)

        # for scoring (checkpoint) information
        _font = pygame.font.SysFont('Ubuntu', 40)
        license_text = _font.render('Your Pengci Score: {}'.format(self.score_from_pengci), True, [255, 0, 0])
        text_rect = speed_text.get_rect(center=(900, 200))
        display.blit(license_text, text_rect)

        # for collision information
        self._notifications.render(display)
        _font = pygame.font.SysFont('Ubuntu', 40)
        license_text = _font.render('Points deducted: {}'.format(self.deduct_score), True, [255, 0, 0])
        text_rect = speed_text.get_rect(center=(900, 260))
        display.blit(license_text, text_rect)



    # ==============================================================================
# -- FadingText ----------------------------------------------------------------
# ==============================================================================


class FadingText(object):
    def __init__(self, font, dim, pos):
        self.font = font
        self.dim = dim
        self.pos = pos
        self.seconds_left = 0
        self.surface = pygame.Surface(self.dim)

    def set_text(self, text, color=(255, 255, 255), seconds=2.0):
        text_texture = self.font.render(text, True, color)
        self.surface = pygame.Surface(self.dim)
        self.seconds_left = seconds
        self.surface.fill((0, 0, 0, 0))
        self.surface.blit(text_texture, (10, 11))

    def tick(self, _, clock):
        delta_seconds = 1e-3 * clock.get_time()
        self.seconds_left = max(0.0, self.seconds_left - delta_seconds)
        self.surface.set_alpha(500.0 * self.seconds_left)

    def render(self, display):
        display.blit(self.surface, self.pos)


# ==============================================================================
# -- HelpText ------------------------------------------------------------------
# ==============================================================================


class HelpText(object):
    """Helper class to handle text output using pygame"""
    def __init__(self, font, width, height):
        lines = __doc__.split('\n')
        self.font = font
        self.line_space = 18
        self.dim = (780, len(lines) * self.line_space + 12)
        self.pos = (0.5 * width - 0.5 * self.dim[0], 0.5 * height - 0.5 * self.dim[1])
        self.seconds_left = 0
        self.surface = pygame.Surface(self.dim)
        self.surface.fill((0, 0, 0, 0))
        for n, line in enumerate(lines):
            text_texture = self.font.render(line, True, (255, 255, 255))
            self.surface.blit(text_texture, (22, n * self.line_space))
            self._render = False
        self.surface.set_alpha(220)

    def toggle(self):
        self._render = not self._render

    def render(self, display):
        if self._render:
            display.blit(self.surface, self.pos)


# ==============================================================================
# -- CollisionSensor -----------------------------------------------------------
# ==============================================================================


class CollisionSensor(object):
    def __init__(self, parent_actor, hud):
        self.sensor = None
        self.history = []
        self._parent = parent_actor
        self.hud = hud
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.collision')
        self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: CollisionSensor._on_collision(weak_self, event))

    def get_collision_history(self):
        history = collections.defaultdict(int)
        for frame, intensity in self.history:
            history[frame] += intensity
        return history

    @staticmethod
    def _on_collision(weak_self, event):
        self = weak_self()
        if not self:
            return
        actor_type = get_actor_display_name(event.other_actor)
        self.hud.notification('Collision with %r, deduct 20 points!' % actor_type)
        self.hud.deduct_score += 20
        impulse = event.normal_impulse
        intensity = math.sqrt(impulse.x**2 + impulse.y**2 + impulse.z**2)
        self.history.append((event.frame, intensity))
        if len(self.history) > 4000:
            self.history.pop(0)


# ==============================================================================
# -- LaneInvasionSensor --------------------------------------------------------
# ==============================================================================


class LaneInvasionSensor(object):
    def __init__(self, parent_actor, hud):
        self.sensor = None

        # If the spawn object is not a vehicle, we cannot use the Lane Invasion Sensor
        if parent_actor.type_id.startswith("vehicle."):
            self._parent = parent_actor
            self.hud = hud
            world = self._parent.get_world()
            bp = world.get_blueprint_library().find('sensor.other.lane_invasion')
            self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=self._parent)
            # We need to pass the lambda a weak reference to self to avoid circular
            # reference.
            weak_self = weakref.ref(self)
            self.sensor.listen(lambda event: LaneInvasionSensor._on_invasion(weak_self, event))

    @staticmethod
    def _on_invasion(weak_self, event):
        self = weak_self()
        if not self:
            return
        lane_types = set(x.type for x in event.crossed_lane_markings)
        text = ['%r' % str(x).split()[-1] for x in lane_types]
        self.hud.notification('Crossed line %s' % ' and '.join(text))


# ==============================================================================
# -- GnssSensor ----------------------------------------------------------------
# ==============================================================================


class GnssSensor(object):
    def __init__(self, parent_actor):
        self.sensor = None
        self._parent = parent_actor
        self.lat = 0.0
        self.lon = 0.0
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.gnss')
        self.sensor = world.spawn_actor(bp, carla.Transform(carla.Location(x=1.0, z=2.8)), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: GnssSensor._on_gnss_event(weak_self, event))

    @staticmethod
    def _on_gnss_event(weak_self, event):
        self = weak_self()
        if not self:
            return
        self.lat = event.latitude
        self.lon = event.longitude


# ==============================================================================
# -- IMUSensor -----------------------------------------------------------------
# ==============================================================================


class IMUSensor(object):
    def __init__(self, parent_actor):
        self.sensor = None
        self._parent = parent_actor
        self.accelerometer = (0.0, 0.0, 0.0)
        self.gyroscope = (0.0, 0.0, 0.0)
        self.compass = 0.0
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.imu')
        self.sensor = world.spawn_actor(
            bp, carla.Transform(), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(
            lambda sensor_data: IMUSensor._IMU_callback(weak_self, sensor_data))

    @staticmethod
    def _IMU_callback(weak_self, sensor_data):
        self = weak_self()
        if not self:
            return
        limits = (-99.9, 99.9)
        self.accelerometer = (
            max(limits[0], min(limits[1], sensor_data.accelerometer.x)),
            max(limits[0], min(limits[1], sensor_data.accelerometer.y)),
            max(limits[0], min(limits[1], sensor_data.accelerometer.z)))
        self.gyroscope = (
            max(limits[0], min(limits[1], math.degrees(sensor_data.gyroscope.x))),
            max(limits[0], min(limits[1], math.degrees(sensor_data.gyroscope.y))),
            max(limits[0], min(limits[1], math.degrees(sensor_data.gyroscope.z))))
        self.compass = math.degrees(sensor_data.compass)


# ==============================================================================
# -- RadarSensor ---------------------------------------------------------------
# ==============================================================================


class RadarSensor(object):
    def __init__(self, parent_actor):
        self.sensor = None
        self._parent = parent_actor
        bound_x = 0.5 + self._parent.bounding_box.extent.x
        bound_y = 0.5 + self._parent.bounding_box.extent.y
        bound_z = 0.5 + self._parent.bounding_box.extent.z

        self.velocity_range = 7.5 # m/s
        world = self._parent.get_world()
        self.debug = world.debug
        bp = world.get_blueprint_library().find('sensor.other.radar')
        bp.set_attribute('horizontal_fov', str(35))
        bp.set_attribute('vertical_fov', str(20))
        self.sensor = world.spawn_actor(
            bp,
            carla.Transform(
                carla.Location(x=bound_x + 0.05, z=bound_z+0.05),
                carla.Rotation(pitch=5)),
            attach_to=self._parent)
        # We need a weak reference to self to avoid circular reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(
            lambda radar_data: RadarSensor._Radar_callback(weak_self, radar_data))

    @staticmethod
    def _Radar_callback(weak_self, radar_data):
        self = weak_self()
        if not self:
            return
        # To get a numpy [[vel, altitude, azimuth, depth],...[,,,]]:
        # points = np.frombuffer(radar_data.raw_data, dtype=np.dtype('f4'))
        # points = np.reshape(points, (len(radar_data), 4))

        current_rot = radar_data.transform.rotation
        for detect in radar_data:
            azi = math.degrees(detect.azimuth)
            alt = math.degrees(detect.altitude)
            # The 0.25 adjusts a bit the distance so the dots can
            # be properly seen
            fw_vec = carla.Vector3D(x=detect.depth - 0.25)
            carla.Transform(
                carla.Location(),
                carla.Rotation(
                    pitch=current_rot.pitch + alt,
                    yaw=current_rot.yaw + azi,
                    roll=current_rot.roll)).transform(fw_vec)

            def clamp(min_v, max_v, value):
                return max(min_v, min(value, max_v))

            norm_velocity = detect.velocity / self.velocity_range # range [-1, 1]
            r = int(clamp(0.0, 1.0, 1.0 - norm_velocity) * 255.0)
            g = int(clamp(0.0, 1.0, 1.0 - abs(norm_velocity)) * 255.0)
            b = int(abs(clamp(- 1.0, 0.0, - 1.0 - norm_velocity)) * 255.0)
            self.debug.draw_point(
                radar_data.transform.location + fw_vec,
                size=0.075,
                life_time=0.06,
                persistent_lines=False,
                color=carla.Color(r, g, b))

# ==============================================================================
# -- CameraManager -------------------------------------------------------------
# ==============================================================================


class CameraManager(object):
    def __init__(self, parent_actor, hud):
        self.sensor = None
        self.surface = None
        self._parent = parent_actor
        self.hud = hud
        self.recording = False
        bound_x = 0.5 + self._parent.bounding_box.extent.x
        bound_y = 0.5 + self._parent.bounding_box.extent.y
        bound_z = 0.5 + self._parent.bounding_box.extent.z
        Attachment = carla.AttachmentType

        self._camera_transforms = [
            (carla.Transform(carla.Location(x=-2.0*bound_x, y=+0.0*bound_y, z=2.0*bound_z), carla.Rotation(pitch=8.0)), Attachment.SpringArm),
            (carla.Transform(carla.Location(x=+0.8*bound_x, y=+0.0*bound_y, z=1.3*bound_z)), Attachment.Rigid),
            # (carla.Transform(carla.Location(x=+1.9*bound_x, y=+1.0*bound_y, z=1.2*bound_z)), Attachment.SpringArm),
            (carla.Transform(carla.Location(x=-2.8*bound_x, y=+0.0*bound_y, z=4.6*bound_z), carla.Rotation(pitch=6.0)), Attachment.SpringArm),
            # (carla.Transform(carla.Location(x=-1.0, y=-1.0*bound_y, z=0.4*bound_z)), Attachment.Rigid),
            (carla.Transform(carla.Location(x=-0.0, y=-0.2, z=1.2), carla.Rotation()), Attachment.Rigid),
            (carla.Transform(carla.Location(x=-0.15, y=-0.4, z=1.2), carla.Rotation()), Attachment.Rigid)]

        self.transform_index = 1
        self.sensors = [['sensor.camera.rgb', cc.Raw, 'Camera RGB']]
        world = self._parent.get_world()
        bp_library = world.get_blueprint_library()
        for item in self.sensors:
            bp = bp_library.find(item[0])
            bp.set_attribute('image_size_x', str(hud.dim[0]))
            bp.set_attribute('image_size_y', str(hud.dim[1]))
            bp.set_attribute('gamma', '2.2')
            item.append(bp)
        self.index = None

    def toggle_camera(self):
        self.transform_index = (self.transform_index + 1) % len(self._camera_transforms)
        self.set_sensor(self.index, notify=False, force_respawn=True)

    def set_sensor(self, index, notify=True, force_respawn=False):
        index = index % len(self.sensors)
        needs_respawn = True if self.index is None else \
            (force_respawn or (self.sensors[index][2] != self.sensors[self.index][2]))
        if needs_respawn:
            if self.sensor is not None:
                self.sensor.destroy()
                self.surface = None
            self.sensor = self._parent.get_world().spawn_actor(
                self.sensors[index][-1],
                self._camera_transforms[self.transform_index][0],
                attach_to=self._parent,
                attachment_type=self._camera_transforms[self.transform_index][1])
            # We need to pass the lambda a weak reference to self to avoid
            # circular reference.
            weak_self = weakref.ref(self)
            self.sensor.listen(lambda image: CameraManager._parse_image(weak_self, image))
        if notify:
            self.hud.notification(self.sensors[index][2])
        self.index = index

    def toggle_recording(self):
        self.recording = not self.recording
        self.hud.notification('Recording %s' % ('On' if self.recording else 'Off'))

    def render(self, display):
        if self.surface is not None:
            display.blit(self.surface, (0, 0))

    @staticmethod
    def _parse_image(weak_self, image):
        self = weak_self()
        if not self:
            return
        if self.sensors[self.index][0].startswith('sensor.lidar'):
            points = np.frombuffer(image.raw_data, dtype=np.dtype('f4'))
            points = np.reshape(points, (int(points.shape[0] / 4), 4))
            lidar_data = np.array(points[:, :2])
            lidar_data *= min(self.hud.dim) / (2.0 * self.lidar_range)
            lidar_data += (0.5 * self.hud.dim[0], 0.5 * self.hud.dim[1])
            lidar_data = np.fabs(lidar_data)  # pylint: disable=E1111
            lidar_data = lidar_data.astype(np.int32)
            lidar_data = np.reshape(lidar_data, (-1, 2))
            lidar_img_size = (self.hud.dim[0], self.hud.dim[1], 3)
            lidar_img = np.zeros((lidar_img_size), dtype=np.uint8)
            lidar_img[tuple(lidar_data.T)] = (255, 255, 255)
            self.surface = pygame.surfarray.make_surface(lidar_img)
        elif self.sensors[self.index][0].startswith('sensor.camera.dvs'):
            # Example of converting the raw_data from a carla.DVSEventArray
            # sensor into a NumPy array and using it as an image
            dvs_events = np.frombuffer(image.raw_data, dtype=np.dtype([
                ('x', np.uint16), ('y', np.uint16), ('t', np.int64), ('pol', np.bool)]))
            dvs_img = np.zeros((image.height, image.width, 3), dtype=np.uint8)
            # Blue is positive, red is negative
            dvs_img[dvs_events[:]['y'], dvs_events[:]['x'], dvs_events[:]['pol'] * 2] = 255
            self.surface = pygame.surfarray.make_surface(dvs_img.swapaxes(0, 1))
        elif self.sensors[self.index][0].startswith('sensor.camera.optical_flow'):
            image = image.get_color_coded_flow()
            array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (image.height, image.width, 4))
            array = array[:, :, :3]
            array = array[:, :, ::-1]
            self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
        else:
            image.convert(self.sensors[self.index][1])
            array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (image.height, image.width, 4))
            array = array[:, :, :3]
            array = array[:, :, ::-1]
            self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
        if self.recording:
            image.save_to_disk('_out/%08d' % image.frame)


# ==============================================================================
# -- game_loop() ---------------------------------------------------------------
# ==============================================================================

class GameArgs(object):
    def __int__(self):
        self.host = '127.0.0.1'
        self.port = 2000
        self.width, self.height = 1920, 1080
        self.scenario = 'OfflineChallenge_1'
        self.autopilot = False
        self.rolename = 'hero'
        self.keep_ego_vehicle = False

def game_loop(args):
    pygame.init()
    pygame.font.init()
    world = None

    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(20.0)
        sim_world = client.get_world()

        display = pygame.display.set_mode(
            (args.width, args.height),
            pygame.HWSURFACE | pygame.DOUBLEBUF)# | pygame.FULLSCREEN)
        display.fill((0,0,0))
        pygame.display.flip()

        hud = HUD(args.width, args.height)
        # setup ego cars based on scenario
        scenario_configurations = ScenarioConfigurationParser.parse_scenario_configuration(
            args.scenario, '')
        start_transform = None
        for config in scenario_configurations:
            for ego_vehicle in config.ego_vehicles:
                if 'lincoln' in ego_vehicle.model:
                    start_transform = ego_vehicle.transform
            world = World(sim_world, hud, args)
            # teleport
            world.player.set_transform(start_transform)
            controller = WheelControl(world, args.autopilot, args)

            sim_world.wait_for_tick()

            clock = pygame.time.Clock()

            # music
            pygame.mixer.music.load(controller._bgm_dir)
            pygame.mixer.music.set_volume(0.9)
            pygame.mixer.music.play(-1)
            while True:
                clock.tick_busy_loop(60)
                if controller.parse_events(client, world, clock):
                    return
                if not world.tick(clock):
                    return
                world.render(display)
                pygame.display.flip()

    finally:

        if (world and world.recording_enabled):
            client.stop_recorder()

        if world is not None:
            # prevent destruction of ego vehicle
            if args.keep_ego_vehicle:
                world.player = None
            world.destroy()

        pygame.quit()


# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================


def game_main():
    argparser = argparse.ArgumentParser(
        description='CARLA Manual Control Client')
    argparser.add_argument(
        '-v', '--verbose',
        action='store_true',
        dest='debug',
        help='print debug information')
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '-a', '--autopilot',
        action='store_true',
        help='enable autopilot. This does not autocomplete the scenario')
    argparser.add_argument(
        '--rolename',
        metavar='NAME',
        default='hero',
        help='role name of ego vehicle to control (default: "hero")')
    argparser.add_argument(
        '--res',
        metavar='WIDTHxHEIGHT',
        default='1920x1080',
        # default='1270x720',
        help='window resolution (default: 1280x720)')
    argparser.add_argument(
        '--keep_ego_vehicle',
        action='store_true',
        help='do not destroy ego vehicle on exit')
    argparser.add_argument(
        '--scenario',
        default='OfflineChallenge_2',
        help='Name of the scenario to be executed. Use the preposition \'group:\' to run all scenarios of one class, e.g. ControlLoss or FollowLeadingVehicle')
    args = argparser.parse_args()

    args.width, args.height = [int(x) for x in args.res.split('x')]

    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)

    logging.info('listening to server %s:%s', args.host, args.port)

    print(__doc__)

    try:

        game_loop(args)

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')
    except Exception as error:
        logging.exception(error)


if __name__ == '__main__':

    game_main()
