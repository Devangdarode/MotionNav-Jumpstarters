"""
Copyright (c) 2023 Thomas Gerling, Devang Darode, Mohamed Yassine Bouchiba

[Project Name] -> A Project of the JumpStarter Group in the Eclipse SDV Hackathon

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""

import carla
import math
import pygame
import sys
import cv2
import numpy as np
import open3d as o3d
from matplotlib import cm
import ecal.core.core as ecal_core
from ecal.core.subscriber import ProtoSubscriber
from ecal.core.subscriber import StringSubscriber

subUser = StringSubscriber("Face/name")

class Simulation:
    def __init__(self):
        # Initialize simulation parameters and objects
        self.port = 2000
        self.weather_mode = False
        self.sensor_data = []
        self.vis = o3d.visualization.Visualizer()
        self.render_object(100, 100)
        self.client = self.setup_client()
        self.world = self.setup_world(self.client, self.weather_mode)
        self.vehicle = self.spawn_npc(self.world)
        self.test_car = self.setup_test_car(self.world)

    def render_object(self, width, height):
        # Placeholder method, not clear what it does
        init_image = np.random.randint(0, 255, (height, width, 3), 'uint8')
        surface = pygame.surfarray.make_surface(init_image.swapaxes(0, 1))

    def add_open3d_axis(self, vis):
        # Add a small 3D axis on Open3D Visualizer
        axis = o3d.geometry.LineSet()
        axis.points = o3d.utility.Vector3dVector(np.array([
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0]
        ]))
        axis.lines = o3d.utility.Vector2iVector(np.array([
            [0, 1],
            [0, 2],
            [0, 3]
        ]))
        axis.colors = o3d.utility.Vector3dVector(np.array([
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0]
        ]))
        self.vis.add_geometry(axis)

    def pygame_callback(self, data, obj):
        # Callback for handling pygame events
        img = np.reshape(np.copy(data.raw_data), (data.height, data.width, 4))
        img = img[:, :, :3]
        img = img[:, :, ::-1]
        obj.surface = pygame.surfarray.make_surface(img.swapaxes(0, 1))

    def on_collision(self, event):
        # Callback for handling collision events
        print("Collision detected!")

    def depth_callback(self, image, data_dict):
        # Callback for handling depth sensor data
        image.convert(carla.ColorConverter.LogarithmicDepth)
        data_dict['depth_image'] = np.reshape(np.copy(image.raw_data), (image.height, image.width, 4))

    def camera_callback(self, image, data_dict):
        # Callback for handling camera sensor data
        img = np.reshape(np.copy(image.raw_data), (image.height, image.width, 4))
        img[:, :, 3] = 255
        data_dict['rgb_image'] = img

    def gps_callback(self, data, data_dict):
        # Callback for handling GPS sensor data
        data_dict['GPS: '] = [data.latitude, data.longitude]
        print(f"GPS: {data.latitude} | {data.longitude}")

    def lidar_callback(self, cloud, point_list):
        # Callback for handling LIDAR sensor data
        data = np.copy(np.frombuffer(cloud.raw_data, dtype=np.dtype('f4')))
        data = np.reshape(data, (int(data.shape[0] / 4), 4))

        VIRIDIS = np.array(cm.get_cmap('plasma').colors)
        VID_RANGE = np.linspace(0.0, 1.0, VIRIDIS.shape[0])

        intensity = data[:, -1]
        intensity_col = 1.0 - np.log(intensity) / np.log(np.exp(0.004 * 100))
        int_color = np.c_[
            np.interp(intensity_col, VID_RANGE, VIRIDIS[:, 0]),
            np.interp(intensity_col, VID_RANGE, VIRIDIS[:, 1]),
            np.interp(intensity_col, VID_RANGE, VIRIDIS[:, 2])
        ]
        points = data[:, :-1]

        point_list.points = o3d.utility.Vector3dVector(points)
        point_list.colors = o3d.utility.Vector3dVector(int_color)

    def lane_callback(self, lane):
        # Callback for handling lane invasion events
        print("Lane invasion detected:\n" + str(lane) + '\n')

    def obstacle_callback(self, obstacle):
        # Callback for handling obstacle detection events
        print("Obstacle detected:\n" + str(obstacle) + '\n')

    def setup_client(self):
        # Set up the CARLA client
        client = carla.Client('localhost', self.port)
        client.set_timeout(5.0)
        return client

    def setup_world(self, client, weather_mode):
        # Set up the CARLA world with specified settings
        world = client.get_world()
        settings = world.get_settings()
        settings.fixed_delta_seconds = 1 / 60
        world.apply_settings(settings)

        if weather_mode:
            # Set weather parameters if weather_mode is True
            weather = carla.WeatherParameters(
                cloudiness=80.0,
                precipitation=30.0,
                sun_altitude_angle=70.0
            )
            world.set_weather(weather)
            print(world.get_weather())
        return world

    def spawn_npc(self, world, num_vehicles=10):
        # Spawn NPC vehicles in the CARLA world
        blueprint_library = world.get_blueprint_library()
        vehicle_blueprints = blueprint_library.filter('vehicle.*')

        spawn_points = world.get_map().get_spawn_points()

        if num_vehicles > len(spawn_points):
            print(f"Error: Requested {num_vehicles} vehicles, but there are only {len(spawn_points)} spawn points available.")
            return

        for i in range(num_vehicles):
            vehicle_bp = vehicle_blueprints.filter('vehicle.*')[i % len(vehicle_blueprints)]
            spawn_point = spawn_points[i]

            vehicle = world.try_spawn_actor(vehicle_bp, spawn_point)
            if vehicle is not None:
                vehicle.set_autopilot(True, self.port)
        return vehicle

    def setup_sensors(self, world, test_car):
        # Set up various sensors for the ego vehicle
        bp_lib = world.get_blueprint_library()

        collision_bp = bp_lib.find('sensor.other.collision')
        depth_camera_bp = bp_lib.find('sensor.camera.depth')
        camera_rgb_bp = bp_lib.find('sensor.camera.rgb')
        gps_bp = bp_lib.find('sensor.other.gnss')
        lidar_bp = bp_lib.find('sensor.lidar.ray_cast')
        obstacle_bp = bp_lib.find('sensor.other.obstacle')
        invasion_bp = bp_lib.find('sensor.other.lane_invasion')

        lidar_bp.set_attribute('range', '45.0')
        lidar_bp.set_attribute('noise_stddev', '0.0')
        lidar_bp.set_attribute('upper_fov', '10.0')
        lidar_bp.set_attribute('lower_fov', '-30.0')
        lidar_bp.set_attribute('channels', '32.0')
        lidar_bp.set_attribute('rotation_frequency', '60')
        lidar_bp.set_attribute('points_per_second', '560000')
        lidar_bp.set_attribute('dropoff_general_rate', '0.0')

        collision_init = carla.Transform(carla.Location())
        camera_rgb_init = carla.Transform(carla.Location(x=0, y=0, z=1.7))
        gps_init = carla.Transform(carla.Location())
        lidar_init = carla.Transform(carla.Location(x=0, z=1.7), carla.Rotation(roll=180, yaw=90))
        obstacle_init = carla.Transform(carla.Location(x=0, z=1.7))
        invasion_init = carla.Transform(carla.Location(x=0, z=1.7))

        self.collision = world.spawn_actor(collision_bp, collision_init, attach_to=test_car)
        self.camera = world.spawn_actor(camera_rgb_bp, camera_rgb_init, attach_to=test_car)
        self.gps = world.spawn_actor(gps_bp, gps_init, attach_to=test_car)
        self.lidar = world.spawn_actor(lidar_bp, lidar_init, attach_to=test_car)
        self.obstacle = world.spawn_actor(obstacle_bp, obstacle_init, attach_to=test_car)
        self.invasion = world.spawn_actor(invasion_bp, invasion_init, attach_to=test_car)
        self.depth_camera = world.spawn_actor(depth_camera_bp, camera_rgb_init, attach_to=test_car)
        self.point_list = o3d.geometry.PointCloud()

        # Set up Open3D visualization window
        self.vis.create_window(
            window_name='Lidar Output',
            width=840,
            height=420,
            left=420,
            top=210
        )
        self.vis.get_render_option().background_color = [0.05, 0.05, 0.05]
        self.vis.get_render_option().point_size = 1
        self.vis.get_render_option().show_coordinate_frame = True
        self.add_open3d_axis(self.vis)

        image_w = camera_rgb_bp.get_attribute("image_size_x").as_int()
        image_h = camera_rgb_bp.get_attribute("image_size_y").as_int()

        self.sensor_data = {'rgb_image': np.zeros((image_h, image_w, 4)),
                            'depth_image': np.zeros((image_h, image_w, 4)),
                            'gnss': [0, 0]
                            }

    def setup_test_car(self, world, spawn_point=None):
        # Set up the ego vehicle and control for manual testing
        blueprint_library = world.get_blueprint_library()
        vehicle_bp = blueprint_library.find('vehicle.tesla.model3')
        spawn_points = world.get_map().get_spawn_points()
        test_car = world.spawn_actor(vehicle_bp, spawn_points[51])

        spectator = world.get_spectator()

        self.setup_sensors(world, test_car)

        pygame.init()

        size = (500, 250)
        pygame.display.set_caption("CARLA Manual Control")
        screen = pygame.display.set_mode(size)

        control = carla.VehicleControl()
        clock = pygame.time.Clock()
        done = False

        self.collision.listen(lambda event: self.on_collision(event))
        self.camera.listen(lambda image: self.camera_callback(image, self.sensor_data))
        self.gps.listen(lambda event: self.gps_callback(event, self.sensor_data))
        self.lidar.listen(lambda data: self.lidar_callback(data, self.point_list))
        self.invasion.listen(lambda lane: self.lane_callback(lane))
        self.obstacle.listen(lambda obstacle: self.obstacle_callback(obstacle))
        self.depth_camera.listen(lambda image: self.depth_callback(image, self.sensor_data))
        
        #Controlling the Car on two different Ways
        while not done:
            keys = pygame.key.get_pressed()
            
            manual_control = subUser.set_callback(callbackUser)
            print ("The User callback is : {}".format(manual_control))
            if not manual_control:
                control.throttle = manual_control[0]
                control.brake = manual_control[1]
                control.steer = manual_control[2]

            else:
                if keys[pygame.K_q]:
                    control.reverse = True
                else:
                    control.reverse = False

                if keys[pygame.K_UP] or keys[pygame.K_w]:
                    control.throttle = min(control.throttle + 0.05, 1.0)
                else:
                    control.throttle = 0.0

                if keys[pygame.K_DOWN] or keys[pygame.K_s]:
                    control.brake = max(control.brake - 0.02, 1.0)
                else:
                    control.brake = 0.0

                if keys[pygame.K_LEFT] or keys[pygame.K_a]:
                    control.steer = max(control.steer - 0.05, -1.0)
                elif keys[pygame.K_RIGHT] or keys[pygame.K_d]:
                    control.steer = min(control.steer + 0.05, 1.0)
                else:
                    control.steer = 0.0

                control.hand_brake = keys[pygame.K_SPACE]

            # Apply the control to the ego vehicle and tick the simulation
            test_car.apply_control(control)
            world.tick()

            # Update the display and check for the quit event
            pygame.display.flip()
            pygame.display.update()
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    done = True
            transform = test_car.get_transform()
            spectator.set_transform(carla.Transform(transform.location + carla.Location(z=50), carla.Rotation(pitch=-90)))
            cv2.imshow("Camera Output", self.sensor_data['rgb_image'])

            self.vis.update_geometry(self.point_list)
            self.vis.add_geometry(self.point_list)
            self.vis.poll_events()
            self.vis.update_renderer()
            clock.tick(60)
        return test_car

def callbackUser(topic_name, msg, time):
    user = "no user"
    user == str(msg)
    return user

# To set up and run the simulation
if __name__ == '__main__':
     # Initialize eCAL and give a process name
    ecal_core.initialize(sys.argv, "HandGesture to destinated detector")

    # Creating a subscriber that listens to "name/Face" topic
    subUser = StringSubscriber("Face/name")
  
    # Just don't exit
    #while ecal_core.ok():
    #    time.sleep(0.5)
    
    # finalize eCAL API
    #ecal_core.finalize()
    Simulation()
