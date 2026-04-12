import carla
import math


class RouteFollower:
    def __init__(self,
                 vehicle,
                 world,
                 target_speed=8.0,
                 steer_k=1.0,
                 steer_soft_term=1.0,
                 speed_kp=0.18,
                 waypoint_reach_dist=4.0,
                 ):
        self.vehicle = vehicle
        self.world = world
        self.target_speed = target_speed
        self.steer_k = steer_k
        self.steer_soft_term = steer_soft_term
        self.speed_kp = speed_kp
        self.waypoint_reach_dist = waypoint_reach_dist

        self.route = []
        self.target_index = 0
        self.finished = False

    def set_route(self, route):
        self.route = route
        self.target_index = 0
        self.finished = (len(route) == 0)

    def is_finished(self):
        return self.finished

    def _normalize_angle(self, angle):
        while angle > 180:
            angle -= 360
        while angle < -180:
            angle += 360
        return angle

    def _compute_steer(self, target_waypoint):
        vehicle_transform = self.vehicle.get_transform()
        vehicle_location = vehicle_transform.location
        vehicle_yaw_deg = vehicle_transform.rotation.yaw
        vehicle_yaw_rad = math.radians(vehicle_yaw_deg)

        target_transform = target_waypoint.transform
        target_location = target_transform.location
        target_yaw_deg = target_transform.rotation.yaw
        target_yaw_rad = math.radians(target_yaw_deg)

        yaw_error = self._normalize_angle(target_yaw_deg - vehicle_yaw_deg)
        heading_error_rad = math.radians(yaw_error)

        wheel_base = 2.5
        front_x = vehicle_location.x + wheel_base * math.cos(vehicle_yaw_rad)
        front_y = vehicle_location.y + wheel_base * math.sin(vehicle_yaw_rad)

        dx = front_x - target_location.x
        dy = front_y - target_location.y

        path_x = math.cos(target_yaw_rad)
        path_y = math.sin(target_yaw_rad)

        cross_track_error = -(dx * (-path_y) + dy * path_x)

        velocity = self.vehicle.get_velocity()
        speed = math.sqrt(
            velocity.x ** 2 +
            velocity.y ** 2 +
            velocity.z ** 2
        )

        cross_track_term = math.atan2(
            self.steer_k * cross_track_error,
            speed + self.steer_soft_term
        )

        steer_rad = heading_error_rad + cross_track_term
        steer = max(-1.0, min(1.0, steer_rad))
        return steer

    def _compute_speed_control(self):
        velocity = self.vehicle.get_velocity()
        speed = math.sqrt(
            velocity.x ** 2 +
            velocity.y ** 2 +
            velocity.z ** 2
        )

        speed_error = self.target_speed - speed

        if speed_error >= 0:
            throttle = self.speed_kp * speed_error
            throttle = max(0.0, min(0.75, throttle))
            brake = 0.0
        else:
            throttle = 0.0
            brake = min(0.5, -self.speed_kp * speed_error)

        return throttle, brake, speed

    def _update_target_waypoint(self):
        if self.finished or not self.route:
            return None

        target_waypoint = self.route[self.target_index]
        vehicle_location = self.vehicle.get_transform().location
        distance = vehicle_location.distance(target_waypoint.transform.location)

        if distance < self.waypoint_reach_dist:
            if self.target_index < len(self.route) - 1:
                self.target_index += 1
                target_waypoint = self.route[self.target_index]
            else:
                self.finished = True

        return target_waypoint

    def run_step(self):
        if self.finished or not self.route:
            return carla.VehicleControl(throttle=0.0, steer=0.0, brake=1.0)

        target_waypoint = self._update_target_waypoint()

        if target_waypoint is None:
            return carla.VehicleControl(throttle=0.0, steer=0.0, brake=1.0)

        steer = self._compute_steer(target_waypoint)
        throttle, brake, speed = self._compute_speed_control()

        self.world.debug.draw_point(
            target_waypoint.transform.location,
            size=0.15,
            color=carla.Color(255, 0, 0),
            life_time=0.1
        )

        return carla.VehicleControl(
            throttle=throttle,
            steer=steer,
            brake=brake
        )
