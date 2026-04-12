import carla
import random
import time
from planner import CarlaGraphPlanner
from controller import RouteFollower
import controller
client = carla.Client("localhost", 2000)
client.set_timeout(5.0)

world = client.get_world()
carla_map = world.get_map()
blueprints = world.get_blueprint_library()
spectator = world.get_spectator()

def update_spectator(vehicle):
    transform = vehicle.get_transform()
    spectator.set_transform(
        carla.Transform(
            transform.location + carla.Location(z=40),
            carla.Rotation(pitch=-90)
        )
    )

def spawn_vehicle(world, start_point):
    blueprints = world.get_blueprint_library()
    vehicle_bp = blueprints.find("vehicle.tesla.model3")

    vehicle = world.try_spawn_actor(vehicle_bp, start_point)
    if vehicle is None:
        print("Spawn failed.")
        return None

    return vehicle



def draw_route(world, route, life_time=30.0):
    if not route:
        print("No route found.")
        return

    for i, wp in enumerate(route):
        loc = wp.transform.location + carla.Location(z=0.5)

        # 起点红色，终点绿色，中间点黄色
        if i == 0:
            color = carla.Color(255, 0, 0)
            size = 0.16
        elif i == len(route) - 1:
            color = carla.Color(0, 255, 0)
            size = 0.16
        else:
            color = carla.Color(255, 255, 0)
            size = 0.12

        world.debug.draw_point(
            loc,
            size=size,
            color=color,
            life_time=life_time
        )

        if i < len(route) - 1:
            next_loc = route[i + 1].transform.location + carla.Location(z=0.5)
            world.debug.draw_line(
                loc,
                next_loc,
                thickness=0.08,
                color=carla.Color(0, 0, 255),
                life_time=life_time
            )


def main():
    client = carla.Client("localhost", 2000)
    client.set_timeout(5.0)

    world = client.get_world()
    carla_map = world.get_map()

    original_settings = world.get_settings()

    vehicle = None

    try:
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        world.apply_settings(settings)

        planner = CarlaGraphPlanner(
            carla_map,
            sampling_resolution=2.0,
            lane_change_cost_factor=1.5,
            uturn_cost_factor=5.0,
            enable_lane_change=True,
            enable_uturn=True,
        )

        print("Planner summary:", planner.debug_summary())

        spawn_points = carla_map.get_spawn_points()
        if len(spawn_points) < 2:
            print("Not enough spawn points.")
            return

        start_point = random.choice(spawn_points)
        goal_point = random.choice(spawn_points)

        while goal_point.location.distance(start_point.location) < 5.0:
            goal_point = random.choice(spawn_points)

        print("Start:", start_point.location)
        print("Goal :", goal_point.location)

        route = planner.plan(
            start_point.location,
            goal_point.location
        )

        if not route:
            print("No route found.")
            return

        print("Route length:", len(route))

        draw_route(world, route, life_time=60.0)

        world.debug.draw_point(
            start_point.location + carla.Location(z=1.0),
            size=0.2,
            color=carla.Color(255, 0, 255),
            life_time=60.0
        )
        world.debug.draw_point(
            goal_point.location + carla.Location(z=1.0),
            size=0.2,
            color=carla.Color(0, 255, 255),
            life_time=60.0
        )

        vehicle = spawn_vehicle(world, start_point)
        if vehicle is None:
            return

        world.tick()

        controller = RouteFollower(
            vehicle,
            world,
            target_speed=8.0,
            steer_k=1.0,
            steer_soft_term=1.0,
            speed_kp=0.18,
            waypoint_reach_dist=4.0,
        )

        controller.set_route(route)

        for _ in range(3000):
            world.tick()
            control = controller.run_step()
            vehicle.apply_control(control)
            update_spectator(vehicle)
            time.sleep(0.02)
            if controller.is_finished():
                print("Route finished.")
                break

        time.sleep(2000)

    finally:
        world.apply_settings(original_settings)
        if vehicle is not None:
            vehicle.destroy()
if __name__ == "__main__":
    main()
