import numpy as np
import random 

import carla
from agents.navigation.cubic_spiral_generator import CubicSpiral

random.seed(100)

def test_generate_spiral():
    
    try:
        # start carla 
        client = carla.Client('localhost', 2000)
        client.set_timeout(2.0)
        world = client.get_world()
        map = world.get_map()

        # spawn ego vehicle
        blueprint = world.get_blueprint_library().filter('vehicle.tesla.model3')[0]
        blueprint.set_attribute('role_name', 'hero')
        blueprint.set_attribute('color', '0,0,0')

        # random spawn point
        spawn_points = map.get_spawn_points()
        start = random.choice(spawn_points)
        
        # get waypoints from map
        start_wp = map.get_waypoint(start.location, project_to_road=True, lane_type=carla.LaneType.Driving)
        end_wp = start_wp.next(20)[0]
        # 
        spiral = CubicSpiral()
        spiral.generate_spiral(start_wp, end_wp)

    finally:
        if world is not None:
            world.destroy()

if __name__ == '__main__':
    from scipy.integrate import simpson 

    x = [i for i in range(9)]
    y = [i for i in range(9)]
    print(simpson(y, x=x))
    test_generate_spiral()
    print("All tests passed")


