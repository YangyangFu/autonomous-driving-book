"""
A simple linear composition velocity profile generator for velocity planning
"""
from agents.navigation.trajectory import PathPoint, Path, Trajectory, TrajectoryPoint


class VelocityGenerator():

    def __init__(self, spiral: Path, current_speed: float, target_speed: float, a_max: float):
        """
        Initialize the velocity generator

        Args:
            spiral (Path): the path to follow
            current_speed (float): the current speed of the vehicle
            target_speed (float): the target speed of the vehicle
            a_max (float): the maximum acceleration of the vehicle
        """

        self._path = spiral
        self._target_speed = target_speed
        self._current_speed = current_speed
        self._a_max = a_max

        self._small_number = 1e-8

    def generate_velocity_profile(self):
        """
        Generate a linear velocity profile
        """
        if self._current_speed <= self._target_speed:
            ramp_distance =  self._cal_distance(self._current_speed, self._target_speed, self._a_max)
        else:
            ramp_distance = self._cal_distance(self._current_speed, self._target_speed, -self._a_max)

        # find the index of point where the ramp ends
        ramp_end_index = 0
        distance = self._path[0].s
        while ramp_end_index < len(self._path) - 1 and distance < ramp_distance:
            ramp_end_index += 1
            distance = self._path[ramp_end_index].s

        # generate velocity profile for the ramp
        traj = Trajectory()
        
        final_ramp_distance = self._path[ramp_end_index].s - self._path[0].s
        ramp_a = (self._target_speed**2 - self._current_speed**2) / (2 * final_ramp_distance + self._small_number)
        if ramp_a > self._a_max:
            ramp_a = self._a_max
        elif ramp_a < -self._a_max:
            ramp_a = -self._a_max


        for i in range(ramp_end_index+1):
            point = self._path[i]
            vi = self._cal_speed(self._current_speed, ramp_a, point.s - self._path[0].s)
            ti = (vi - self._current_speed) / (ramp_a + self._small_number)
            
            traj.push_back(point, vi, ti)

        # generate velocity profile for the rest of the path
        for i in range(ramp_end_index+1, len(self._path)):
            point = self._path[i]
            vi = self._target_speed
            ti = (self._path[i].s - self._path[i-1].s) / (self._target_speed + self._small_number)
            traj.push_back(point, vi, ti)

        return traj


    def _cal_distance(self, v0: float, vf: float, a: float):
        """
        Calculate the distance to accelerate from v0 to vf with acceleration a
        
        Args:
            v0 (float): initial velocity
            vf (float): final velocity
            a (float): acceleration

        """
        # validity check: vf-v0 and a should have the same sign
        if (vf - v0) * a < 0:
            raise ValueError("The velocity difference and acceleration should have the same sign")
        
        # avoid division by zero
        if abs(a) < self._small_number:
            return float('inf')

        return (vf**2 - v0**2) / (2 * a)
    
    def _cal_speed(self, vs: float, a: float, distance: float):
        """
        Calculate the final speed after accelerating from v0 with acceleration a over distance

        Args:
            v0 (float): initial velocity
            a (float): acceleration
            distance (float): distance to accelerate

        """
        vf_square = vs**2 + 2 * a * distance

        if vf_square <= 0:
            return 0
        
        return vf_square**0.5
    

# add some tests 
def test_velocity_generator():
    import math 

    # straight line path with reachable target speed
    path = Path([0, 1, 2, 3, 4], [0, 0, 0, 0, 0], [0, 0, 0, 0, 0], [0, 0, 0, 0, 0], [0, 1, 2, 3, 4])
    generator = VelocityGenerator(path, 0, 4, 2)
    traj = generator.generate_velocity_profile()
    for i, point in enumerate(traj):
        print(f"Point {i}: s={point.path_point.s}, v={point.v}, t={point.t}")

    assert len(traj) == 5
    assert traj[0].v == 0
    assert abs(traj[1].v - math.sqrt(2*2*traj[1].path_point.s)) < 1e-6
    assert abs(traj[2].v - math.sqrt(2*2*traj[2].path_point.s)) < 1e-6
    assert abs(traj[3].v - math.sqrt(2*2*traj[3].path_point.s)) < 1e-6
    assert abs(traj[4].v - 4) < 1e-6

    # straight line path with unreachable target speed
    path = Path([0, 1, 2, 3, 4], [0, 0, 0, 0, 0], [0, 0, 0, 0, 0], [0, 0, 0, 0, 0], [0, 1, 2, 3, 4])
    generator = VelocityGenerator(path, 0, 4, 1)
    traj = generator.generate_velocity_profile()
    for i, point in enumerate(traj):
        print(f"Point {i}: s={point.path_point.s}, v={point.v}, t={point.t}")
    
    assert len(traj) == 5
    assert traj[0].v == 0
    assert abs(traj[-1].v - 4) > 1e-6


if __name__ == '__main__':
    test_velocity_generator()
    print("All tests passed")