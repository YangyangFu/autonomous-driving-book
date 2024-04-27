"""
Trajectory for spirals and spline generation
"""
from typing import List
from dataclasses import dataclass

@dataclass
class PathPoint:
    x: float
    y: float
    theta: float
    kappa: float
    s: float = 0.0

class Path:
    def __init__(self, x: List, y: List, theta: List, kappa: List, s: List = None):
        """
        Construct a trajectory of road points
        """
        
        # dimension check
        assert len(x) == len(y) == len(theta) == len(kappa) == len(s), "All input lists should have the same length"
        
        self.traj = []
        for i in range(len(x)):
            self.traj.append(PathPoint(x[i], y[i], theta[i], kappa[i], s[i]))
        
    def __len__(self):
        return len(self.traj)

    def push_back(self, x, y, theta, kappa, s=0.0):
        """
        Append a point to the trajectory
        """
        point = PathPoint(x, y, theta, kappa, s)
        self.traj.append(point)

    def pop_front(self):
        """
        Remove the first point from the trajectory
        """
        front = self.traj.pop(0)
        return front
    
    def __getitem__(self, i):
        """
        Get the point at index i
        """
        return self.traj[i]
    
    # add iterator:
    # iterrate over the trajectory point (x, y, theta, kappa)
    # add __iter__ and __next__ methods
    def __iter__(self):
        self._index = 0
        return self 

    def __next__(self):
        if self._index < len(self.traj):
            point = self.traj[self._index]
            self._index += 1
            return point
        else:
            raise StopIteration

@dataclass
class TrajectoryPoint:
    path_point: PathPoint
    v: float # velocity
    t: float # relative time from start

class Trajectory:
    def __init__(self):
        self.traj = []

    def push_back(self, path_point, v, t):
        """
        Append a point to the trajectory
        """
        point = TrajectoryPoint(path_point, v, t)
        self.traj.append(point)

    def pop_front(self):
        """
        Remove the first point from the trajectory
        """
        front = self.traj.pop(0)
        return front

    def __len__(self):
        return len(self.traj)

    def __getitem__(self, i):
        """
        Get the point at index i
        """
        return self.traj[i]

    def __iter__(self):
        self._index = 0
        return self

    def __next__(self):
        if self._index < len(self.traj):
            point = self.traj[self._index]
            self._index += 1
            return point
        else:
            raise StopIteration
        
# add some test
def test_path():
    traj = Path([0, 1, 2], [0, 1, 2], [0, 1, 2], [0, 1, 2], [0, 1, 2])
    assert len(traj) == 3
    traj.push_back(3, 3, 3, 3, 3)
    assert len(traj) == 4
    front = traj.pop_front()
    assert len(traj) == 3
    assert front.x == 0
    assert front.y == 0
    assert front.theta == 0
    assert front.kappa == 0
    assert front.s == 0

    # test iteration
    for i, point in enumerate(traj):
        assert point.x == i + 1
        assert point.y == i + 1
        assert point.theta == i + 1
        assert point.kappa == i + 1
        assert point.s == i + 1
    # test at
    for i in range(len(traj)):
        point = traj[i]
        assert point.x == i + 1
        assert point.y == i + 1
        assert point.theta == i + 1
        assert point.kappa == i + 1
        assert point.s == i + 1

    print("Path test passed")

# test trajectory
def test_trajectory():
    traj = Trajectory()
    path = Path([0, 1, 2], [0, 1, 2], [0, 1, 2], [0, 1, 2], [0, 1, 2])
    v_list = [10, 20, 30]
    t_list = [0, 1, 2]
    for i in range(len(path)):
        traj.push_back(path[i], v_list[i], t_list[i])
    
    assert len(traj) == 3
    front = traj.pop_front()
    assert len(traj) == 2
    assert front.path_point.x == 0
    assert front.path_point.y == 0
    assert front.path_point.theta == 0
    assert front.path_point.kappa == 0
    assert front.path_point.s == 0
    assert front.v == 10
    assert front.t == 0

    # test iteration
    for i, point in enumerate(traj):
        assert point.path_point.x == i + 1
        assert point.path_point.y == i + 1
        assert point.path_point.theta == i + 1
        assert point.path_point.kappa == i + 1
        assert point.path_point.s == i + 1
        assert point.v == v_list[i + 1]
        assert point.t == t_list[i + 1]
    
    # test at
    for i in range(len(traj)):
        point = traj[i]
        assert point.path_point.x == i + 1
        assert point.path_point.y == i + 1
        assert point.path_point.theta == i + 1
        assert point.path_point.kappa == i + 1
        assert point.path_point.s == i + 1
        assert point.v == v_list[i + 1]
        assert point.t == t_list[i + 1]

    print("Trajectory test passed")


if __name__ == "__main__":
    test_path()
    test_trajectory()