"""
Trajectory for spirals and spline generation
"""
from typing import List
from dataclasses import dataclass

@dataclass
class RoadPoint:
    x: float
    y: float
    theta: float
    kappa: float

class Trajectory:
    def __init__(self, x: List, y: List, theta: List, kappa: List):
        """
        Construct a trajectory
        """
        
        # dimension check
        assert len(x) == len(y) == len(theta) == len(kappa), "All input lists should have the same length"
        
        self.traj = []
        for i in range(len(x)):
            self.traj.append(RoadPoint(x[i], y[i], theta[i], kappa[i]))
        
    def __len__(self):
        return len(self.traj)

    def push_back(self, x, y, theta, kappa):
        """
        Append a point to the trajectory
        """
        point = RoadPoint(x, y, theta, kappa)
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

# add some test
def test_trajectory():
    traj = Trajectory([0, 1, 2], [0, 1, 2], [0, 1, 2], [0, 1, 2])
    assert len(traj) == 3
    traj.push_back(3, 3, 3, 3)
    assert len(traj) == 4
    front = traj.pop_front()
    assert len(traj) == 3
    assert front.x == 0
    assert front.y == 0
    assert front.theta == 0
    assert front.kappa == 0

    # test iteration
    for i, point in enumerate(traj):
        assert point.x == i + 1
        assert point.y == i + 1
        assert point.theta == i + 1
        assert point.kappa == i + 1
    # test at
    for i in range(len(traj)):
        point = traj[i]
        assert point.x == i + 1
        assert point.y == i + 1
        assert point.theta == i + 1
        assert point.kappa == i + 1

    print("Trajectory test passed")

if __name__ == "__main__":
    test_trajectory()