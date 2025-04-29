import numpy as np

class PathEvaluator:
    def __init__(self, sim, buffer_distance=0.5):
        self.sim = sim
        self.buffer_distance = buffer_distance

    def compute_smoothness(self, path):
        if not path or len(path) < 2:
            return 0.0

        safe_segments = 0
        total_segments = len(path) - 1

        for i in range(total_segments):
            p1 = path[i]
            p2 = path[i + 1]
            if self.is_segment_safe(p1, p2):
                safe_segments += 1

        return (safe_segments / total_segments) * 100.0

    def is_segment_safe(self, p1, p2, samples=10):
        for i in range(samples + 1):
            alpha = i / samples
            x = p1[0] * (1 - alpha) + p2[0] * alpha
            y = p1[1] * (1 - alpha) + p2[1] * alpha
            z = p1[2] * (1 - alpha) + p2[2] * alpha
            if self.is_near_obstacle((x, y, z)):
                return False
        return True

    def is_near_obstacle(self, point):
        for obs_id in self.sim.get_obstacle_ids():
            closest_points = self.sim.get_closest_points(obs_id, point, self.buffer_distance)
            if closest_points:
                return True
        return False
