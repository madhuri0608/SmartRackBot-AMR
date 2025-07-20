import math
import random

class YOLOSim:
    def __init__(self, tag_locations=None, confidence_threshold=0.5, max_range=5):
        self.tag_locations = tag_locations or []  # List of (x, y)
        self.confidence_threshold = confidence_threshold
        self.max_range = max_range

    def detect(self, robot_pos):
        for tag_pos in self.tag_locations:
            dx = tag_pos[0] - robot_pos[0]
            dy = tag_pos[1] - robot_pos[1]
            distance = math.hypot(dx, dy)
            if distance <= self.max_range:
                confidence = max(0.0, 1.0 - distance / self.max_range)
                if confidence >= self.confidence_threshold:
                    return tag_pos, round(confidence, 2)
        return None, 0.0
