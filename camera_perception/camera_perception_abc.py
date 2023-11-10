from abc import ABC


class CameraPerceptionABC(ABC):
    def get_cones(self, frame):
        return frame, [], []
