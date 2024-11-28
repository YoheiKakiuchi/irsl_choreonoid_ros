import tf
from cnoid.IRSLCoords import coordinates

class TransformListener(tf.TransformListener):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
    def lookupCoords(self, target_frame, source_frame, time):
        trs, quat = super().lookupTransform(target_frame, source_frame, time)
        return coordinates(trs, quat)

class TransformBroadcaster(tf.TransformBroadcaster):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
    def sendCoords(self, coords, time, child, parent):
        pos = coords.pos
        translation = [pos[0], pos[1], pos[2]]
        q = coords.quaternion
        rotation = [q[0], q[1], q[2], q[3]]
        return self.sendTransform(translation, rotation, time, child, parent)
