import carla
import weakref
from ..utils import get_actor_display_name
import math


class CollisionSensor(object):
    callback = None

    def __init__(self, parent_actor):
        self.sensor = None
        self._parent = parent_actor
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find("sensor.other.collision")
        self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(
            lambda event: CollisionSensor._on_collision(weak_self, event)
        )

    def __del__(self):
        self.sensor.destroy()

    def set_callback(self, callback):
        self.callback = callback

    @staticmethod
    def _on_collision(weak_self, event):
        me = weak_self()
        if not me:
            return

        me.callback(event)

        # actor_type = get_actor_display_name(event.other_actor)
        # me.hud.notification("Collision with %r" % actor_type)
        # impulse = event.normal_impulse
        # intensity = math.sqrt(impulse.x**2 + impulse.y**2 + impulse.z**2)
        # me.history.append((event.frame, intensity))
        # if len(me.history) > 4000:
        #     me.history.pop(0)
