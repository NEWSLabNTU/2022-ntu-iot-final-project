import carla
import weakref


class LaneInvasionSensor(object):
    sensor = None
    callback = None

    def __init__(self, parent_actor):

        # If the spawn object is not a vehicle, we cannot use the Lane Invasion Sensor
        if parent_actor.type_id.startswith("vehicle."):
            self._parent = parent_actor
            world = self._parent.get_world()
            bp = world.get_blueprint_library().find("sensor.other.lane_invasion")
            self.sensor = world.spawn_actor(
                bp, carla.Transform(), attach_to=self._parent
            )
            # We need to pass the lambda a weak reference to self to avoid circular
            # reference.
            weak_self = weakref.ref(self)
            self.sensor.listen(
                lambda event: LaneInvasionSensor._on_invasion(weak_self, event)
            )

    def __del__(self):
        self.sensor.destroy()

    def set_callback(self, callback):
        self.callback = callback

    @staticmethod
    def _on_invasion(weak_self, event):
        me = weak_self()
        if not me:
            return

        if me.callback is not None:
            me.callback(event)
