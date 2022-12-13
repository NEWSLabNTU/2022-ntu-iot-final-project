from carla import World

class Obstacle:
    def __init__(self, world: World, coordinate):
        blu = world.get_blueprint_library().find("static.prop.mailbox")
        actor = world.spawn_actor(blu, coordinate)
        self.actor = actor
