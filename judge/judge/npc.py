from carla import World
from agents.navigation.basic_agent import BasicAgent


class Npc:
    def __init__(self, role_name: str, world: World, speed: float, points):
        assert len(points) >= 1

        blu = world.get_blueprint_library().find("vehicle.tesla.model3")
        blu.set_attribute("role_name", role_name)
        actor = world.spawn_actor(blu, points[0])

        physics_control = actor.get_physics_control()
        physics_control.use_sweep_wheel_collision = True
        actor.apply_physics_control(physics_control)

        # MAGIC here. Must tick once to make sure the vehicle data is initialized.
        world.tick()

        agent = BasicAgent(actor, speed)
        agent.set_destination(points[0].location)

        self.actor = actor
        self.agent = agent
        self.points = points
        self.next_index = 0

    def step(self):
        if self.agent.done():
            self.next_index = (self.next_index + 1) % len(self.points)
            next_point = self.points[self.next_index].location
            self.agent.set_destination(next_point)

        control = self.agent.run_step()
        control.manual_gear_shift = False
        self.actor.apply_control(control)
