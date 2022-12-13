import re
from carla import WeatherParameters, Vector3D, Actor, World, Location


def planar_distance(lhs: Location, rhs: Location) -> float:
    return ((lhs.x - rhs.x) ** 2 + (lhs.y - rhs.y) ** 2) ** 0.5


def find_weather_presets():
    rgx = re.compile(".+?(?:(?<=[a-z])(?=[A-Z])|(?<=[A-Z])(?=[A-Z][a-z])|$)")
    name = lambda x: " ".join(m.group(0) for m in rgx.finditer(x))
    presets = [x for x in dir(WeatherParameters) if re.match("[A-Z].+", x)]
    return [(getattr(WeatherParameters, x), name(x)) for x in presets]


def get_actor_display_name(actor: Actor, truncate=250):
    name = " ".join(actor.type_id.replace("_", ".").title().split(".")[1:])
    return (name[: truncate - 1] + "\u2026") if len(name) > truncate else name


def get_actor_blueprints(world: World, filter: str, generation: str):
    bps = world.get_blueprint_library().filter(filter)

    if generation.lower() == "all":
        return bps

    # If the filter returns only one bp, we assume that this one needed
    # and therefore, we ignore the generation
    if len(bps) == 1:
        return bps

    try:
        int_generation = int(generation)
        # Check if generation is in available generations
        if int_generation in [1, 2]:
            bps = [
                x for x in bps if int(x.get_attribute("generation")) == int_generation
            ]
            return bps
        else:
            print(
                "   Warning! Actor Generation is not valid. No actor will be spawned."
            )
            return []
    except:
        print("   Warning! Actor Generation is not valid. No actor will be spawned.")
        return []


def get_actor_bounding_extent(actor: Actor) -> Vector3D:
    orig_extent = actor.bounding_box.extent
    new_extent = Vector3D(orig_extent.x + 0.5, orig_extent.y + 0.5, orig_extent.z + 0.5)
    return new_extent
