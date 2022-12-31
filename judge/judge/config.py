from carla import Transform, Location, Rotation, Transform
import random

WORLD = "Town03"
SPEC_HEIGHT = 3
INIT_TRANS = Transform(Location(145.968, -105.675, 8.1), Rotation(0.0, 90.0, 0.0))
SPECTATOR_TRANS = Transform(
    Location(
        INIT_TRANS.location.x,
        INIT_TRANS.location.y,
        INIT_TRANS.location.z + SPEC_HEIGHT,
    ),
    INIT_TRANS.rotation,
)

LIDAR_RANGE_M = 30
STOP_VEL_THRESH = 0.01
NPC1_ROUTE = [
    # Transform(Location(-3.247418, 23.639963, 0.1), Rotation(0.0, 0.84, 0.0)),
    # Transform(Location(1.474368, -22.95079, 0.1), Rotation(0.0, -174, 0.0)),
    Transform(
        Location(13.695746, 18.811819, 0.1), Rotation(-0.000031, -43.884972, -0.007568)
    ),
    Transform(
        Location(22.825905, 1.144211, 0.1), Rotation(0.003324, -89.318871, -0.000676)
    ),
    Transform(
        Location(18.419220, -13.330012, 0.1),
        Rotation(-0.000854, -129.323746, -0.000246),
    ),
    Transform(
        Location(-13.913259, -18.567612, 0.1), Rotation(-0.000031, 140.858994, 0.021522)
    ),
    Transform(
        Location(-23.580221, -2.984560, 0.1), Rotation(-0.000122, 99.207008, -0.014651)
    ),
    Transform(
        Location(-20.525547, 12.254155, 0.1), Rotation(-0.000305, 57.465370, -0.036132)
    ),
]
NPC2_ROUTE = [
    Transform(Location(165.969940, -194.172440, 0.1), Rotation(0.0, 1.005202, 0.0)),
    Transform(Location(232.168045, -21.722141, 0.1), Rotation(0.0, 92.112457, 0.0)),
]
NPC3_ROUTE = [
    Transform(Location(-16.424643, -135, 0.1), Rotation(0.0, 1.1394, 0.0)),
    Transform(Location(7.562178, -181.43866, 0.1), Rotation(0.0, -88.1558, 0.0)),
    Transform(Location(-46.9482, -205.7256, 0.1), Rotation(0.0, -177.528, 0.0)),
    Transform(Location(-84.9675, -154.1095, 0.1), Rotation(0.0, 91.6498, 0.0)),
]


rand1 = random.uniform(0, 0.4)
rand2 = random.uniform(0.6, 1)
if random.uniform(0, 1) > 0.5:
    rand1, rand2 = rand2, rand1

def inter(a1, a2, m):
    return a1 * (1 - m) + a2 * m

def loc_inter(v1, v2, m):
    return Location(inter(v1[0], v2[0], m), inter(v1[1], v2[1], m), inter(v1[2], v2[2], m))


v1 = (-88.790787, -124.746361, 0.1)
v2 = (-88.045822, 117.770729, 0.1)
location = loc_inter(v1, v2, rand1)
OBSTACLE1_POSE = Transform(location, Rotation(0.0, 179.820488, 0.989469))

v1 = (-85.305145, -124.721504, 0.1)
v2 = (-84.569199, 118.390099, 0.1)
location = loc_inter(v1, v2, rand2)
OBSTACLE2_POSE = Transform(location, Rotation(0.0, 179.820488, 0.989469))


TASK_ENDPOINTS = [
    ("A", Location(-8.885858, -70.011162, 0), Location(12.04340, -94.9872, 0)),
    ("B", Location(173.494965, -190.685989, 0), Location(229.268784, -28.728645, 0)),
    ("C", Location(63.244614, -209.547195, 0), Location(-107.195251, 129.096527, 0)),
]

# TARGET_LOCATION = Location(15.71, -240.04, 0.18)
STOP_SPEED_THERSHOLD = 1e-3
CHECKPOINT_DISTANCE_THRESHOLD = 1.0
LANE_INV_DISTANCE_THRESHOLD = 8.0
LANE_INVASION_PENALTY = 1.0
LANE_INVASION_COLD_TICKS = 30
