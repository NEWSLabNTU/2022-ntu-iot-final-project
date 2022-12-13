import carla
import numpy as np
import pygame
import weakref
from ..ui import HUD
from ..utils import get_actor_bounding_extent
from carla import (
    ColorConverter as CC,
    Transform,
    Location,
    Rotation,
    AttachmentType,
    Vector3D,
)
from typing import Optional, Dict


DEFAULT_SENSOR_CONFIGS = [
    ["sensor.camera.rgb", CC.Raw, "Camera RGB", {}],
    # ["sensor.camera.depth", CC.Raw, "Camera Depth (Raw)", {}],
    # ["sensor.camera.depth", CC.Depth, "Camera Depth (Gray Scale)", {}],
    # [
    #     "sensor.camera.depth",
    #     CC.LogarithmicDepth,
    #     "Camera Depth (Logarithmic Gray Scale)",
    #     {},
    # ],
    # [
    #     "sensor.camera.semantic_segmentation",
    #     CC.Raw,
    #     "Camera Semantic Segmentation (Raw)",
    #     {},
    # ],
    # [
    #     "sensor.camera.semantic_segmentation",
    #     CC.CityScapesPalette,
    #     "Camera Semantic Segmentation (CityScapes Palette)",
    #     {},
    # ],
    # [
    #     "sensor.camera.instance_segmentation",
    #     CC.CityScapesPalette,
    #     "Camera Instance Segmentation (CityScapes Palette)",
    #     {},
    # ],
    # [
    #     "sensor.camera.instance_segmentation",
    #     CC.Raw,
    #     "Camera Instance Segmentation (Raw)",
    #     {},
    # ],
    ["sensor.lidar.ray_cast", None, "Lidar (Ray-Cast)", {"range": "50"}],
    # ["sensor.camera.dvs", CC.Raw, "Dynamic Vision Sensor", {}],
    # [
    #     "sensor.camera.rgb",
    #     CC.Raw,
    #     "Camera RGB Distorted",
    #     {
    #         "lens_circle_multiplier": "3.0",
    #         "lens_circle_falloff": "3.0",
    #         "chromatic_aberration_intensity": "0.5",
    #         "chromatic_aberration_offset": "0",
    #     },
    # ],
    # ["sensor.camera.optical_flow", CC.Raw, "Optical Flow", {}],
]


class CameraManager(object):
    def __init__(self, parent_actor, hud: HUD, gamma_correction):
        # Generate camera transformations
        if not parent_actor.type_id.startswith("walker.pedestrian"):
            bbox_extent = get_actor_bounding_extent(parent_actor)
            camera_transforms = generate_vehicle_transforms(bbox_extent)
        else:
            camera_transforms = generate_walker_transforms()

        # Configure sensors
        sensor_configs = DEFAULT_SENSOR_CONFIGS
        world = parent_actor.get_world()
        bp_library = world.get_blueprint_library()
        lidar_range = None

        sensors = list()

        for (kind, cc, desc, params) in sensor_configs:
            bp = bp_library.find(kind)
            if kind.startswith("sensor.camera"):
                bp.set_attribute("image_size_x", str(hud.dim[0]))
                bp.set_attribute("image_size_y", str(hud.dim[1]))
                if bp.has_attribute("gamma"):
                    bp.set_attribute("gamma", str(gamma_correction))
                for attr_name, attr_value in params.items():
                    bp.set_attribute(attr_name, attr_value)
            elif kind.startswith("sensor.lidar"):
                lidar_range = 50

                for attr_name, attr_value in params.items():
                    bp.set_attribute(attr_name, attr_value)
                    if attr_name == "range":
                        lidar_range = float(attr_value)

            sensors.append(SensorDesc(kind, cc, desc, params, bp))

        # Assign fields
        self.index = 0
        self.transform_index = 0
        self.sensors = sensors
        self.sensor = None
        self.surface = None
        self._parent = parent_actor
        self.hud = hud
        self.recording = False
        self._camera_transforms = camera_transforms
        self.lidar_range = lidar_range

        self.set_sensor(0, notify=False)

    def __del__(self):
        if self.sensor is not None:
            self.sensor.destroy()

    def toggle_camera(self):
        self.transform_index = (self.transform_index + 1) % len(self._camera_transforms)
        self.set_sensor(self.index, notify=False)

    def set_sensor(self, index: int, notify: bool = True):
        if self.sensor is not None:
            self.sensor.destroy()
            self.surface = None

        self.sensor = self._parent.get_world().spawn_actor(
            self.sensors[index].blueprint,
            self._camera_transforms[self.transform_index][0],
            attach_to=self._parent,
            attachment_type=self._camera_transforms[self.transform_index][1],
        )

        def sensor_callback(weak_me, image):
            me = weak_me()
            if me is None:
                return
            sensor = me.sensors[me.index]
            self.surface = CameraManager._parse_image(
                sensor.kind, sensor.cc, image, me.hud, me.recording, me.lidar_range
            )

        # We need to pass the lambda a weak reference to self to avoid
        # circular reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda image: sensor_callback(weak_self, image))

        if notify:
            self.hud.notification(self.sensors[index].desc)
        self.index = index

    def next_sensor(self):
        self.set_sensor((self.index + 1) % len(self.sensors))

    def toggle_recording(self):
        self.recording = not self.recording
        self.hud.notification("Recording %s" % ("On" if self.recording else "Off"))

    def render(self, display):
        if self.surface is not None:
            display.blit(self.surface, (0, 0))

    @staticmethod
    def _parse_image(
        kind: str,
        cc: Optional[CC],
        image: carla.Image,
        hud: HUD,
        recording: bool,
        lidar_range: float,
    ) -> pygame.Surface:
        if kind.startswith("sensor.lidar"):
            points = np.frombuffer(image.raw_data, dtype=np.dtype("f4"))
            points = np.reshape(points, (int(points.shape[0] / 4), 4))
            lidar_data = np.array(points[:, :2])
            lidar_data *= min(hud.dim) / (2.0 * lidar_range)
            lidar_data += (0.5 * hud.dim[0], 0.5 * hud.dim[1])
            lidar_data = np.fabs(lidar_data)  # pylint: disable=E1111
            lidar_data = lidar_data.astype(np.int32)
            lidar_data = np.reshape(lidar_data, (-1, 2))
            lidar_img_size = (hud.dim[0], hud.dim[1], 3)
            lidar_img = np.zeros((lidar_img_size), dtype=np.uint8)
            lidar_img[tuple(lidar_data.T)] = (255, 255, 255)
            surface = pygame.surfarray.make_surface(lidar_img)

        elif kind.startswith("sensor.camera.dvs"):
            # Example of converting the raw_data from a carla.DVSEventArray
            # sensor into a NumPy array and using it as an image
            dvs_events = np.frombuffer(
                image.raw_data,
                dtype=np.dtype(
                    [
                        ("x", np.uint16),
                        ("y", np.uint16),
                        ("t", np.int64),
                        ("pol", np.bool),
                    ]
                ),
            )
            dvs_img = np.zeros((image.height, image.width, 3), dtype=np.uint8)
            # Blue is positive, red is negative
            dvs_img[
                dvs_events[:]["y"], dvs_events[:]["x"], dvs_events[:]["pol"] * 2
            ] = 255
            surface = pygame.surfarray.make_surface(dvs_img.swapaxes(0, 1))

        elif kind.startswith("sensor.camera.optical_flow"):
            image = image.get_color_coded_flow()
            array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (image.height, image.width, 4))
            array = array[:, :, :3]
            array = array[:, :, ::-1]
            surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))

        else:
            assert cc is not None
            image.convert(cc)
            array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (image.height, image.width, 4))
            array = array[:, :, :3]
            array = array[:, :, ::-1]
            surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))

        if recording:
            image.save_to_disk("_out/%08d" % image.frame)

        return surface


class SensorDesc:
    kind: str
    cc: CC
    desc: str
    params: Dict[str, str]
    blueprint: carla.ActorBlueprint

    def __init__(
        self,
        kind: str,
        cc: CC,
        desc: str,
        params: Dict[str, str],
        blueprint: carla.ActorBlueprint,
    ):
        self.kind = kind
        self.cc = cc
        self.desc = desc
        self.params = params
        self.blueprint = blueprint


def generate_vehicle_transforms(extent: Vector3D):
    bound_x = extent.x
    bound_y = extent.y
    bound_z = extent.z

    return [
        (
            Transform(
                Location(x=-2.0 * bound_x, y=+0.0 * bound_y, z=2.0 * bound_z),
                Rotation(pitch=8.0),
            ),
            AttachmentType.SpringArm,
        ),
        (
            Transform(Location(x=+0.8 * bound_x, y=+0.0 * bound_y, z=1.3 * bound_z)),
            AttachmentType.Rigid,
        ),
        (
            Transform(Location(x=+1.9 * bound_x, y=+1.0 * bound_y, z=1.2 * bound_z)),
            AttachmentType.SpringArm,
        ),
        (
            Transform(
                Location(x=-2.8 * bound_x, y=+0.0 * bound_y, z=4.6 * bound_z),
                Rotation(pitch=6.0),
            ),
            AttachmentType.SpringArm,
        ),
        (
            Transform(Location(x=-1.0, y=-1.0 * bound_y, z=0.4 * bound_z)),
            AttachmentType.Rigid,
        ),
    ]


def generate_walker_transforms():
    return [
        (
            Transform(Location(x=-2.5, z=0.0), Rotation(pitch=-8.0)),
            AttachmentType.SpringArm,
        ),
        (Transform(Location(x=1.6, z=1.7)), AttachmentType.Rigid),
        (
            Transform(Location(x=2.5, y=0.5, z=0.0), Rotation(pitch=-8.0)),
            AttachmentType.SpringArm,
        ),
        (
            Transform(Location(x=-4.0, z=2.0), Rotation(pitch=6.0)),
            AttachmentType.SpringArm,
        ),
        (
            Transform(Location(x=0, y=-2.5, z=-0.0), Rotation(yaw=90.0)),
            AttachmentType.Rigid,
        ),
    ]
