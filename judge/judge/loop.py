import pygame
import carla
from .world import World
from .config import *
from .ui import HUD
from .keyboard_control import KeyboardControl
from .vehicle import Vehicle
from .utils import get_actor_display_name, planar_distance
from .npc import Npc
import math
import datetime
from .obstacle import Obstacle

# from .agent import TaAgent
from pygame.time import Clock
from .state import State, Task


def game_loop(args):
    pygame.init()
    pygame.font.init()
    world = None
    original_settings = None

    try:
        ## Initialize world
        client = carla.Client(args.host, args.port)
        client.set_timeout(20.0)
        client.load_world(WORLD)

        sim_world = client.get_world()
        # spec = sim_world.get_spectator()
        # spec.set_transform(OBSTACLE2_POSE)
        if args.sync:
            original_settings = sim_world.get_settings()
            settings = sim_world.get_settings()
            if not settings.synchronous_mode:
                settings.synchronous_mode = True
                settings.fixed_delta_seconds = 0.05
            sim_world.apply_settings(settings)

            traffic_manager = client.get_trafficmanager()
            traffic_manager.set_synchronous_mode(True)

        if args.autopilot and not sim_world.get_settings().synchronous_mode:
            print(
                "WARNING: You are currently in asynchronous mode and could "
                "experience some issues with the traffic simulation"
            )

        ## Initialize pygame display
        display = pygame.display.set_mode(
            (args.width, args.height), pygame.HWSURFACE | pygame.DOUBLEBUF
        )
        display.fill((0, 0, 0))
        pygame.display.flip()

        hud = HUD(args.width, args.height)

        # Initialize state
        state = State()
        state.save_point = INIT_TRANS

        for (name, src, tgt) in TASK_ENDPOINTS:
            task = Task(name, src, tgt)
            state.pending_tasks.append(task)

        ## Create NPC cars
        npc1 = Npc("npc1", sim_world, 10, NPC1_ROUTE)
        npc2 = Npc("npc2", sim_world, 10, NPC2_ROUTE)
        npc3 = Npc("npc3", sim_world, 20, NPC3_ROUTE)
        obstacle1 = Obstacle(sim_world, OBSTACLE1_POSE)
        obstacle2 = Obstacle(sim_world, OBSTACLE2_POSE)

        ## Create the player
        player = Vehicle(
            "hero",
            sim_world,
            hud,
            spawn_point=INIT_TRANS,
            gamma=args.gamma,
            actor_filter=args.actor_filter,
            actor_generation=args.actor_generation,
        )
        player.listen_collision(
            lambda vehicle, event: on_collision(vehicle, event, state)
        )
        player.listen_lane_invasion(
            lambda vehicle, event: on_lane_invasion(vehicle, event, state)
        )

        # Set keyboard controller
        world = World(sim_world, hud, args)
        controller = KeyboardControl(player, hud, args.autopilot)

        if args.sync:
            sim_world.tick()
        else:
            sim_world.wait_for_tick()

        clock = Clock()

        # Looping
        while True:
            if args.sync:
                sim_world.tick()
            clock.tick_busy_loop(60)

            if controller.parse_events(client, world, hud, player, clock, args.sync):
                return

            player_pose = player.actor.get_transform()
            player_speed = player.actor.get_velocity().length()

            # Update actors
            player.step()
            npc1.step()
            npc2.step()
            npc3.step()

            # Update state
            if state.finished:
                pass

            elif state.is_starting:
                dist = planar_distance(player_pose.location, INIT_TRANS.location)
                if dist > LANE_INV_DISTANCE_THRESHOLD:
                    state.is_starting = False

            elif state.collision_detected:
                state.collision_detected = False
                player.actor.set_transform(state.save_point)
                # player.stop()

            elif state.curr_task is not None:
                dist = planar_distance(player_pose.location, state.curr_task.tgt)

                if (
                    dist <= CHECKPOINT_DISTANCE_THRESHOLD
                    and player_speed <= STOP_SPEED_THERSHOLD
                ):
                    hud.notification("Task {} finishes".format(state.curr_task.name))
                    state.save_point = player_pose
                    state.finished_tasks.append(state.curr_task)
                    state.curr_task = None

            elif len(state.pending_tasks) > 0:
                for (task_idx, task) in enumerate(state.pending_tasks):
                    dist = planar_distance(player_pose.location, task.src)

                    if (
                        dist < CHECKPOINT_DISTANCE_THRESHOLD
                        and player_speed <= STOP_SPEED_THERSHOLD
                    ):
                        state.save_point = player_pose
                        state.curr_task = task
                        del state.pending_tasks[task_idx]
                        hud.notification("Task {} starts".format(state.curr_task.name))
                        break

            elif player_speed > STOP_SPEED_THERSHOLD:
                pass
            else:
                state.finished = True
                hud.notification("Cheers! You finish the game.")

            # Update score
            if not state.finished:
                state.score = compute_score(state)

            # Update UI
            update_hud(state, hud, player, sim_world, clock)
            render(hud, player, display)
            pygame.display.flip()

            # Increase tick count
            state.n_ticks += 1

    finally:
        if original_settings:
            sim_world.apply_settings(original_settings)

        if world and world.recording_enabled:
            client.stop_recorder()

        pygame.quit()


def update_hud(state: State, hud: HUD, player: Vehicle, world: carla.World, clock):
    # score = compute_score(state)
    hud._notifications.tick(world, clock)
    if not hud._show_info:
        return
    t = player.actor.get_transform()
    v = player.actor.get_velocity()
    c = player.actor.get_control()
    compass = player.imu_sensor.compass
    heading = "N" if compass > 270.5 or compass < 89.5 else ""
    heading += "S" if 90.5 < compass < 269.5 else ""
    heading += "E" if 0.5 < compass < 179.5 else ""
    heading += "W" if 180.5 < compass < 359.5 else ""
    colhist = player.get_collision_history()
    dist_meter = player.distance_meter
    collision = [colhist[x + hud.frame - 200] for x in range(0, 200)]
    max_col = max(1.0, max(collision))
    collision = [x / max_col for x in collision]
    vehicles = world.get_actors().filter("vehicle.*")

    curr_task_name = None
    if state.curr_task is not None:
        curr_task_name = state.curr_task.name

    pending_task_names = " ".join(task.name for task in state.pending_tasks)
    finished_task_names = " ".join(task.name for task in state.finished_tasks)

    hud._info_text = [
        "Server:  % 16.0f FPS" % hud.server_fps,
        "Client:  % 16.0f FPS" % clock.get_fps(),
        "",
        "Vehicle: % 20s" % get_actor_display_name(player.actor, truncate=20),
        "Map:     % 20s" % world.get_map().name.split("/")[-1],
        "Simulation time: % 12s" % datetime.timedelta(seconds=int(hud.simulation_time)),
        "N ticks: %d" % state.n_ticks,
        "Traveled dist.: %fm" % dist_meter,
        "Score: %f" % state.score,
        "",
        "Speed:   % 15.0f km/h" % (3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2)),
        "Compass:% 17.0f\N{DEGREE SIGN} % 2s" % (compass, heading),
        "Accelero: (%5.2f,%5.2f,%5.2f)" % (player.imu_sensor.accelerometer),
        "Gyroscop: (%5.2f,%5.2f,%5.2f)" % (player.imu_sensor.gyroscope),
        "Location:% 20s"
        % ("(% 5.2f, % 5.2f, % 5.2f)" % (t.location.x, t.location.y, t.location.z)),
        "Rotation:% 20s"
        % (
            "(% 5.2f, % 5.2f, % 5.2f)"
            % (t.rotation.pitch, t.rotation.yaw, t.rotation.roll)
        ),
        "GNSS:% 24s"
        % ("(% 2.6f, % 3.6f)" % (player.gnss_sensor.lat, player.gnss_sensor.lon)),
        "Height:  % 18.0f m" % t.location.z,
        # "Score:  %.2f" % score,
        "Total lane invasions:  %d" % state.lane_invasion_count,
        "Current task: %s" % curr_task_name,
        "Pending Tasks: %s" % pending_task_names,
        "Finished Tasks: %s" % finished_task_names,
    ]
    if isinstance(c, carla.VehicleControl):
        hud._info_text += [
            ("Throttle:", c.throttle, 0.0, 1.0),
            ("Steer:", c.steer, -1.0, 1.0),
            ("Brake:", c.brake, 0.0, 1.0),
            ("Reverse:", c.reverse),
            ("Hand brake:", c.hand_brake),
            ("Manual:", c.manual_gear_shift),
            "Gear:        %s" % {-1: "R", 0: "N"}.get(c.gear, c.gear),
        ]
    elif isinstance(c, carla.WalkerControl):
        hud._info_text += [("Speed:", c.speed, 0.0, 5.556), ("Jump:", c.jump)]
    hud._info_text += [
        "",
        "Collision:",
        collision,
        "",
        "Number of vehicles: % 8d" % len(vehicles),
    ]
    if len(vehicles) > 1:
        hud._info_text += ["Nearby vehicles:"]
        distance = lambda l: math.sqrt(
            (l.x - t.location.x) ** 2
            + (l.y - t.location.y) ** 2
            + (l.z - t.location.z) ** 2
        )
        vehicles = [
            (distance(x.get_location()), x) for x in vehicles if x.id != player.actor.id
        ]
        for d, vehicle in sorted(vehicles, key=lambda vehicles: vehicles[0]):
            if d > 200.0:
                break
            vehicle_type = get_actor_display_name(vehicle, truncate=22)
            hud._info_text.append("% 4dm %s" % (d, vehicle_type))


def render(hud: HUD, player: Vehicle, display):
    player.render(display)

    if hud._show_info:
        info_surface = pygame.Surface((220, hud.dim[1]))
        info_surface.set_alpha(100)
        display.blit(info_surface, (0, 0))
        v_offset = 4
        bar_h_offset = 100
        bar_width = 106
        for item in hud._info_text:
            if v_offset + 18 > hud.dim[1]:
                break
            if isinstance(item, list):
                if len(item) > 1:
                    points = [
                        (x + 8, v_offset + 8 + (1.0 - y) * 30)
                        for x, y in enumerate(item)
                    ]
                    pygame.draw.lines(display, (255, 136, 0), False, points, 2)
                item = None
                v_offset += 18
            elif isinstance(item, tuple):
                if isinstance(item[1], bool):
                    rect = pygame.Rect((bar_h_offset, v_offset + 8), (6, 6))
                    pygame.draw.rect(
                        display, (255, 255, 255), rect, 0 if item[1] else 1
                    )
                else:
                    rect_border = pygame.Rect(
                        (bar_h_offset, v_offset + 8), (bar_width, 6)
                    )
                    pygame.draw.rect(display, (255, 255, 255), rect_border, 1)
                    f = (item[1] - item[2]) / (item[3] - item[2])
                    if item[2] < 0.0:
                        rect = pygame.Rect(
                            (bar_h_offset + f * (bar_width - 6), v_offset + 8),
                            (6, 6),
                        )
                    else:
                        rect = pygame.Rect(
                            (bar_h_offset, v_offset + 8), (f * bar_width, 6)
                        )
                    pygame.draw.rect(display, (255, 255, 255), rect)
                item = item[0]
            if item:  # At this point has to be a str.
                surface = hud._font_mono.render(item, True, (255, 255, 255))
                display.blit(surface, (8, v_offset))
            v_offset += 18
    hud._notifications.render(display)
    hud.help.render(display)


# def judge(state: State, hud: HUD):
#     score = compute_score(state)

#     if state.collision:
#         hud.notification("SCORE: %.2f because collision happend" % score)

#     elif state.finished:
#         hud.notification(
#             "SCORE: %.2f (%d lane invasions)" % (score, state.lane_invasion_count)
#         )

#     else:
#         hud.notification(
#             "SCORE: %.2f (%d lane invasions)" % (score, state.lane_invasion_count)
#         )


# def compute_score(state: State):
#     if state.collision:
#         return 0

#     elif state.finished:
#         lane_invasion_penalty = (
#             max(state.lane_invasion_count - 5, 0) * LANE_INVASION_PENALTY
#         )
#         # exceed_last_ckpt_penalty = (
#         #     EXCEED_LAST_CHECKPOINT_PENALTY if state.exceed_last_checkpoint else 0.0
#         # )
#         penalty = lane_invasion_penalty
#         checkpoint_score = CHECKPOINTS[-1].score
#         score = max(0.0, checkpoint_score - penalty)
#         return score

#     else:
#         lane_invasion_penalty = (
#             max(state.lane_invasion_count - 5, 0) * LANE_INVASION_PENALTY
#         )
#         penalty = lane_invasion_penalty

#         if state.checkpoint_index > 0:
#             checkpoint_score = CHECKPOINTS[state.checkpoint_index - 1].score
#         else:
#             checkpoint_score = 0.0

#         score = max(0.0, checkpoint_score - penalty)
#         return score


def on_collision(vehicle, event, state):
    state.collision_detected = True
    state.collision_count += 1
    # actor_type = get_actor_display_name(event.other_actor)
    # vehicle.hud.notification("Collision with %r" % actor_type)

    impulse = event.normal_impulse
    intensity = math.sqrt(impulse.x**2 + impulse.y**2 + impulse.z**2)

    vehicle.collision_history.append((event.frame, intensity))
    if len(vehicle.collision_history) > 4000:
        vehicle.collision_history.pop(0)


def on_lane_invasion(vehicle, event, state):
    # lane_types = set(x.type for x in event.crossed_lane_markings)
    # text = ["%r" % str(x).split()[-1] for x in lane_types]
    # vehicle.hud.notification("Crossed line %s" % " and ".join(text))

    violated = False
    player_loc = vehicle.actor.get_location()

    if state.is_starting:
        return

    if state.last_lane_inv_tick is not None:
        if state.last_lane_inv_tick + LANE_INVASION_COLD_TICKS >= state.n_ticks:
            return

    elif state.curr_task is not None:
        dist = planar_distance(state.curr_task.tgt, player_loc)

        if dist <= LANE_INV_DISTANCE_THRESHOLD:
            return

    else:
        for task in state.pending_tasks:
            dist = planar_distance(task.src, player_loc)

            if dist <= LANE_INV_DISTANCE_THRESHOLD:
                return

    for mark in event.crossed_lane_markings:
        if mark.type != carla.LaneMarkingType.Broken:
            violated = True
            break

    if violated:
        state.lane_invasion_count += 1
        state.last_lane_inv_tick = state.n_ticks


def compute_score(state: State):
    score = 40

    if state.curr_task is not None:
        score += 10

    score += len(state.finished_tasks) * 20
    score -= state.lane_invasion_count * LANE_INVASION_PENALTY
    score = max(0, score)
    return score
