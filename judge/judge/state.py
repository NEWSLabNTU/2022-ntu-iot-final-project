from carla import Transform, Location
from typing import List, Optional


class Task:
    name: str
    src: Location
    tgt: Location

    def __init__(self, name, src, tgt):
        self.name = name
        self.src = src
        self.tgt = tgt


class State:
    score: float = 40.0
    finished: bool = False
    lane_invasion_count: int = 0
    collision_count: int = 0
    collision_detected: bool = False
    pending_tasks: List[Task] = list()
    finished_tasks: List[Task] = list()
    curr_task: Optional[Task] = None
    save_point: Transform
    is_starting: bool = True
    n_ticks = 0
    last_lane_inv_tick = None
