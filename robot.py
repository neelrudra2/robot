import random
import heapq
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
from dataclasses import dataclass, field
from typing import List, Tuple, Optional, Set

GRID_SIZE = (10, 10)
NUM_ROBOTS = 3
NUM_PARCELS = 6
MAX_STEPS = 400
random.seed(2)

def astar(start: Tuple[int,int], goal: Tuple[int,int], grid_shape: Tuple[int,int], obstacles: Set[Tuple[int,int]]):
    R, C = grid_shape
    def neighbors(node):
        r, c = node
        for dr, dc in ((1,0),(-1,0),(0,1),(0,-1)):
            nr, nc = r+dr, c+dc
            if 0 <= nr < R and 0 <= nc < C and (nr, nc) not in obstacles:
                yield (nr, nc)
    def h(a,b):
        return abs(a[0]-b[0]) + abs(a[1]-b[1])
    open_set = [(h(start,goal), 0, start)]
    came_from = {}
    gscore = {start:0}
    closed = set()
    while open_set:
        _, g, current = heapq.heappop(open_set)
        if current == goal:
            path = []
            while current != start:
                path.append(current)
                current = came_from[current]
            path.reverse()
            return path
        if current in closed:
            continue
        closed.add(current)
        for nb in neighbors(current):
            tentative_g = g + 1
            if tentative_g < gscore.get(nb, 1e9):
                gscore[nb] = tentative_g
                came_from[nb] = current
                heapq.heappush(open_set, (tentative_g + h(nb, goal), tentative_g, nb))
    return None

@dataclass
class Parcel:
    id: int
    pos: Tuple[int,int]
    dest: Tuple[int,int]
    picked: bool = False

@dataclass
class Robot:
    id: int
    pos: Tuple[int,int]
    path: List[Tuple[int,int]] = field(default_factory=list)
    carrying: Optional[Parcel] = None
    state: str = "IDLE"
    target_parcel_id: Optional[int] = None

R, C = GRID_SIZE
bins = [(i, C-1) for i in range(min(NUM_PARCELS, R))]

parcels: List[Parcel] = []
positions = set()
pid = 0
while len(parcels) < NUM_PARCELS:
    r = random.randint(0, R-1)
    c = random.randint(0, C-2)
    if (r,c) in positions:
        continue
    dest = random.choice(bins)
    parcels.append(Parcel(id=pid, pos=(r,c), dest=dest))
    positions.add((r,c))
    pid += 1

robots: List[Robot] = []
for i in range(NUM_ROBOTS):
    robots.append(Robot(id=i, pos=(i % R, 0)))

reserved_parcels: Set[int] = set()
delivered_count = 0
step = 0

def nearest_unreserved_parcel(robot: Robot, parcels: List[Parcel], reserved: Set[int]):
    best = None
    bestd = 1e9
    for p in parcels:
        if p.id in reserved or p.picked:
            continue
        d = abs(robot.pos[0]-p.pos[0]) + abs(robot.pos[1]-p.pos[1])
        if d < bestd:
            bestd, best = d, p
    return best

def get_state_arrays():
    parcel_arr = np.zeros((R,C))
    dest_arr = np.zeros((R,C))
    for p in parcels:
        if not p.picked:
            parcel_arr[p.pos] = 1
        dest_arr[p.dest] = 1
    robot_positions = [r.pos for r in robots]
    return parcel_arr, dest_arr, robot_positions

def simulation_step():
    global step, delivered_count, reserved_parcels
    step += 1
    obstacles = set([r.pos for r in robots])

    for r in robots:
        if r.state == "IDLE":
            p = nearest_unreserved_parcel(r, parcels, reserved_parcels)
            if p:
                reserved_parcels.add(p.id)
                r.target_parcel_id = p.id
                path = astar(r.pos, p.pos, (R,C), obstacles - {r.pos})
                if path is not None:
                    r.path = path
                    r.state = "TO_PICKUP"
                else:
                    reserved_parcels.discard(p.id)
                    r.target_parcel_id = None

    for r in robots:
        if (r.state == "TO_PICKUP" and not r.path) or (r.state == "TO_DEST" and not r.path and r.carrying):
            if r.state == "TO_PICKUP" and r.target_parcel_id is not None:
                target = next((p for p in parcels if p.id == r.target_parcel_id), None)
                if target:
                    newpath = astar(r.pos, target.pos, (R,C), obstacles - {r.pos})
                    if newpath:
                        r.path = newpath
            elif r.state == "TO_DEST" and r.carrying is not None:
                newpath = astar(r.pos, r.carrying.dest, (R,C), obstacles - {r.pos})
                if newpath:
                    r.path = newpath

    planned_next_positions = {}
    for r in robots:
        planned_next_positions[r.id] = r.path[0] if r.path else r.pos

    target_counts = {}
    for pos in planned_next_positions.values():
        target_counts[pos] = target_counts.get(pos, 0) + 1

    for r in robots:
        nextpos = planned_next_positions[r.id]
        if nextpos != r.pos and target_counts[nextpos] > 1:
            claimants = [rid for rid,p in planned_next_positions.items() if p == nextpos]
            if min(claimants) != r.id:
                continue
        if r.path:
            r.pos = r.path.pop(0)

        if r.state == "TO_PICKUP" and r.target_parcel_id is not None:
            target = next((p for p in parcels if p.id == r.target_parcel_id), None)
            if target and r.pos == target.pos and not target.picked:
                target.picked = True
                r.carrying = target
                path_to_dest = astar(r.pos, target.dest, (R,C), set([x.pos for x in robots]) - {r.pos})
                r.path = path_to_dest if path_to_dest else []
                r.state = "TO_DEST"

        elif r.state == "TO_DEST" and r.carrying:
            if r.pos == r.carrying.dest:
                pid = r.carrying.id
                delivered_count += 1
                try:
                    parcels.remove(r.carrying)
                except ValueError:
                    pass
                reserved_parcels.discard(pid)
                r.carrying = None
                r.target_parcel_id = None
                r.state = "IDLE"
                r.path = []

fig, ax = plt.subplots(figsize=(6,6))
ax.set_title("Multi-Robot Warehouse Simulation")
ax.set_xticks(np.arange(-0.5, C, 1))
ax.set_yticks(np.arange(-0.5, R, 1))
ax.set_xlim(-0.5, C-0.5)
ax.set_ylim(R-0.5, -0.5)
ax.grid(True)

robot_scat = ax.scatter([], [], s=200)
parcel_scat = ax.scatter([], [], s=120, marker='s')
dest_scat = ax.scatter([], [], s=80, marker='x')
text_stats = ax.text(0.02, 1.02, "", transform=ax.transAxes)

def init():
    robot_scat.set_offsets(np.empty((0,2)))
    parcel_scat.set_offsets(np.empty((0,2)))
    dest_scat.set_offsets(np.empty((0,2)))
    text_stats.set_text("")
    return robot_scat, parcel_scat, dest_scat, text_stats

def update(frame):
    if frame > MAX_STEPS or not parcels:
        return robot_scat, parcel_scat, dest_scat, text_stats
    simulation_step()
    parcel_arr, dest_arr, robot_positions = get_state_arrays()
    if parcel_arr.sum():
        pr = np.argwhere(parcel_arr == 1)
        parcel_scat.set_offsets([[c, r] for r,c in pr])
    else:
        parcel_scat.set_offsets(np.empty((0,2)))
    if dest_arr.sum():
        dr = np.argwhere(dest_arr == 1)
        dest_scat.set_offsets([[c, r] for r,c in dr])
    else:
        dest_scat.set_offsets(np.empty((0,2)))
    if robot_positions:
        coords = [[c, r] for r,c in robot_positions]
        colors = ['red' if r.carrying else 'blue' for r in robots]
        robot_scat.set_offsets(coords)
        robot_scat.set_color(colors)
    text_stats.set_text(f"Step: {step} | Parcels left: {len(parcels)} | Delivered: {delivered_count}")
    return robot_scat, parcel_scat, dest_scat, text_stats

ani = animation.FuncAnimation(fig, update, frames=MAX_STEPS, init_func=init, interval=300, blit=True)
plt.show()
