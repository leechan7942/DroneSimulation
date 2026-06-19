#!/usr/bin/env python3

import heapq
import math


class LocalGridPlanner:
    def __init__(
        self,
        resolution=0.30,
        forward_range=7.0,
        side_range=6.0,
        backward_range=2.0,
        obstacle_inflation=0.40,
        clearance_radius=0.75,
        local_goal_distance=5.0,
        path_lookahead=0.8,
        min_target_distance=0.5,
        sample_step=1,
        backward_penalty=1.5,
        fallback_max_angle_deg=180.0,
        fallback_angle_step_deg=15.0,
        emergency_escape_distance=0.25,
        side_hysteresis_margin=0.35,
        side_hold_bonus=0.25,
        side_switch_penalty=0.20,
        heading_hold_bonus=0.25,
    ):
        self.resolution = resolution
        self.forward_range = forward_range
        self.side_range = side_range
        self.backward_range = backward_range
        self.obstacle_inflation = obstacle_inflation
        self.clearance_radius = clearance_radius
        self.local_goal_distance = local_goal_distance
        self.path_lookahead = path_lookahead
        self.min_target_distance = min_target_distance
        self.sample_step = max(1, sample_step)
        self.start_clear_radius = 0.35
        self.clearance_weight = 0.8
        self.backward_penalty = max(0.0, backward_penalty)
        self.fallback_max_angle_deg = min(180.0, max(90.0, fallback_max_angle_deg))
        self.fallback_angle_step_deg = max(1.0, fallback_angle_step_deg)
        self.emergency_escape_distance = max(0.10, emergency_escape_distance)
        self.side_hysteresis_margin = side_hysteresis_margin
        self.side_hold_bonus = side_hold_bonus
        self.side_switch_penalty = side_switch_penalty
        self.heading_hold_bonus = heading_hold_bonus
        self.side_deadband = max(self.resolution * 0.5, 0.08)
        self.previous_status = "global"
        self.previous_avoidance_side = 0
        self.previous_selected_heading = None

        self.min_ix = int(math.floor(-self.backward_range / self.resolution))
        self.max_ix = int(math.ceil(self.forward_range / self.resolution))
        self.min_iy = int(math.floor(-self.side_range / self.resolution))
        self.max_iy = int(math.ceil(self.side_range / self.resolution))

    def compute_offset(self, scan, yaw=0.0):
        offset_x, offset_y, _, _, _, _, _ = self.compute_avoidance(
            scan,
            yaw,
            (1.0, 0.0),
            0.30,
        )
        return offset_x, offset_y

    def compute_avoidance(self, scan, yaw=0.0, goal_body_vector=None, step_distance=0.30):
        goal_x, goal_y, goal_length = self.normalize_goal(goal_body_vector)
        blocked, min_distance = self.build_occupancy(scan)
        start_cell = (0, 0)
        local_goal = self.local_goal_point(goal_x, goal_y, goal_length)
        goal_cell = self.xy_to_cell(local_goal[0], local_goal[1])
        goal_blocked = goal_cell in blocked

        if scan is None:
            body_x, body_y = self.direct_step(goal_x, goal_y, step_distance)
            status = "scan 없음"
            active = False
        elif not goal_blocked and self.is_line_free(blocked, start_cell, goal_cell):
            body_x, body_y = self.direct_step(goal_x, goal_y, step_distance)
            status = "global"
            active = False
        else:
            path, status = self.plan_local_path(
                blocked,
                (goal_x, goal_y),
                local_goal,
                goal_cell,
                goal_blocked,
            )
            if path:
                body_x, body_y = self.step_from_path(path, step_distance)
                active = True
            else:
                body_x, body_y, fallback_held = self.fallback_step(
                    blocked,
                    (goal_x, goal_y),
                    step_distance,
                )
                if body_x or body_y:
                    progress = body_x * goal_x + body_y * goal_y
                    status = "fallback_reverse" if progress < -0.01 else "fallback"
                else:
                    status = "blocked_hold"
                if fallback_held:
                    status = self.append_hold_status(status, self.previous_avoidance_side)
                active = True

        selected_heading = math.atan2(body_y, body_x) if math.hypot(body_x, body_y) > 0.01 else 0.0
        goal_heading = math.atan2(goal_y, goal_x)
        goal_diff = abs(self.angle_diff(selected_heading, goal_heading))
        map_x, map_y = self.rotate_body_to_map(body_x, body_y, yaw)
        self.update_avoidance_memory(active, body_x, body_y, (goal_x, goal_y), selected_heading)
        self.previous_status = status
        return (
            map_x,
            map_y,
            active,
            min_distance,
            status,
            math.degrees(selected_heading),
            math.degrees(goal_diff),
        )

    def build_occupancy(self, scan):
        blocked = set()
        min_distance = float("inf")
        if scan is None:
            return blocked, min_distance

        inflation_cells = int(math.ceil(self.obstacle_inflation / self.resolution))
        for index in range(0, len(scan.ranges), self.sample_step):
            distance = scan.ranges[index]
            if not self.is_valid_distance(distance, scan):
                continue

            min_distance = min(min_distance, distance)
            if distance > max(self.forward_range, self.side_range):
                continue

            angle = scan.angle_min + index * scan.angle_increment
            obstacle_x = math.cos(angle) * distance
            obstacle_y = math.sin(angle) * distance
            center = self.xy_to_cell(obstacle_x, obstacle_y)

            for dx in range(-inflation_cells, inflation_cells + 1):
                for dy in range(-inflation_cells, inflation_cells + 1):
                    cell = (center[0] + dx, center[1] + dy)
                    if not self.in_bounds(cell):
                        continue
                    cell_x, cell_y = self.cell_to_xy(cell)
                    if math.hypot(cell_x - obstacle_x, cell_y - obstacle_y) <= self.obstacle_inflation:
                        blocked.add(cell)

        for cell in list(blocked):
            cell_x, cell_y = self.cell_to_xy(cell)
            if math.hypot(cell_x, cell_y) <= self.start_clear_radius:
                blocked.remove(cell)

        return blocked, min_distance

    def plan_local_path(self, blocked, goal_unit, local_goal, goal_cell, goal_blocked):
        start = (0, 0)
        distances = {start: 0.0}
        parents = {}
        queue = [(0.0, start)]
        clearance_cache = {}

        while queue:
            current_cost, current = heapq.heappop(queue)
            if current_cost > distances[current]:
                continue

            for neighbor, move_cost in self.neighbors(current):
                if neighbor in blocked:
                    continue
                clearance = self.clearance_at(neighbor, blocked, clearance_cache)
                clearance_cost = max(0.0, self.clearance_radius - clearance) * self.clearance_weight
                backward_cost = self.backward_penalty if neighbor[0] < 0 else 0.0
                new_cost = current_cost + move_cost + clearance_cost + backward_cost
                if new_cost >= distances.get(neighbor, float("inf")):
                    continue
                distances[neighbor] = new_cost
                parents[neighbor] = current
                heapq.heappush(queue, (new_cost, neighbor))

        goal_reachable = (not goal_blocked) and goal_cell in distances
        target, held_side = self.choose_reachable_target(
            distances,
            blocked,
            clearance_cache,
            goal_unit,
            local_goal,
        )
        if target is None:
            return [], "blocked_goal" if goal_blocked else "blocked"

        status = self.build_local_status(goal_blocked, goal_reachable, target, goal_unit, held_side)
        return self.reconstruct_path(parents, target), status

    def choose_reachable_target(self, distances, blocked, clearance_cache, goal_unit, local_goal):
        candidates = []
        goal_x, goal_y = local_goal
        goal_unit_x, goal_unit_y = goal_unit

        for cell, path_cost in distances.items():
            if cell == (0, 0) or cell in blocked:
                continue

            x, y = self.cell_to_xy(cell)
            distance_from_start = math.hypot(x, y)
            if distance_from_start < self.min_target_distance:
                continue

            progress = x * goal_unit_x + y * goal_unit_y
            lateral = abs(-x * goal_unit_y + y * goal_unit_x)
            distance_to_goal = math.hypot(x - goal_x, y - goal_y)
            clearance = self.clearance_at(cell, blocked, clearance_cache)
            score = (
                2.8 * progress
                - 0.7 * lateral
                - 0.25 * path_cost
                - 0.8 * distance_to_goal
                + 0.7 * min(clearance, self.clearance_radius)
            )
            if progress < -0.1:
                score -= 6.0
            if cell[0] >= self.max_ix - 1 or abs(cell[1]) >= self.max_iy - 1:
                score += 0.4

            side = self.side_for_point(x, y, goal_unit)
            heading = math.atan2(y, x)
            score = self.apply_memory_bias(score, side, heading)
            candidates.append({"cell": cell, "score": score, "side": side})

        selected = self.choose_with_hysteresis(candidates)
        if selected is None:
            return None, False
        return selected["cell"], selected.get("held_side", False)

    def fallback_step(self, blocked, goal_unit, step_distance):
        candidates = []
        goal_heading = math.atan2(goal_unit[1], goal_unit[0])
        max_angle = int(round(self.fallback_max_angle_deg))
        angle_step = max(1, int(round(self.fallback_angle_step_deg)))
        probe_distance = max(step_distance * 2.0, 0.5)

        for degree in range(-max_angle, max_angle + 1, angle_step):
            heading = goal_heading + math.radians(degree)
            target_x = math.cos(heading) * probe_distance
            target_y = math.sin(heading) * probe_distance
            target_cell = self.xy_to_cell(target_x, target_y)
            if not self.is_line_free(blocked, (0, 0), target_cell):
                continue

            progress = target_x * goal_unit[0] + target_y * goal_unit[1]
            clearance = self.clearance_at(target_cell, blocked, {})
            angle_ratio = abs(degree) / float(max(max_angle, 1))
            reverse_cost = 0.6 if abs(degree) > 90 else 0.0
            score = 1.2 * progress + 1.5 * clearance - 0.8 * angle_ratio - reverse_cost
            side = self.side_for_point(target_x, target_y, goal_unit)
            score = self.apply_memory_bias(score, side, heading)
            candidates.append(
                {
                    "point": (target_x, target_y),
                    "score": score,
                    "side": side,
                }
            )

        selected = self.choose_with_hysteresis(candidates)
        if selected is None:
            return self.emergency_escape_step(blocked, goal_unit, step_distance)

        target_x, target_y = selected["point"]
        limited_x, limited_y = self.limit_vector(target_x, target_y, step_distance)
        return limited_x, limited_y, selected.get("held_side", False)

    def emergency_escape_step(self, blocked, goal_unit, step_distance):
        candidates = []
        goal_heading = math.atan2(goal_unit[1], goal_unit[0])
        angle_step = max(1, int(round(self.fallback_angle_step_deg)))
        probe_distance = max(
            self.resolution,
            min(step_distance, self.emergency_escape_distance),
        )

        for degree in range(-180, 181, angle_step):
            heading = goal_heading + math.radians(degree)
            target_x = math.cos(heading) * probe_distance
            target_y = math.sin(heading) * probe_distance
            target_cell = self.xy_to_cell(target_x, target_y)
            if target_cell in blocked:
                continue
            if not self.is_line_free(blocked, (0, 0), target_cell):
                continue

            progress = target_x * goal_unit[0] + target_y * goal_unit[1]
            clearance = self.clearance_at(target_cell, blocked, {})
            angle_ratio = abs(degree) / 180.0
            score = 2.0 * clearance + 0.2 * progress - 0.2 * angle_ratio
            side = self.side_for_point(target_x, target_y, goal_unit)
            score = self.apply_memory_bias(score, side, heading)
            candidates.append(
                {
                    "point": (target_x, target_y),
                    "score": score,
                    "side": side,
                }
            )

        selected = self.choose_with_hysteresis(candidates)
        if selected is None:
            return 0.0, 0.0, False

        target_x, target_y = selected["point"]
        limited_x, limited_y = self.limit_vector(target_x, target_y, step_distance)
        return limited_x, limited_y, selected.get("held_side", False)

    def choose_with_hysteresis(self, candidates):
        if not candidates:
            return None

        best = max(candidates, key=lambda candidate: candidate["score"])
        if self.previous_avoidance_side == 0:
            best["held_side"] = False
            return best

        same_side_candidates = [
            candidate
            for candidate in candidates
            if candidate["side"] == self.previous_avoidance_side
        ]
        if not same_side_candidates or best["side"] == self.previous_avoidance_side:
            best["held_side"] = False
            return best

        same_side_best = max(same_side_candidates, key=lambda candidate: candidate["score"])
        if same_side_best["score"] >= best["score"] - self.side_hysteresis_margin:
            same_side_best["held_side"] = True
            return same_side_best

        best["held_side"] = False
        return best

    def apply_memory_bias(self, score, side, heading):
        if self.previous_avoidance_side != 0 and side != 0:
            if side == self.previous_avoidance_side:
                score += self.side_hold_bonus
            else:
                score -= self.side_switch_penalty

        if self.previous_selected_heading is not None:
            heading_change = abs(self.angle_diff(heading, self.previous_selected_heading))
            score += self.heading_hold_bonus * (1.0 - min(heading_change, math.pi) / math.pi)

        return score

    def build_local_status(self, goal_blocked, goal_reachable, target, goal_unit, held_side):
        if goal_blocked:
            status = "local_goal_blocked"
        elif not goal_reachable:
            status = "local_goal_unreachable"
        else:
            status = "local_astar"

        side = self.side_for_cell(target, goal_unit)
        if held_side:
            return self.append_hold_status(status, self.previous_avoidance_side)
        if side != 0:
            return "%s_%s" % (status, self.side_name(side))
        return status

    def append_hold_status(self, status, side):
        if side == 0:
            return status
        return "%s_hold_%s" % (status, self.side_name(side))

    def update_avoidance_memory(self, active, body_x, body_y, goal_unit, selected_heading):
        if not active or math.hypot(body_x, body_y) <= 0.01:
            self.previous_avoidance_side = 0
            self.previous_selected_heading = None
            return

        side = self.side_for_point(body_x, body_y, goal_unit)
        if side != 0:
            self.previous_avoidance_side = side
        self.previous_selected_heading = selected_heading

    def side_for_cell(self, cell, goal_unit):
        x, y = self.cell_to_xy(cell)
        return self.side_for_point(x, y, goal_unit)

    def side_for_point(self, x, y, goal_unit):
        goal_unit_x, goal_unit_y = goal_unit
        lateral = -x * goal_unit_y + y * goal_unit_x
        if lateral > self.side_deadband:
            return 1
        if lateral < -self.side_deadband:
            return -1
        return 0

    def side_name(self, side):
        if side > 0:
            return "left"
        if side < 0:
            return "right"
        return "center"

    def reconstruct_path(self, parents, target):
        path = [target]
        current = target
        while current != (0, 0):
            current = parents.get(current)
            if current is None:
                return []
            path.append(current)
        path.reverse()
        return [self.cell_to_xy(cell) for cell in path]

    def step_from_path(self, path, step_distance):
        if len(path) < 2:
            return 0.0, 0.0

        previous = path[0]
        traveled = 0.0
        target = path[-1]
        for point in path[1:]:
            segment = math.hypot(point[0] - previous[0], point[1] - previous[1])
            if traveled + segment >= self.path_lookahead:
                target = point
                break
            traveled += segment
            previous = point

        return self.limit_vector(target[0], target[1], step_distance)

    def direct_step(self, goal_x, goal_y, step_distance):
        return goal_x * step_distance, goal_y * step_distance

    def local_goal_point(self, goal_x, goal_y, goal_length):
        distance = min(max(goal_length, self.min_target_distance), self.local_goal_distance)
        return goal_x * distance, goal_y * distance

    def neighbors(self, cell):
        directions = [
            (-1, -1), (-1, 0), (-1, 1),
            (0, -1), (0, 1),
            (1, -1), (1, 0), (1, 1),
        ]
        for dx, dy in directions:
            neighbor = (cell[0] + dx, cell[1] + dy)
            if not self.in_bounds(neighbor):
                continue
            move_cost = math.hypot(dx, dy) * self.resolution
            yield neighbor, move_cost

    def clearance_at(self, cell, blocked, cache):
        if cell in cache:
            return cache[cell]
        if not blocked:
            cache[cell] = self.clearance_radius
            return cache[cell]

        cell_x, cell_y = self.cell_to_xy(cell)
        best = self.clearance_radius
        for blocked_cell in blocked:
            blocked_x, blocked_y = self.cell_to_xy(blocked_cell)
            distance = math.hypot(cell_x - blocked_x, cell_y - blocked_y)
            if distance < best:
                best = distance
                if best <= self.resolution:
                    break
        cache[cell] = best
        return best

    def is_line_free(self, blocked, start, end):
        for cell in self.line_cells(start, end):
            if not self.in_bounds(cell) or cell in blocked:
                return False
        return True

    def line_cells(self, start, end):
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        steps = max(abs(dx), abs(dy), 1)
        cells = []
        for index in range(steps + 1):
            ratio = float(index) / float(steps)
            ix = int(round(start[0] + dx * ratio))
            iy = int(round(start[1] + dy * ratio))
            cell = (ix, iy)
            if not cells or cells[-1] != cell:
                cells.append(cell)
        return cells

    def xy_to_cell(self, x, y):
        ix = int(round(x / self.resolution))
        iy = int(round(y / self.resolution))
        return (
            min(max(ix, self.min_ix), self.max_ix),
            min(max(iy, self.min_iy), self.max_iy),
        )

    def cell_to_xy(self, cell):
        return cell[0] * self.resolution, cell[1] * self.resolution

    def in_bounds(self, cell):
        return (
            self.min_ix <= cell[0] <= self.max_ix
            and self.min_iy <= cell[1] <= self.max_iy
        )

    def is_valid_distance(self, distance, scan):
        if math.isinf(distance) or math.isnan(distance):
            return False
        if distance < scan.range_min or distance > scan.range_max:
            return False
        return True

    def normalize_goal(self, vector):
        if vector is None:
            return 1.0, 0.0, self.local_goal_distance
        x, y = vector
        length = math.hypot(x, y)
        if length == 0.0:
            return 1.0, 0.0, self.local_goal_distance
        return x / length, y / length, length

    def limit_vector(self, x, y, limit):
        length = math.hypot(x, y)
        if length <= limit or length == 0.0:
            return x, y
        scale = limit / length
        return x * scale, y * scale

    def angle_diff(self, angle, reference):
        return (angle - reference + math.pi) % (2.0 * math.pi) - math.pi

    def rotate_body_to_map(self, body_x, body_y, yaw):
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        map_x = body_x * cos_yaw - body_y * sin_yaw
        map_y = body_x * sin_yaw + body_y * cos_yaw
        return map_x, map_y
