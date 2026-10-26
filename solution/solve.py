#!/usr/bin/env python3
import csv
import heapq
import itertools
import json
import math
import os
import statistics
import ezdxf
import trimesh
from shapely.affinity import translate
from shapely.geometry import LineString, Point, Polygon, box
from shapely.ops import unary_union

DATA, OUT = "/app/data", "/app/output"


def rows(name):
    with open(os.path.join(DATA, name), newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def register_probe_data():
    grouped = {}
    for r in rows("probe_runs.csv"):
        grouped.setdefault((r["setup"], r["pose"]), {})[r["feature"]] = (
            float(r["x_mm"]),
            float(r["y_mm"]),
        )
    registered, frame_spans = {}, []
    for key, f in grouped.items():
        a, d = f["DATUM_A"], f["DATUM_D"]
        dx, dy = d[0] - a[0], d[1] - a[1]
        span = math.hypot(dx, dy)
        ex, ey = (dx / span, dy / span), (-dy / span, dx / span)
        frame_spans.append(span)

        def local(p):
            px, py = p[0] - a[0], p[1] - a[1]
            return px * ex[0] + py * ex[1], px * ey[0] + py * ey[1]

        registered[key] = {"B": local(f["MOVING_B"]), "C": local(f["MOVING_C"])}
    corrected, swapped = {}, []
    for setup in sorted({k[0] for k in registered}):
        poses = sorted(k[1] for k in registered if k[0] == setup)
        best = (float("inf"), None)
        for bits in itertools.product((0, 1), repeat=len(poses)):
            tracks = []
            for pose, bit in zip(poses, bits):
                pair = registered[(setup, pose)]
                tracks.append((pair["C"], pair["B"]) if bit else (pair["B"], pair["C"]))
            cost = sum(
                math.dist(tracks[i - 1][0], tracks[i][0]) ** 2
                + math.dist(tracks[i - 1][1], tracks[i][1]) ** 2
                for i in range(1, len(tracks))
            )
            if cost < best[0]:
                best = cost, bits
        for pose, bit in zip(poses, best[1]):
            pair = registered[(setup, pose)]
            corrected[(setup, pose)] = (
                (pair["C"], pair["B"]) if bit else (pair["B"], pair["C"])
            )
            if bit:
                swapped.append(f"{setup}/{pose}")
    moving_spans = [math.dist(b, c) for b, c in corrected.values()]
    motion = []
    # The task defines the machine-frame datums as A=(0, 0), D=(92, 14).
    canonical_span = math.hypot(92.0, 14.0)
    canonical_ex, canonical_ey = (92.0 / canonical_span, 14.0 / canonical_span), (
        -14.0 / canonical_span,
        92.0 / canonical_span,
    )

    def machine(p):
        return (
            p[0] * canonical_ex[0] + p[1] * canonical_ey[0],
            p[0] * canonical_ex[1] + p[1] * canonical_ey[1],
        )

    for pose in sorted({k[1] for k in corrected}):
        samples = [corrected[k] for k in corrected if k[1] == pose]
        b = tuple(statistics.median(s[0][axis] for s in samples) for axis in range(2))
        c = tuple(statistics.median(s[1][axis] for s in samples) for axis in range(2))
        motion.append((machine(b), machine(c)))
    return (
        statistics.median(frame_spans),
        statistics.median(moving_spans),
        swapped,
        motion,
    )


def closest_point_on_segment(point, a, b):
    vx, vy = b[0] - a[0], b[1] - a[1]
    wx, wy = point[0] - a[0], point[1] - a[1]
    t = max(0.0, min(1.0, (wx * vx + wy * vy) / (vx * vx + vy * vy)))
    closest = (a[0] + t * vx, a[1] + t * vy)
    return closest, math.dist(point, closest)


def segment_distance(point, a, b):
    return closest_point_on_segment(point, a, b)[1]


def clearance_for(motion, keepouts, offset, width):
    minimum = float("inf")
    for i in range(len(motion) - 1):
        for step in range(9):
            t = step / 8
            b = tuple(
                motion[i][0][a] * (1 - t) + motion[i + 1][0][a] * t for a in range(2)
            )
            c = tuple(
                motion[i][1][a] * (1 - t) + motion[i + 1][1][a] * t for a in range(2)
            )
            dx, dy = c[0] - b[0], c[1] - b[1]
            length = math.hypot(dx, dy)
            elbow = (
                (b[0] + c[0]) / 2 - dy / length * offset,
                (b[1] + c[1]) / 2 + dx / length * offset,
            )
            for k in keepouts:
                exclusion = float(k["radius_mm"]) + float(k["clearance_mm"])
                centre = float(k["x_mm"]), float(k["y_mm"])
                distance = min(
                    segment_distance(centre, b, elbow),
                    segment_distance(centre, elbow, c),
                )
                minimum = min(minimum, distance - exclusion - width / 2)
    return minimum


def build_clearance_audit(motion, pose_names, keepouts, offset, width):
    audit = []
    for interval_index in range(len(motion) - 1):
        for sample_index in range(9):
            fraction = sample_index / 8
            b = tuple(
                motion[interval_index][0][axis] * (1 - fraction)
                + motion[interval_index + 1][0][axis] * fraction
                for axis in range(2)
            )
            c = tuple(
                motion[interval_index][1][axis] * (1 - fraction)
                + motion[interval_index + 1][1][axis] * fraction
                for axis in range(2)
            )
            dx, dy = c[0] - b[0], c[1] - b[1]
            length = math.hypot(dx, dy)
            elbow = (
                (b[0] + c[0]) / 2 - dy / length * offset,
                (b[1] + c[1]) / 2 + dx / length * offset,
            )

            for keepout in keepouts:
                centre = float(keepout["x_mm"]), float(keepout["y_mm"])
                first_point, first_distance = closest_point_on_segment(centre, b, elbow)
                second_point, second_distance = closest_point_on_segment(
                    centre, elbow, c
                )
                if first_distance <= second_distance:
                    controlling_segment = "B-elbow"
                    closest_point, centreline_distance = first_point, first_distance
                else:
                    controlling_segment = "elbow-C"
                    closest_point, centreline_distance = second_point, second_distance

                exclusion = float(keepout["radius_mm"]) + float(keepout["clearance_mm"])
                audit.append(
                    {
                        "interval_start_pose": pose_names[interval_index],
                        "interval_end_pose": pose_names[interval_index + 1],
                        "sample_index": sample_index,
                        "sample_fraction": fraction,
                        "keepout": keepout["name"],
                        "controlling_segment": controlling_segment,
                        "closest_x_mm": closest_point[0],
                        "closest_y_mm": closest_point[1],
                        "clearance_mm": centreline_distance - exclusion - width / 2,
                    }
                )
    return audit


def write_clearance_audit(audit):
    columns = [
        "interval_start_pose",
        "interval_end_pose",
        "sample_index",
        "sample_fraction",
        "keepout",
        "controlling_segment",
        "closest_x_mm",
        "closest_y_mm",
        "clearance_mm",
    ]
    path = os.path.join(OUT, "clearance_audit.csv")
    with open(path, "w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=columns)
        writer.writeheader()
        for observation in audit:
            writer.writerow(
                {
                    **observation,
                    "sample_fraction": f'{observation["sample_fraction"]:.3f}',
                    "closest_x_mm": f'{observation["closest_x_mm"]:.3f}',
                    "closest_y_mm": f'{observation["closest_y_mm"]:.3f}',
                    "clearance_mm": f'{observation["clearance_mm"]:.3f}',
                }
            )


def build_profile(
    span, offset, width, end_od, hole_d, slot_width, slot_fraction, facets
):
    quadrant_segments = facets // 4
    path = LineString([(0, 0), (span / 2, offset), (span, 0)])
    web = path.buffer(
        width / 2, quad_segs=quadrant_segments, cap_style="round", join_style="round"
    )
    bosses = [
        Point(0, 0).buffer(end_od / 2, quad_segs=quadrant_segments),
        Point(span, 0).buffer(end_od / 2, quad_segs=quadrant_segments),
    ]
    holes = unary_union(
        [
            Point(0, 0).buffer(hole_d / 2, quad_segs=quadrant_segments),
            Point(span, 0).buffer(hole_d / 2, quad_segs=quadrant_segments),
        ]
    )
    elbow = (span / 2, offset)
    slot_start = (elbow[0] * slot_fraction, elbow[1] * slot_fraction)
    slot_end = (
        elbow[0] + (span - elbow[0]) * (1 - slot_fraction),
        elbow[1] * slot_fraction,
    )
    lightening_slot = LineString([slot_start, elbow, slot_end]).buffer(
        slot_width / 2,
        quad_segs=quadrant_segments,
        cap_style="round",
        join_style="round",
    )
    cutouts = unary_union([holes, lightening_slot])
    return unary_union([web, *bosses]).difference(cutouts)


def extrude_profile(profile, thickness):
    mesh = trimesh.creation.extrude_polygon(profile, height=thickness, engine="earcut")
    mesh.process(validate=True)
    return mesh


def write_cut_profiles_dxf(left_profile, right_profile, stock):
    sheet_width, sheet_height = (
        float(value) for value in stock["available_sheet_mm"].lower().split("x")
    )
    edge_margin = 10.0
    requested_spacing = 8.0

    left_nested = translate(
        left_profile,
        xoff=edge_margin - left_profile.bounds[0],
        yoff=edge_margin - left_profile.bounds[1],
    )
    right_nested = translate(
        right_profile,
        xoff=edge_margin - right_profile.bounds[0],
        yoff=left_nested.bounds[3] + requested_spacing - right_profile.bounds[1],
    )
    combined = unary_union([left_nested, right_nested])
    min_x, min_y, max_x, max_y = combined.bounds
    measured_margin = min(
        min_x,
        min_y,
        sheet_width - max_x,
        sheet_height - max_y,
    )
    measured_spacing = left_nested.distance(right_nested)
    if measured_margin < edge_margin - 1e-6:
        raise ValueError("The selected stock cannot hold the required DXF nest")

    # Stable metadata keeps repeated manufacturing exports byte-for-byte equal.
    ezdxf.options.write_fixed_meta_data_for_testing = True
    document = ezdxf.new("R2010")
    document.units = 4  # millimetres
    document.layers.add("LEFT_LINK", color=3)
    document.layers.add("RIGHT_LINK", color=5)
    modelspace = document.modelspace()

    def add_profile(profile, layer):
        rings = [profile.exterior, *profile.interiors]
        for ring in rings:
            points = list(ring.coords)[:-1]
            modelspace.add_lwpolyline(
                points,
                format="xy",
                close=True,
                dxfattribs={"layer": layer},
            )

    add_profile(left_nested, "LEFT_LINK")
    add_profile(right_nested, "RIGHT_LINK")
    document.saveas(os.path.join(OUT, "cut_profiles.dxf"))
    return (
        measured_margin,
        measured_spacing,
        {
            "LEFT_LINK": left_nested,
            "RIGHT_LINK": right_nested,
        },
    )


def named_nominal_contours(profile):
    """Return the four manufacturing contours with stable shop names."""
    voids = [Polygon(ring) for ring in profile.interiors]
    slot = max(voids, key=lambda polygon: polygon.area)
    pin_holes = sorted(
        (polygon for polygon in voids if polygon is not slot),
        key=lambda polygon: polygon.centroid.x,
    )
    return {
        "PIN_B": pin_holes[0],
        "PIN_C": pin_holes[1],
        "SLOT": slot,
        "OUTER": Polygon(profile.exterior),
    }


def axis_extreme_vertices(polygon):
    """Find deterministic E/N/S/W choices from an already faceted contour."""
    points = [tuple(point[:2]) for point in list(polygon.exterior.coords)[:-1]]
    selected = {
        "E": min(points, key=lambda point: (-point[0], point[1])),
        "N": min(points, key=lambda point: (-point[1], point[0])),
        "S": min(points, key=lambda point: (point[1], point[0])),
        "W": min(points, key=lambda point: (point[0], point[1])),
    }
    return [(side, point) for side, point in selected.items()]


def build_cam_items(nested_profiles, stock, process):
    """Build compensated contours and every scrap-side lead-in candidate."""
    kerf = float(process["kerf_mm"])
    lead_length = float(process["lead_in_mm"])
    sheet_width, sheet_height = (
        float(value) for value in stock["available_sheet_mm"].lower().split("x")
    )
    sheet = Polygon(
        [(0, 0), (sheet_width, 0), (sheet_width, sheet_height), (0, sheet_height)]
    )
    material = unary_union(list(nested_profiles.values()))
    items = []

    for part in ("LEFT_LINK", "RIGHT_LINK"):
        profile = nested_profiles[part]
        for contour, nominal in named_nominal_contours(profile).items():
            outer = contour == "OUTER"
            toolpath = nominal.buffer(
                kerf / 2 if outer else -kerf / 2,
                quad_segs=24,
                join_style="round",
            )
            if toolpath.geom_type != "Polygon" or toolpath.is_empty:
                raise ValueError(f"Kerf compensation collapsed {part} {contour}")

            candidates = []
            centre = (toolpath.centroid.x, toolpath.centroid.y)
            for side, cut_start in axis_extreme_vertices(toolpath):
                dx, dy = cut_start[0] - centre[0], cut_start[1] - centre[1]
                length = math.hypot(dx, dy)
                direction = (dx / length, dy / length)
                sign = 1 if outer else -1
                lead_start = (
                    cut_start[0] + sign * lead_length * direction[0],
                    cut_start[1] + sign * lead_length * direction[1],
                )
                lead = LineString([lead_start, cut_start])
                in_scrap = (
                    lead.disjoint(material)
                    if outer
                    else nominal.buffer(1e-7).covers(lead)
                )
                if sheet.buffer(1e-7).covers(lead) and in_scrap:
                    candidates.append(
                        {
                            "side": side,
                            "cut_start": cut_start,
                            "lead_start": lead_start,
                        }
                    )
            if not candidates:
                raise ValueError(f"No valid lead-in for {part} {contour}")
            items.append(
                {
                    "part": part,
                    "contour": contour,
                    "outer": outer,
                    "toolpath": toolpath,
                    "nominal": nominal,
                    "candidates": candidates,
                }
            )
    return items


def expanded_rapid_obstacles(keepout_rows):
    """Return fixture rectangles expanded by the programmed head clearance."""
    obstacles = []
    for row in keepout_rows:
        allowance = float(row["head_clearance_mm"])
        obstacles.append(
            box(
                float(row["min_x_mm"]) - allowance,
                float(row["min_y_mm"]) - allowance,
                float(row["max_x_mm"]) + allowance,
                float(row["max_y_mm"]) + allowance,
            )
        )
    return obstacles


def segment_clears_obstacles(start, end, obstacles):
    """Permit boundary contact, but never let a rapid enter a keep-out interior."""
    segment = LineString([start, end])
    return all(
        segment.intersection(obstacle.buffer(-1e-7)).is_empty for obstacle in obstacles
    )


def shortest_rapid_path(start, end, obstacles):
    """Find a shortest legal polyline on the rectangle-corner visibility graph."""
    corner_nodes = {
        (float(x), float(y))
        for obstacle in obstacles
        for x, y in list(obstacle.exterior.coords)[:-1]
    }
    nodes = [tuple(start), tuple(end), *sorted(corner_nodes)]
    adjacency = [[] for _ in nodes]
    for first in range(len(nodes)):
        for second in range(first + 1, len(nodes)):
            if segment_clears_obstacles(nodes[first], nodes[second], obstacles):
                distance = math.dist(nodes[first], nodes[second])
                adjacency[first].append((second, distance))
                adjacency[second].append((first, distance))

    # The full coordinate path is the secondary key, which makes ties stable.
    queue = [(0.0, (nodes[0],), 0)]
    best = {0: (0.0, (nodes[0],))}
    while queue:
        cost, path, node_index = heapq.heappop(queue)
        if (cost, path) != best.get(node_index):
            continue
        if node_index == 1:
            return cost, list(path[1:-1])
        for neighbour, edge_length in adjacency[node_index]:
            proposal = (cost + edge_length, path + (nodes[neighbour],))
            if neighbour not in best or proposal < best[neighbour]:
                best[neighbour] = proposal
                heapq.heappush(queue, (*proposal, neighbour))
    raise ValueError(f"No rapid path from {start} to {end}")


def minimum_rapid_route(items, rapid_start, obstacles):
    """Solve start choice and cut order with obstacle-aware subset dynamic programming."""
    predecessors = {}
    for index, item in enumerate(items):
        predecessors[index] = {
            other_index
            for other_index, other in enumerate(items)
            if item["outer"] and other["part"] == item["part"] and not other["outer"]
        }

    # State values carry both the cost and path.  The path label is a stable
    # secondary key when symmetric geometry produces equal-length routes.
    path_cache = {}

    def travel(start, end):
        key = tuple(start), tuple(end)
        if key not in path_cache:
            path_cache[key] = shortest_rapid_path(start, end, obstacles)[0]
        return path_cache[key]

    states = {(0, -1, -1): (0.0, tuple(), tuple())}
    full_mask = (1 << len(items)) - 1
    for mask in range(full_mask + 1):
        current_states = [
            (key, value) for key, value in states.items() if key[0] == mask
        ]
        for (_, last_item, last_candidate), (cost, path, labels) in current_states:
            previous = (
                rapid_start
                if last_item < 0
                else items[last_item]["candidates"][last_candidate]["cut_start"]
            )
            for item_index, item in enumerate(items):
                if mask & (1 << item_index) or any(
                    not mask & (1 << required) for required in predecessors[item_index]
                ):
                    continue
                for candidate_index, candidate in enumerate(item["candidates"]):
                    new_cost = cost + travel(previous, candidate["lead_start"])
                    new_path = path + ((item_index, candidate_index),)
                    new_labels = labels + (
                        (item["part"], item["contour"], candidate["side"]),
                    )
                    state_key = (
                        mask | (1 << item_index),
                        item_index,
                        candidate_index,
                    )
                    proposal = (new_cost, new_path, new_labels)
                    incumbent = states.get(state_key)
                    if incumbent is None or (new_cost, new_labels) < (
                        incumbent[0],
                        incumbent[2],
                    ):
                        states[state_key] = proposal

    best = min(
        (value for key, value in states.items() if key[0] == full_mask),
        key=lambda value: (value[0], value[2]),
    )
    return best[0], best[1]


def write_machine_toolpaths(nested_profiles, stock, process, rapid_keepouts):
    items = build_cam_items(nested_profiles, stock, process)
    obstacles = expanded_rapid_obstacles(rapid_keepouts)
    rapid_start = (
        float(process["rapid_start_x_mm"]),
        float(process["rapid_start_y_mm"]),
    )
    total_rapid, route = minimum_rapid_route(items, rapid_start, obstacles)

    ezdxf.options.write_fixed_meta_data_for_testing = True
    document = ezdxf.new("R2010")
    document.units = 4
    document.layers.add("TOOL_INNER", color=1)
    document.layers.add("TOOL_OUTER", color=2)
    document.layers.add("LEAD_IN", color=6)
    modelspace = document.modelspace()
    records = []

    previous = rapid_start
    for sequence, (item_index, candidate_index) in enumerate(route, start=1):
        item = items[item_index]
        candidate = item["candidates"][candidate_index]
        cut_start, lead_start = candidate["cut_start"], candidate["lead_start"]
        modelspace.add_lwpolyline(
            [lead_start, cut_start],
            format="xy",
            dxfattribs={"layer": "LEAD_IN"},
        )

        points = [tuple(point[:2]) for point in item["toolpath"].exterior.coords[:-1]]
        start_index = min(
            range(len(points)), key=lambda index: math.dist(points[index], cut_start)
        )
        ordered_points = points[start_index:] + points[:start_index]
        modelspace.add_lwpolyline(
            ordered_points,
            format="xy",
            close=True,
            dxfattribs={"layer": "TOOL_OUTER" if item["outer"] else "TOOL_INNER"},
        )
        _, rapid_waypoints = shortest_rapid_path(previous, lead_start, obstacles)
        records.append(
            {
                "sequence": sequence,
                "part": item["part"],
                "contour": item["contour"],
                "side": candidate["side"],
                "rapid_waypoints_mm": [
                    [round(value, 3) for value in waypoint]
                    for waypoint in rapid_waypoints
                ],
                "lead_start_mm": [round(value, 3) for value in lead_start],
                "cut_start_mm": [round(value, 3) for value in cut_start],
            }
        )
        previous = cut_start

    document.saveas(os.path.join(OUT, "machine_toolpaths.dxf"))
    report = {
        "schema_version": 1,
        "units": "mm",
        "machine_id": process["machine_id"],
        "kerf_mm": float(process["kerf_mm"]),
        "lead_in_mm": float(process["lead_in_mm"]),
        "rapid_start_mm": [round(value, 3) for value in rapid_start],
        "total_rapid_mm": round(total_rapid, 3),
        "cut_sequence": records,
    }
    with open(os.path.join(OUT, "cam_report.json"), "w", encoding="utf-8") as handle:
        json.dump(report, handle, indent=2, sort_keys=True)
        handle.write("\n")


def build_sweep_envelope(motion, offset, width, end_od, thickness, facets):
    quadrant_segments = facets // 4
    silhouettes = []
    for pose_index in range(len(motion) - 1):
        for step in range(9):
            fraction = step / 8
            b = tuple(
                motion[pose_index][0][axis] * (1 - fraction)
                + motion[pose_index + 1][0][axis] * fraction
                for axis in range(2)
            )
            c = tuple(
                motion[pose_index][1][axis] * (1 - fraction)
                + motion[pose_index + 1][1][axis] * fraction
                for axis in range(2)
            )
            dx, dy = c[0] - b[0], c[1] - b[1]
            length = math.hypot(dx, dy)
            elbow = (
                (b[0] + c[0]) / 2 - dy / length * offset,
                (b[1] + c[1]) / 2 + dx / length * offset,
            )
            web = LineString([b, elbow, c]).buffer(
                width / 2,
                quad_segs=quadrant_segments,
                cap_style="round",
                join_style="round",
            )
            bosses = (
                Point(*b).buffer(end_od / 2, quad_segs=quadrant_segments),
                Point(*c).buffer(end_od / 2, quad_segs=quadrant_segments),
            )
            silhouettes.append(unary_union([web, *bosses]))

    # Repeated unions can leave sub-micron boundary notches. Removing them is
    # below fabrication resolution and avoids non-manifold triangulation.
    envelope = unary_union(silhouettes).simplify(0.001, preserve_topology=True)
    mesh = trimesh.creation.extrude_polygon(envelope, height=thickness, engine="earcut")
    return mesh, envelope.area


def main():
    os.makedirs(OUT, exist_ok=True)
    frame_span, moving_span, swapped, motion = register_probe_data()
    pose_names = sorted({row["pose"] for row in rows("probe_runs.csv")})
    keepouts, loads, stocks, process, rapid_keepouts = (
        rows("keepouts.csv"),
        rows("load_cases.csv"),
        rows("stock.csv"),
        rows("waterjet.csv")[0],
        rows("rapid_keepouts.csv"),
    )
    width, end_od, hole_d, facets = 18.0, 22.0, 10.4, 96
    slot_width, slot_fraction = 7.0, 0.30
    offset = next(
        v / 10
        for v in range(401)
        if clearance_for(motion, keepouts, v / 10, width) >= 2.5
    )
    audit = build_clearance_audit(motion, pose_names, keepouts, offset, width)
    write_clearance_audit(audit)
    critical = min(audit, key=lambda observation: observation["clearance_mm"])
    clearance = critical["clearance_mm"]
    peak = max(float(x["pin_load_n"]) for x in loads)
    candidates = []
    for stock in stocks:
        thickness = float(stock["thickness_mm"])
        safety = float(stock["yield_mpa"]) / (
            peak / (2 * thickness * (end_od - hole_d))
        )
        if safety >= 2:
            candidates.append((float(stock["density_g_cm3"]), stock, safety))
    density, stock, safety = min(candidates, key=lambda x: x[0])
    thickness = float(stock["thickness_mm"])
    left, right = os.path.join(OUT, "left_link.stl"), os.path.join(
        OUT, "right_link.stl"
    )
    left_profile = build_profile(
        moving_span,
        offset,
        width,
        end_od,
        hole_d,
        slot_width,
        slot_fraction,
        facets,
    )
    right_profile = build_profile(
        moving_span,
        -offset,
        width,
        end_od,
        hole_d,
        slot_width,
        slot_fraction,
        facets,
    )
    left_mesh = extrude_profile(left_profile, thickness)
    right_mesh = extrude_profile(right_profile, thickness)
    left_mesh.export(left)
    right_mesh.export(right)
    dxf_margin, dxf_spacing, nested_profiles = write_cut_profiles_dxf(
        left_profile, right_profile, stock
    )
    write_machine_toolpaths(nested_profiles, stock, process, rapid_keepouts)
    sweep_mesh, sweep_area = build_sweep_envelope(
        motion, offset, width, end_od, thickness, facets
    )
    sweep_mesh.export(os.path.join(OUT, "left_sweep_envelope.stl"))
    mass = (left_mesh.volume + right_mesh.volume) * density / 1000
    report = {
        "schema_version": 1,
        "units": "mm",
        "reconstruction": {
            "frame_pivot_span_mm": round(frame_span, 3),
            "moving_link_span_mm": round(moving_span, 3),
            "corrected_feature_swap": swapped[0],
        },
        "link": {
            "material_stock_id": stock["stock_id"],
            "pin_span_mm": round(moving_span, 3),
            "dogleg_offset_mm": round(offset, 3),
            "body_width_mm": width,
            "end_outer_diameter_mm": end_od,
            "pin_hole_diameter_mm": hole_d,
            "lightening_slot_width_mm": slot_width,
            "lightening_slot_end_fraction": slot_fraction,
            "thickness_mm": thickness,
            "facet_count_per_circle": facets,
        },
        "engineering_checks": {
            "minimum_swept_clearance_mm": round(clearance, 3),
            "jam_load_safety_factor": round(safety, 3),
            "pair_mass_g": round(mass, 2),
            "left_sweep_envelope_area_mm2": round(sweep_area, 3),
            "dxf_min_edge_margin_mm": round(dxf_margin, 3),
            "dxf_part_spacing_mm": round(dxf_spacing, 3),
            "clearance_audit_rows": len(audit),
            "critical_clearance_witness": {
                "interval": (
                    f'{critical["interval_start_pose"]}-'
                    f'{critical["interval_end_pose"]}'
                ),
                "sample_index": critical["sample_index"],
                "keepout": critical["keepout"],
                "controlling_segment": critical["controlling_segment"],
                "closest_point_mm": [
                    round(critical["closest_x_mm"], 3),
                    round(critical["closest_y_mm"], 3),
                ],
            },
        },
        "artifacts": [
            "left_link.stl",
            "right_link.stl",
            "left_sweep_envelope.stl",
            "clearance_audit.csv",
            "cut_profiles.dxf",
        ],
    }
    with open(os.path.join(OUT, "design.json"), "w", encoding="utf-8") as f:
        json.dump(report, f, indent=2, sort_keys=True)
        f.write("\n")


if __name__ == "__main__":
    main()
