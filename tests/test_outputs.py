import csv
import heapq
import itertools
import json
import math
import os
import re
import statistics
import ezdxf
import numpy as np
import trimesh
from shapely.affinity import translate
from shapely.geometry import LineString, Point, Polygon, box
from shapely.ops import unary_union

ROOT = os.environ.get("TOGGLE_ARTIFACT_ROOT", "/app/output")
DATA = os.environ.get("TOGGLE_TEST_DATA", "/tests/data")

# Keep comparison bands in one place so a reviewer can distinguish reporting
# precision from an engineering requirement.  These bands were exercised with
# 64-, 96-, and 128-facet exports and with both rounded and mitered web joins.
SPAN_TOLERANCE_MM = 0.01
STRENGTH_REPORT_TOLERANCE = 0.01
CLEARANCE_TOLERANCE_MM = 0.02
AUDIT_FRACTION_TOLERANCE = 0.0005
AUDIT_COORDINATE_TOLERANCE_MM = 0.001
MESH_Z_TOLERANCE_MM = 0.01
THICKNESS_TOLERANCE_MM = 0.05
LINK_BOUNDS_TOLERANCE_MM = 0.12
MIRROR_X_TOLERANCE_MM = 0.08
MASS_TOLERANCE_G = 0.08
SECTION_SHAPE_RELATIVE_TOLERANCE = 0.01
SECTION_AREA_ABSOLUTE_TOLERANCE_MM2 = 0.5
DXF_METRIC_TOLERANCE_MM = 0.02
ENVELOPE_BOUNDS_TOLERANCE_MM = 0.20
ENVELOPE_AREA_RELATIVE_TOLERANCE = 0.01
ENVELOPE_AREA_ABSOLUTE_TOLERANCE_MM2 = 1.0
CAM_SHAPE_RELATIVE_TOLERANCE = 0.005
CAM_SHAPE_ABSOLUTE_TOLERANCE_MM2 = 0.1
CAM_POINT_TOLERANCE_MM = 0.02
CAM_ROUTE_TOLERANCE_MM = 0.02


def rows(name):
    with open(os.path.join(DATA, name), newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def ground_truth():
    grouped = {}
    for r in rows("probe_runs.csv"):
        grouped.setdefault((r["setup"], r["pose"]), {})[r["feature"]] = (
            float(r["x_mm"]),
            float(r["y_mm"]),
        )
    registered = {}
    frames = []
    for key, f in grouped.items():
        a, d = f["DATUM_A"], f["DATUM_D"]
        dx, dy = d[0] - a[0], d[1] - a[1]
        span = math.hypot(dx, dy)
        frames.append(span)
        ex, ey = (dx / span, dy / span), (-dy / span, dx / span)

        def local(p):
            q = (p[0] - a[0], p[1] - a[1])
            return q[0] * ex[0] + q[1] * ex[1], q[0] * ey[0] + q[1] * ey[1]

        registered[key] = {"B": local(f["MOVING_B"]), "C": local(f["MOVING_C"])}
    corrected = {}
    swaps = []
    for setup in sorted({k[0] for k in registered}):
        poses = sorted(k[1] for k in registered if k[0] == setup)
        best = (float("inf"), None)
        for bits in itertools.product((0, 1), repeat=len(poses)):
            tracks = []
            for pose, bit in zip(poses, bits):
                p = registered[(setup, pose)]
                tracks.append((p["C"], p["B"]) if bit else (p["B"], p["C"]))
            cost = sum(
                math.dist(tracks[i - 1][0], tracks[i][0]) ** 2
                + math.dist(tracks[i - 1][1], tracks[i][1]) ** 2
                for i in range(1, len(tracks))
            )
            if cost < best[0]:
                best = cost, bits
        for pose, bit in zip(poses, best[1]):
            p = registered[(setup, pose)]
            corrected[(setup, pose)] = (p["C"], p["B"]) if bit else (p["B"], p["C"])
            if bit:
                swaps.append(f"{setup}/{pose}")
    motion = []
    # Independently reconstruct the machine frame stated in instruction.md.
    canonical_span = math.hypot(92, 14)
    canonical_ex = (92 / canonical_span, 14 / canonical_span)
    canonical_ey = (-14 / canonical_span, 92 / canonical_span)

    def machine(p):
        return (
            p[0] * canonical_ex[0] + p[1] * canonical_ey[0],
            p[0] * canonical_ex[1] + p[1] * canonical_ey[1],
        )

    for pose in sorted({k[1] for k in corrected}):
        samples = [corrected[k] for k in corrected if k[1] == pose]
        motion.append(
            (
                machine(
                    tuple(statistics.median(s[0][a] for s in samples) for a in range(2))
                ),
                machine(
                    tuple(statistics.median(s[1][a] for s in samples) for a in range(2))
                ),
            )
        )
    return (
        statistics.median(frames),
        statistics.median(math.dist(b, c) for b, c in corrected.values()),
        swaps,
        motion,
    )


def closest_point_on_segment(point, a, b):
    v = (b[0] - a[0], b[1] - a[1])
    w = (point[0] - a[0], point[1] - a[1])
    t = max(0, min(1, (w[0] * v[0] + w[1] * v[1]) / (v[0] ** 2 + v[1] ** 2)))
    closest = (a[0] + t * v[0], a[1] + t * v[1])
    return closest, math.dist(point, closest)


def segment_distance(point, a, b):
    return closest_point_on_segment(point, a, b)[1]


def swept_clearance(motion, offset, width):
    keepouts = rows("keepouts.csv")
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
                centre = (float(k["x_mm"]), float(k["y_mm"]))
                exclusion = float(k["radius_mm"]) + float(k["clearance_mm"])
                minimum = min(
                    minimum,
                    min(
                        segment_distance(centre, b, elbow),
                        segment_distance(centre, elbow, c),
                    )
                    - exclusion
                    - width / 2,
                )
    return minimum


AUDIT_COLUMNS = [
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


def expected_clearance_audit(motion, offset, width):
    pose_names = sorted({row["pose"] for row in rows("probe_runs.csv")})
    keepouts = rows("keepouts.csv")
    expected = []
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
                    segment, closest, distance = (
                        "B-elbow",
                        first_point,
                        first_distance,
                    )
                else:
                    segment, closest, distance = (
                        "elbow-C",
                        second_point,
                        second_distance,
                    )
                exclusion = float(keepout["radius_mm"]) + float(keepout["clearance_mm"])
                expected.append(
                    {
                        "interval_start_pose": pose_names[interval_index],
                        "interval_end_pose": pose_names[interval_index + 1],
                        "sample_index": sample_index,
                        "sample_fraction": fraction,
                        "keepout": keepout["name"],
                        "controlling_segment": segment,
                        "closest_x_mm": closest[0],
                        "closest_y_mm": closest[1],
                        "clearance_mm": distance - exclusion - width / 2,
                    }
                )
    return expected


def ray_hits(mesh, xy):
    origin = np.array([[xy[0], xy[1], float(mesh.bounds[1, 2]) + 1]])
    locations, _, _ = mesh.ray.intersects_location(
        origin, np.array([[0, 0, -1.0]]), multiple_hits=True
    )
    return len(locations)


def test_report_is_derived_from_sealed_inputs():
    with open(os.path.join(ROOT, "design.json"), encoding="utf-8") as f:
        report = json.load(f)
    assert set(report) == {
        "schema_version",
        "units",
        "reconstruction",
        "link",
        "engineering_checks",
        "artifacts",
    }
    assert (
        report["schema_version"] == 1
        and report["units"] == "mm"
        and report["artifacts"]
        == [
            "left_link.stl",
            "right_link.stl",
            "left_sweep_envelope.stl",
            "clearance_audit.csv",
            "cut_profiles.dxf",
        ]
    )
    frame, span, swaps, motion = ground_truth()
    recon, link, checks = (
        report["reconstruction"],
        report["link"],
        report["engineering_checks"],
    )
    assert (
        abs(recon["frame_pivot_span_mm"] - frame) <= SPAN_TOLERANCE_MM
        and abs(recon["moving_link_span_mm"] - span) <= SPAN_TOLERANCE_MM
    )
    assert (
        recon["corrected_feature_swap"] == swaps[0]
        and abs(link["pin_span_mm"] - span) <= SPAN_TOLERANCE_MM
    )
    assert (
        link["pin_hole_diameter_mm"] == 10.4
        and link["body_width_mm"] >= 18
        and link["end_outer_diameter_mm"] >= 22
        and 6 <= link["lightening_slot_width_mm"] <= 8
        and 0.25 <= link["lightening_slot_end_fraction"] <= 0.35
        and link["facet_count_per_circle"] >= 64
    )
    stock = next(
        s for s in rows("stock.csv") if s["stock_id"] == link["material_stock_id"]
    )
    assert abs(link["thickness_mm"] - float(stock["thickness_mm"])) < 0.001
    peak = max(float(x["pin_load_n"]) for x in rows("load_cases.csv"))
    expected_sf = float(stock["yield_mpa"]) / (
        peak
        / (
            2
            * link["thickness_mm"]
            * (link["end_outer_diameter_mm"] - link["pin_hole_diameter_mm"])
        )
    )
    assert (
        expected_sf >= 2
        and abs(checks["jam_load_safety_factor"] - expected_sf)
        <= STRENGTH_REPORT_TOLERANCE
    )
    expected_clearance = swept_clearance(
        motion, link["dogleg_offset_mm"], link["body_width_mm"]
    )
    assert (
        expected_clearance >= 2
        and abs(checks["minimum_swept_clearance_mm"] - expected_clearance)
        <= CLEARANCE_TOLERANCE_MM
    )


def test_clearance_audit_covers_every_sample_and_keepout():
    with open(os.path.join(ROOT, "design.json"), encoding="utf-8") as handle:
        report = json.load(handle)
    link = report["link"]
    checks = report["engineering_checks"]
    _, _, _, motion = ground_truth()
    expected_rows = expected_clearance_audit(
        motion, link["dogleg_offset_mm"], link["body_width_mm"]
    )

    audit_path = os.path.join(ROOT, "clearance_audit.csv")
    assert os.path.isfile(audit_path) and os.path.getsize(audit_path) < 1_000_000
    with open(audit_path, newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        assert reader.fieldnames == AUDIT_COLUMNS
        submitted_rows = list(reader)

    def key(row):
        return (
            row["interval_start_pose"],
            row["interval_end_pose"],
            int(row["sample_index"]),
            row["keepout"],
        )

    expected_count = (len(motion) - 1) * 9 * len(rows("keepouts.csv"))
    assert len(submitted_rows) == len(expected_rows) == expected_count
    submitted = {key(row): row for row in submitted_rows}
    expected = {key(row): row for row in expected_rows}
    assert len(submitted) == expected_count and submitted.keys() == expected.keys()

    three_decimal_number = re.compile(r"-?\d+\.\d{3}")
    for row_key, expected_row in expected.items():
        actual = submitted[row_key]
        for field in (
            "sample_fraction",
            "closest_x_mm",
            "closest_y_mm",
            "clearance_mm",
        ):
            assert three_decimal_number.fullmatch(actual[field])
        assert actual["controlling_segment"] == expected_row["controlling_segment"]
        assert (
            abs(float(actual["sample_fraction"]) - expected_row["sample_fraction"])
            <= AUDIT_FRACTION_TOLERANCE
        )
        assert (
            abs(float(actual["closest_x_mm"]) - expected_row["closest_x_mm"])
            <= AUDIT_COORDINATE_TOLERANCE_MM
        )
        assert (
            abs(float(actual["closest_y_mm"]) - expected_row["closest_y_mm"])
            <= AUDIT_COORDINATE_TOLERANCE_MM
        )
        assert (
            abs(float(actual["clearance_mm"]) - expected_row["clearance_mm"])
            <= AUDIT_COORDINATE_TOLERANCE_MM
        )

    minimum = min(expected_rows, key=lambda row: row["clearance_mm"])
    witness = checks["critical_clearance_witness"]
    assert checks["clearance_audit_rows"] == expected_count
    assert witness["interval"] == (
        f'{minimum["interval_start_pose"]}-{minimum["interval_end_pose"]}'
    )
    assert witness["sample_index"] == minimum["sample_index"]
    assert witness["keepout"] == minimum["keepout"]
    assert witness["controlling_segment"] == minimum["controlling_segment"]
    assert np.allclose(
        witness["closest_point_mm"],
        [minimum["closest_x_mm"], minimum["closest_y_mm"]],
        atol=AUDIT_COORDINATE_TOLERANCE_MM,
    )
    assert (
        abs(checks["minimum_swept_clearance_mm"] - minimum["clearance_mm"])
        <= CLEARANCE_TOLERANCE_MM
    )


def test_meshes_are_manufacturable_and_match_report():
    with open(os.path.join(ROOT, "design.json"), encoding="utf-8") as f:
        report = json.load(f)
    link = report["link"]
    stock = next(
        s for s in rows("stock.csv") if s["stock_id"] == link["material_stock_id"]
    )
    meshes = []
    for mesh_index, name in enumerate(("left_link.stl", "right_link.stl")):
        path = os.path.join(ROOT, name)
        assert os.path.isfile(path) and os.path.getsize(path) < 20_000_000
        mesh = trimesh.load(path, force="mesh", process=True)
        assert (
            mesh.is_watertight
            and mesh.is_winding_consistent
            and mesh.volume > 0
            and len(mesh.split()) == 1
        )
        assert abs(mesh.bounds[0, 2]) <= MESH_Z_TOLERANCE_MM
        assert abs(mesh.extents[2] - link["thickness_mm"]) <= THICKNESS_TOLERANCE_MM
        sheet_width, sheet_height = (
            float(value) for value in stock["available_sheet_mm"].lower().split("x")
        )
        profile_width, profile_height = mesh.extents[:2]
        assert (profile_width <= sheet_width and profile_height <= sheet_height) or (
            profile_width <= sheet_height and profile_height <= sheet_width
        )
        span = link["pin_span_mm"]
        hole_r = link["pin_hole_diameter_mm"] / 2
        boss_r = link["end_outer_diameter_mm"] / 2
        assert (
            abs(mesh.bounds[0, 0] + boss_r) <= LINK_BOUNDS_TOLERANCE_MM
            and abs(mesh.bounds[1, 0] - (span + boss_r)) <= LINK_BOUNDS_TOLERANCE_MM
        )
        for centre in ((0, 0), (span, 0)):
            assert ray_hits(mesh, centre) == 0
            assert ray_hits(mesh, (centre[0] + hole_r + 0.6, centre[1])) > 0
            radii = np.linalg.norm(mesh.vertices[:, :2] - np.array(centre), axis=1)
            assert (
                len(
                    np.unique(
                        np.round(
                            np.arctan2(
                                mesh.vertices[np.abs(radii - hole_r) < 0.08, 1]
                                - centre[1],
                                mesh.vertices[np.abs(radii - hole_r) < 0.08, 0]
                                - centre[0],
                            ),
                            5,
                        )
                    )
                )
                >= 64
            )
        elbow_y = link["dogleg_offset_mm"] * (1 if mesh_index == 0 else -1)
        elbow = np.array((span / 2, elbow_y))
        slot_fraction = link["lightening_slot_end_fraction"]
        slot_start = elbow * slot_fraction
        slot_end = elbow + (np.array((span, 0.0)) - elbow) * (1 - slot_fraction)
        slot_width = link["lightening_slot_width_mm"]
        for point in (
            slot_start,
            (slot_start + elbow) / 2,
            elbow,
            (elbow + slot_end) / 2,
            slot_end,
        ):
            assert ray_hits(mesh, point) == 0
        for a, b in ((slot_start, elbow), (elbow, slot_end)):
            midpoint = (a + b) / 2
            direction = b - a
            normal = np.array((-direction[1], direction[0])) / np.linalg.norm(direction)
            wall_probe = slot_width / 2 + 1.0
            assert ray_hits(mesh, midpoint + normal * wall_probe) > 0
            assert ray_hits(mesh, midpoint - normal * wall_probe) > 0
        assert ray_hits(mesh, (span / 2, elbow_y + link["body_width_mm"] / 2 - 0.5)) > 0
        assert ray_hits(mesh, (span / 2, elbow_y - link["body_width_mm"] / 2 + 0.5)) > 0
        meshes.append(mesh)
    assert np.allclose(
        meshes[0].bounds[:, 0], meshes[1].bounds[:, 0], atol=MIRROR_X_TOLERANCE_MM
    )
    assert (
        abs(meshes[0].bounds[1, 1] + meshes[1].bounds[0, 1]) <= LINK_BOUNDS_TOLERANCE_MM
        and abs(meshes[0].bounds[0, 1] + meshes[1].bounds[1, 1])
        <= LINK_BOUNDS_TOLERANCE_MM
    )
    mass = sum(m.volume for m in meshes) * float(stock["density_g_cm3"]) / 1000
    assert (
        mass <= 55
        and abs(report["engineering_checks"]["pair_mass_g"] - mass) <= MASS_TOLERANCE_G
    )


def expected_link_profile(link, offset):
    span = link["pin_span_mm"]
    width = link["body_width_mm"]
    end_radius = link["end_outer_diameter_mm"] / 2
    hole_radius = link["pin_hole_diameter_mm"] / 2
    quadrant_segments = link["facet_count_per_circle"] // 4
    elbow = (span / 2, offset)
    web = LineString([(0, 0), elbow, (span, 0)]).buffer(
        width / 2,
        quad_segs=quadrant_segments,
        cap_style="round",
        join_style="round",
    )
    bosses = unary_union(
        [
            Point(0, 0).buffer(end_radius, quad_segs=quadrant_segments),
            Point(span, 0).buffer(end_radius, quad_segs=quadrant_segments),
        ]
    )
    holes = unary_union(
        [
            Point(0, 0).buffer(hole_radius, quad_segs=quadrant_segments),
            Point(span, 0).buffer(hole_radius, quad_segs=quadrant_segments),
        ]
    )
    fraction = link["lightening_slot_end_fraction"]
    slot_start = (elbow[0] * fraction, elbow[1] * fraction)
    slot_end = (
        elbow[0] + (span - elbow[0]) * (1 - fraction),
        elbow[1] * fraction,
    )
    slot = LineString([slot_start, elbow, slot_end]).buffer(
        link["lightening_slot_width_mm"] / 2,
        quad_segs=quadrant_segments,
        cap_style="round",
        join_style="round",
    )
    return unary_union([web, bosses]).difference(unary_union([holes, slot]))


def profile_from_dxf_layer(document, layer):
    entities = [
        entity
        for entity in document.modelspace()
        if entity.dxftype() == "LWPOLYLINE" and entity.dxf.layer == layer
    ]
    assert len(entities) == 4
    contours = []
    for entity in entities:
        assert entity.closed
        points = list(entity.get_points("xyb"))
        assert len(points) >= 64
        assert all(abs(point[2]) <= 1e-12 for point in points)
        contour = Polygon([(point[0], point[1]) for point in points])
        assert contour.is_valid and contour.area > 0
        contours.append(contour)

    outer = max(contours, key=lambda contour: contour.area)
    inner = [contour for contour in contours if contour is not outer]
    assert all(outer.contains(contour) for contour in inner)
    profile = outer.difference(unary_union(inner))
    assert profile.geom_type == "Polygon" and len(profile.interiors) == 3
    return profile


def named_nominal_contours(profile):
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
    points = [tuple(point[:2]) for point in list(polygon.exterior.coords)[:-1]]
    selected = {
        "E": min(points, key=lambda point: (-point[0], point[1])),
        "N": min(points, key=lambda point: (-point[1], point[0])),
        "S": min(points, key=lambda point: (point[1], point[0])),
        "W": min(points, key=lambda point: (point[0], point[1])),
    }
    return [(side, point) for side, point in selected.items()]


def expected_cam_items(profiles, stock, process):
    kerf = float(process["kerf_mm"])
    lead_length = float(process["lead_in_mm"])
    sheet_width, sheet_height = (
        float(value) for value in stock["available_sheet_mm"].lower().split("x")
    )
    sheet = Polygon(
        [(0, 0), (sheet_width, 0), (sheet_width, sheet_height), (0, sheet_height)]
    )
    material = unary_union(list(profiles.values()))
    items = []
    for part in ("LEFT_LINK", "RIGHT_LINK"):
        for contour, nominal in named_nominal_contours(profiles[part]).items():
            outer = contour == "OUTER"
            toolpath = nominal.buffer(
                kerf / 2 if outer else -kerf / 2,
                quad_segs=24,
                join_style="round",
            )
            assert toolpath.geom_type == "Polygon" and not toolpath.is_empty
            centre = toolpath.centroid.x, toolpath.centroid.y
            candidates = []
            for side, cut_start in axis_extreme_vertices(toolpath):
                dx, dy = cut_start[0] - centre[0], cut_start[1] - centre[1]
                length = math.hypot(dx, dy)
                direction = dx / length, dy / length
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
            assert candidates
            items.append(
                {
                    "part": part,
                    "contour": contour,
                    "outer": outer,
                    "toolpath": toolpath,
                    "candidates": candidates,
                }
            )
    return items


def expected_rapid_obstacles():
    obstacles = []
    for row in rows("rapid_keepouts.csv"):
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


def rapid_segment_is_clear(start, end, obstacles):
    segment = LineString([start, end])
    return all(
        segment.intersection(obstacle.buffer(-1e-7)).is_empty for obstacle in obstacles
    )


def independent_shortest_rapid_path(start, end, obstacles):
    corners = {
        (float(x), float(y))
        for obstacle in obstacles
        for x, y in list(obstacle.exterior.coords)[:-1]
    }
    nodes = [tuple(start), tuple(end), *sorted(corners)]
    adjacency = [[] for _ in nodes]
    for first, second in itertools.combinations(range(len(nodes)), 2):
        if rapid_segment_is_clear(nodes[first], nodes[second], obstacles):
            distance = math.dist(nodes[first], nodes[second])
            adjacency[first].append((second, distance))
            adjacency[second].append((first, distance))

    queue = [(0.0, 0)]
    distances = {0: 0.0}
    while queue:
        cost, node_index = heapq.heappop(queue)
        if cost > distances[node_index] + 1e-12:
            continue
        if node_index == 1:
            return cost
        for neighbour, edge_length in adjacency[node_index]:
            proposal = cost + edge_length
            if proposal < distances.get(neighbour, float("inf")) - 1e-12:
                distances[neighbour] = proposal
                heapq.heappush(queue, (proposal, neighbour))
    raise AssertionError("No collision-free rapid path exists")


def minimum_rapid_distance(items, rapid_start, obstacles):
    predecessors = {
        index: {
            other_index
            for other_index, other in enumerate(items)
            if item["outer"] and other["part"] == item["part"] and not other["outer"]
        }
        for index, item in enumerate(items)
    }
    states = {(0, -1, -1): 0.0}
    path_cache = {}

    def travel(start, end):
        key = tuple(start), tuple(end)
        if key not in path_cache:
            path_cache[key] = independent_shortest_rapid_path(start, end, obstacles)
        return path_cache[key]

    full_mask = (1 << len(items)) - 1
    for mask in range(full_mask + 1):
        for (state_mask, last_item, last_candidate), cost in list(states.items()):
            if state_mask != mask:
                continue
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
                    state = (
                        mask | (1 << item_index),
                        item_index,
                        candidate_index,
                    )
                    new_cost = cost + travel(previous, candidate["lead_start"])
                    states[state] = min(states.get(state, float("inf")), new_cost)
    return min(cost for state, cost in states.items() if state[0] == full_mask)


def test_waterjet_dxf_matches_both_mesh_profiles_and_stock():
    with open(os.path.join(ROOT, "design.json"), encoding="utf-8") as handle:
        report = json.load(handle)
    link = report["link"]
    checks = report["engineering_checks"]
    stock = next(
        item
        for item in rows("stock.csv")
        if item["stock_id"] == link["material_stock_id"]
    )
    path = os.path.join(ROOT, "cut_profiles.dxf")
    assert os.path.isfile(path) and os.path.getsize(path) < 5_000_000
    document = ezdxf.readfile(path)
    assert document.units == 4
    entities = list(document.modelspace())
    assert len(entities) == 8
    assert all(entity.dxftype() == "LWPOLYLINE" for entity in entities)
    assert {entity.dxf.layer for entity in entities} == {"LEFT_LINK", "RIGHT_LINK"}

    profiles = {
        "LEFT_LINK": profile_from_dxf_layer(document, "LEFT_LINK"),
        "RIGHT_LINK": profile_from_dxf_layer(document, "RIGHT_LINK"),
    }
    expected = {
        "LEFT_LINK": expected_link_profile(link, link["dogleg_offset_mm"]),
        "RIGHT_LINK": expected_link_profile(link, -link["dogleg_offset_mm"]),
    }
    mesh_names = {"LEFT_LINK": "left_link.stl", "RIGHT_LINK": "right_link.stl"}
    for layer in ("LEFT_LINK", "RIGHT_LINK"):
        candidate = profiles[layer]
        target = expected[layer]
        aligned = translate(
            candidate,
            xoff=target.centroid.x - candidate.centroid.x,
            yoff=target.centroid.y - candidate.centroid.y,
        )
        assert aligned.symmetric_difference(target).area <= max(
            SECTION_AREA_ABSOLUTE_TOLERANCE_MM2,
            target.area * SECTION_SHAPE_RELATIVE_TOLERANCE,
        )
        mesh = trimesh.load(
            os.path.join(ROOT, mesh_names[layer]), force="mesh", process=True
        )
        mesh_area = mesh.volume / link["thickness_mm"]
        assert abs(candidate.area - mesh_area) <= max(
            SECTION_AREA_ABSOLUTE_TOLERANCE_MM2,
            mesh_area * SECTION_SHAPE_RELATIVE_TOLERANCE,
        )

    sheet_width, sheet_height = (
        float(value) for value in stock["available_sheet_mm"].lower().split("x")
    )
    combined = unary_union(list(profiles.values()))
    min_x, min_y, max_x, max_y = combined.bounds
    measured_margin = min(min_x, min_y, sheet_width - max_x, sheet_height - max_y)
    measured_spacing = profiles["LEFT_LINK"].distance(profiles["RIGHT_LINK"])
    assert measured_margin >= 10 and measured_spacing >= 8
    assert profiles["LEFT_LINK"].intersection(profiles["RIGHT_LINK"]).is_empty
    assert (
        abs(checks["dxf_min_edge_margin_mm"] - measured_margin)
        <= DXF_METRIC_TOLERANCE_MM
    )
    assert (
        abs(checks["dxf_part_spacing_mm"] - measured_spacing) <= DXF_METRIC_TOLERANCE_MM
    )


def test_machine_toolpaths_are_compensated_safe_and_route_optimal():
    with open(os.path.join(ROOT, "design.json"), encoding="utf-8") as handle:
        design = json.load(handle)
    with open(os.path.join(ROOT, "cam_report.json"), encoding="utf-8") as handle:
        report = json.load(handle)
    assert set(report) == {
        "schema_version",
        "units",
        "machine_id",
        "kerf_mm",
        "lead_in_mm",
        "rapid_start_mm",
        "total_rapid_mm",
        "cut_sequence",
    }

    process = rows("waterjet.csv")[0]
    obstacles = expected_rapid_obstacles()
    assert report["schema_version"] == 1 and report["units"] == "mm"
    assert report["machine_id"] == process["machine_id"]
    assert abs(report["kerf_mm"] - float(process["kerf_mm"])) < 1e-9
    assert abs(report["lead_in_mm"] - float(process["lead_in_mm"])) < 1e-9
    rapid_start = (
        float(process["rapid_start_x_mm"]),
        float(process["rapid_start_y_mm"]),
    )
    assert np.allclose(
        report["rapid_start_mm"], rapid_start, atol=CAM_POINT_TOLERANCE_MM
    )

    stock = next(
        item
        for item in rows("stock.csv")
        if item["stock_id"] == design["link"]["material_stock_id"]
    )
    nominal_document = ezdxf.readfile(os.path.join(ROOT, "cut_profiles.dxf"))
    profiles = {
        "LEFT_LINK": profile_from_dxf_layer(nominal_document, "LEFT_LINK"),
        "RIGHT_LINK": profile_from_dxf_layer(nominal_document, "RIGHT_LINK"),
    }
    items = expected_cam_items(profiles, stock, process)
    item_by_name = {(item["part"], item["contour"]): item for item in items}

    toolpath_document = ezdxf.readfile(os.path.join(ROOT, "machine_toolpaths.dxf"))
    assert toolpath_document.units == 4
    entities = list(toolpath_document.modelspace())
    assert len(entities) == 16
    assert all(entity.dxftype() == "LWPOLYLINE" for entity in entities)
    assert {entity.dxf.layer for entity in entities} == {
        "TOOL_INNER",
        "TOOL_OUTER",
        "LEAD_IN",
    }
    lead_entities = [entity for entity in entities if entity.dxf.layer == "LEAD_IN"]
    cut_entities = [entity for entity in entities if entity.dxf.layer != "LEAD_IN"]
    assert len(lead_entities) == len(cut_entities) == 8

    lead_lines = []
    for entity in lead_entities:
        points = list(entity.get_points("xyb"))
        assert not entity.closed and len(points) == 2
        assert all(abs(point[2]) <= 1e-12 for point in points)
        lead_lines.append((points[0][:2], points[1][:2]))

    submitted_cuts = []
    for entity in cut_entities:
        points = list(entity.get_points("xyb"))
        assert entity.closed and len(points) >= 64
        assert all(abs(point[2]) <= 1e-12 for point in points)
        polygon = Polygon([(point[0], point[1]) for point in points])
        assert polygon.is_valid and polygon.area > 0
        submitted_cuts.append(
            {
                "layer": entity.dxf.layer,
                "polygon": polygon,
                "cut_start": points[0][:2],
            }
        )

    sequence = report["cut_sequence"]
    assert len(sequence) == 8
    assert [record["sequence"] for record in sequence] == list(range(1, 9))
    names = [(record["part"], record["contour"]) for record in sequence]
    assert len(set(names)) == 8 and set(names) == set(item_by_name)

    used_cut_entities = set()
    used_lead_entities = set()
    actual_rapid = 0.0
    previous = rapid_start
    completed = set()
    for record in sequence:
        assert set(record) == {
            "sequence",
            "part",
            "contour",
            "side",
            "rapid_waypoints_mm",
            "lead_start_mm",
            "cut_start_mm",
        }
        name = record["part"], record["contour"]
        item = item_by_name[name]
        if item["outer"]:
            assert {
                (item["part"], "PIN_B"),
                (item["part"], "PIN_C"),
                (item["part"], "SLOT"),
            } <= completed
        completed.add(name)

        assert record["side"] in {candidate["side"] for candidate in item["candidates"]}
        expected_candidate = next(
            candidate
            for candidate in item["candidates"]
            if candidate["side"] == record["side"]
        )
        assert np.allclose(
            record["cut_start_mm"],
            expected_candidate["cut_start"],
            atol=CAM_POINT_TOLERANCE_MM,
        )
        assert np.allclose(
            record["lead_start_mm"],
            expected_candidate["lead_start"],
            atol=CAM_POINT_TOLERANCE_MM,
        )

        expected_layer = "TOOL_OUTER" if item["outer"] else "TOOL_INNER"
        matches = sorted(
            (
                (
                    submitted["polygon"].symmetric_difference(item["toolpath"]).area,
                    index,
                    submitted,
                )
                for index, submitted in enumerate(submitted_cuts)
                if index not in used_cut_entities
                and submitted["layer"] == expected_layer
            ),
            key=lambda match: match[0],
        )
        difference, cut_index, submitted = matches[0]
        assert difference <= max(
            CAM_SHAPE_ABSOLUTE_TOLERANCE_MM2,
            item["toolpath"].area * CAM_SHAPE_RELATIVE_TOLERANCE,
        )
        assert np.allclose(
            submitted["cut_start"],
            record["cut_start_mm"],
            atol=CAM_POINT_TOLERANCE_MM,
        )
        used_cut_entities.add(cut_index)

        matching_leads = [
            (index, lead)
            for index, lead in enumerate(lead_lines)
            if index not in used_lead_entities
            and np.allclose(
                lead[1], record["cut_start_mm"], atol=CAM_POINT_TOLERANCE_MM
            )
        ]
        assert len(matching_leads) == 1
        lead_index, lead = matching_leads[0]
        assert np.allclose(
            lead[0], record["lead_start_mm"], atol=CAM_POINT_TOLERANCE_MM
        )
        assert (
            abs(math.dist(lead[0], lead[1]) - float(process["lead_in_mm"]))
            <= CAM_POINT_TOLERANCE_MM
        )
        used_lead_entities.add(lead_index)

        waypoints = [tuple(point) for point in record["rapid_waypoints_mm"]]
        obstacle_corners = {
            (float(x), float(y))
            for obstacle in obstacles
            for x, y in list(obstacle.exterior.coords)[:-1]
        }
        assert all(
            any(
                np.allclose(waypoint, corner, atol=CAM_POINT_TOLERANCE_MM)
                for corner in obstacle_corners
            )
            for waypoint in waypoints
        )
        rapid_points = [tuple(previous), *waypoints, tuple(record["lead_start_mm"])]
        assert all(
            rapid_segment_is_clear(start, end, obstacles)
            for start, end in zip(rapid_points, rapid_points[1:])
        )
        leg_length = sum(
            math.dist(start, end) for start, end in zip(rapid_points, rapid_points[1:])
        )
        shortest_leg = independent_shortest_rapid_path(
            previous, record["lead_start_mm"], obstacles
        )
        assert abs(leg_length - shortest_leg) <= CAM_ROUTE_TOLERANCE_MM
        actual_rapid += leg_length
        previous = record["cut_start_mm"]

    assert len(used_cut_entities) == len(used_lead_entities) == 8
    optimum = minimum_rapid_distance(items, rapid_start, obstacles)
    assert abs(report["total_rapid_mm"] - actual_rapid) <= CAM_ROUTE_TOLERANCE_MM
    assert abs(report["total_rapid_mm"] - optimum) <= CAM_ROUTE_TOLERANCE_MM


def expected_sweep_envelope(motion, offset, width, end_od, facets):
    quadrant_segments = facets // 4
    silhouettes = []
    for pose_index in range(len(motion) - 1):
        for step in range(9):
            fraction = step / 8
            b = (
                np.array(motion[pose_index][0]) * (1 - fraction)
                + np.array(motion[pose_index + 1][0]) * fraction
            )
            c = (
                np.array(motion[pose_index][1]) * (1 - fraction)
                + np.array(motion[pose_index + 1][1]) * fraction
            )
            direction = c - b
            length = np.linalg.norm(direction)
            elbow = (b + c) / 2 + np.array((-direction[1], direction[0])) * (
                offset / length
            )
            web = LineString([b, elbow, c]).buffer(
                width / 2,
                quad_segs=quadrant_segments,
                cap_style="round",
                join_style="round",
            )
            silhouettes.append(
                unary_union(
                    [
                        web,
                        Point(*b).buffer(end_od / 2, quad_segs=quadrant_segments),
                        Point(*c).buffer(end_od / 2, quad_segs=quadrant_segments),
                    ]
                )
            )
    return unary_union(silhouettes)


def test_sweep_envelope_matches_the_reconstructed_motion():
    with open(os.path.join(ROOT, "design.json"), encoding="utf-8") as handle:
        report = json.load(handle)
    link = report["link"]
    _, _, _, motion = ground_truth()
    expected = expected_sweep_envelope(
        motion,
        link["dogleg_offset_mm"],
        link["body_width_mm"],
        link["end_outer_diameter_mm"],
        link["facet_count_per_circle"],
    )
    mesh = trimesh.load(
        os.path.join(ROOT, "left_sweep_envelope.stl"), force="mesh", process=True
    )
    assert mesh.is_watertight and mesh.is_winding_consistent and mesh.volume > 0
    assert len(mesh.split()) == 1 and abs(mesh.bounds[0, 2]) <= MESH_Z_TOLERANCE_MM
    assert abs(mesh.extents[2] - link["thickness_mm"]) <= THICKNESS_TOLERANCE_MM
    expected_bounds = np.array(expected.bounds).reshape(2, 2)
    assert np.allclose(
        mesh.bounds[:, :2], expected_bounds, atol=ENVELOPE_BOUNDS_TOLERANCE_MM
    )
    measured_area = mesh.volume / link["thickness_mm"]
    assert abs(measured_area - expected.area) <= max(
        ENVELOPE_AREA_ABSOLUTE_TOLERANCE_MM2,
        expected.area * ENVELOPE_AREA_RELATIVE_TOLERANCE,
    )
    assert abs(
        report["engineering_checks"]["left_sweep_envelope_area_mm2"] - measured_area
    ) <= max(
        ENVELOPE_AREA_ABSOLUTE_TOLERANCE_MM2,
        measured_area * ENVELOPE_AREA_RELATIVE_TOLERANCE,
    )
