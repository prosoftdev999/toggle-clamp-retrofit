# Carton-guide linkage retrofit

The outside links on our carton-guide clamp are bowed, and the old drawing is
missing the dogleg offset. Reconstruct the linkage from the inspection exports
in `/app/data` and provide a waterjet-ready replacement pair without moving the
frame pivots or pneumatic line.

## Recover the linkage

The three CMM setups have different origins and rotations, and one pose has its
MOVING_B/MOVING_C labels reversed. Register each observation from DATUM_A with
the measured A-to-D direction as local +x and `(-dy, dx)` as local +y; do not
reflect a setup. Within each setup, correct the B/C assignment that minimizes
the summed squared travel of both tracks over its ordered poses.

Report the median of all A-D spans and the median of all corrected B-C spans.
For motion calculations, combine setups at each pose by coordinate-wise median,
then align local +x with the machine-frame A-to-D vector `(92, 14)`. In that
machine frame DATUM_A is `(0, 0)`; keep-out coordinates are already expressed
there.

## Design requirements

Make mirrored left- and right-hand dogleg links using the recovered B-C pin
spacing and one unthinned stock entry from `stock.csv`. Both parts must fit its
listed sheet and satisfy:

- 10.4 mm through pin holes, at least 22 mm end OD, and at least 18 mm outer
  web-envelope width between bosses;
- constant stock thickness, one connected solid per link, at least 64 facets
  per circular edge, and pair mass no greater than 55 g;
- yield safety factor at least 2.0 for the largest load in `load_cases.csv`,
  assuming equal load sharing and per-link net area `thickness * (end OD -
  hole diameter)`;
- at least 2 mm swept clearance after each radius and allowance in
  `keepouts.csv` is applied;
- one closed round-ended through-slot per link. Choose a constant width from
  6–8 mm and `lightening_slot_end_fraction` from 0.25–0.35. Its centreline runs
  from that fraction along the first pin-to-elbow segment, through the elbow,
  to `(1 - fraction)` along the elbow-to-second-pin segment. It must remain
  separate from the pin holes and outer edge. The 18 mm rule applies to the
  outer envelope; this internal slot is allowed.

`keepouts.csv` is the left-side fixture survey. Divide every interval between
consecutive recorded poses into eight equal linear subintervals and evaluate
all nine positions, including both endpoints. At each sample, put the left
elbow at the B-C midpoint plus
`dogleg_offset_mm * (-dy, dx) / B-C length`. Treat the B-elbow-C centreline as
expanded by half the outer web width. Clearance is centre-to-segment distance
minus keep-out radius, listed allowance, and half web width. Report the minimum
across the left-side sweep.

Also make a conservative left-link sweep envelope for fixture review. Union
the outer link silhouettes at those same motion samples, with the end bosses
included and the pin holes and lightening slot filled. Extrude that planar union
from `z = 0` by the selected stock thickness and report its XY area.

Provide the clearance review trail in `clearance_audit.csv`. Include one row
for every pose interval, each of its nine sample positions, and every keep-out;
the repeated endpoint between adjacent intervals remains in both intervals.
Use exactly these columns:

`interval_start_pose, interval_end_pose, sample_index, sample_fraction,
keepout, controlling_segment, closest_x_mm, closest_y_mm, clearance_mm`

`sample_index` runs from 0 through 8 and `sample_fraction` is
`sample_index / 8`. For each row, compare the B-to-elbow and elbow-to-C
segments. `controlling_segment` is `B-elbow` or `elbow-C`, whichever has the
smaller centreline distance to the keep-out centre; choose `B-elbow` on a tie.
The closest point is the ordinary perpendicular projection clamped to that
finite segment. Record its machine-frame XY coordinates and the clearance
defined above. Keep `sample_index` as an integer; write the fraction,
coordinates, and clearance to three decimal places. Row order does not matter,
but every required combination must appear exactly once.

In `design.json`, summarize the audit with its row count and the unique row of
minimum clearance. The witness must repeat that row's interval, sample index,
keep-out, controlling segment, and closest XY point.

The waterjet shop also needs `cut_profiles.dxf`. Its importer accepts closed
`LWPOLYLINE` contours without bulges, splines, rotation, or scaling. Put the
left part on layer `LEFT_LINK` and the right part on `RIGHT_LINK`, with exactly
four contours on each layer: the outer edge, two pin holes, and the lightening
slot. The profiles must match the corresponding STL XY sections up to
translation. Nest both parts inside the selected stock sheet with at least
10 mm to every sheet edge and 8 mm between parts. They may be translated but
must not overlap. Set DXF units to millimetres and report the measured minimum
edge margin and inter-part spacing.

Turn that nominal nest into a machine handoff using the single row in
`waterjet.csv`. In `machine_toolpaths.dxf`, use round Euclidean offsets: move
each outer contour outward into scrap by half the kerf and each hole or slot
contour inward into its void by half the kerf. Keep the nominal placements.
Write eight closed, bulge-free `LWPOLYLINE` cutter paths with at least 64
vertices each on `TOOL_OUTER` or `TOOL_INNER`, plus one open, two-point
`LWPOLYLINE` lead-in per cutter path on `LEAD_IN`. Use millimetres.

A cutter path may start only at its east, north, south, or west vertex. For an
axis tie, choose the lower coordinate on the other axis. An outer lead starts
the listed lead-in distance farther along the ray from that contour's centroid
through the cut start; an inner lead starts the same distance toward its
contour centroid. Discard a start if its lead is not wholly inside the sheet
and scrap: outside both solid profiles for an outer cut, or inside that
specific nominal hole or slot for an inner cut.

Plan all eight cuts from the rapid start in `waterjet.csv`. A closed cut ends
where it starts. Expand each axis-aligned rectangle in `rapid_keepouts.csv` by
its `head_clearance_mm` on all four sides. A rapid may touch this expanded
boundary but must not enter its interior; it may turn only at expanded-rectangle
corners. Use a shortest legal polyline for every rapid leg. All three internal
contours of a part must be cut before that part's outer contour. Among every
permitted start and precedence-respecting order, minimize total obstacle-aware
rapid travel from the machine start to the first lead and between cuts. Report
the selected route in `cam_report.json`; each sequence record identifies `part` as
`LEFT_LINK` or `RIGHT_LINK`, `contour` as `PIN_B`, `PIN_C`, `SLOT`, or `OUTER`,
the chosen `side`, `rapid_waypoints_mm` as the intermediate expanded-corner XY
points (excluding leg endpoints), and its lead and cut XY points. Report total
rapid travel to three decimals.

## Coordinate and output contract

Use millimetres. In the two link STLs, place pin centres at `(0, 0)` and
`(pin_span_mm, 0)`, with the right XY profile mirrored from the left. The left
uses the positive dogleg convention above. Put the lower face at `z = 0`; use
finite facets enclosing positive volume.

Create exactly:

- `/app/output/left_link.stl`
- `/app/output/right_link.stl`
- `/app/output/left_sweep_envelope.stl`
- `/app/output/clearance_audit.csv`
- `/app/output/cut_profiles.dxf`
- `/app/output/machine_toolpaths.dxf`
- `/app/output/cam_report.json`
- `/app/output/design.json`

`design.json` may have only the six top-level keys shown here:

```json
{
  "schema_version": 1,
  "units": "mm",
  "reconstruction": {"frame_pivot_span_mm": 0.0, "moving_link_span_mm": 0.0, "corrected_feature_swap": "setup/pose"},
  "link": {"material_stock_id": "", "pin_span_mm": 0.0, "dogleg_offset_mm": 0.0, "body_width_mm": 0.0, "end_outer_diameter_mm": 0.0, "pin_hole_diameter_mm": 0.0, "lightening_slot_width_mm": 0.0, "lightening_slot_end_fraction": 0.0, "thickness_mm": 0.0, "facet_count_per_circle": 0},
  "engineering_checks": {"minimum_swept_clearance_mm": 0.0, "jam_load_safety_factor": 0.0, "pair_mass_g": 0.0, "left_sweep_envelope_area_mm2": 0.0, "clearance_audit_rows": 0, "critical_clearance_witness": {"interval": "pose-pose", "sample_index": 0, "keepout": "", "controlling_segment": "", "closest_point_mm": [0.0, 0.0]}, "dxf_min_edge_margin_mm": 0.0, "dxf_part_spacing_mm": 0.0},
  "artifacts": ["left_link.stl", "right_link.stl", "left_sweep_envelope.stl", "clearance_audit.csv", "cut_profiles.dxf"]
}
```

Report mesh-derived values, using three decimals where inspection precision
supports it. The meshes, not JSON claims alone, determine geometry and mass.

`cam_report.json` has this shape:

```json
{
  "schema_version": 1,
  "units": "mm",
  "machine_id": "",
  "kerf_mm": 0.0,
  "lead_in_mm": 0.0,
  "rapid_start_mm": [0.0, 0.0],
  "total_rapid_mm": 0.0,
  "cut_sequence": [{"sequence": 1, "part": "LEFT_LINK", "contour": "PIN_B", "side": "E", "rapid_waypoints_mm": [], "lead_start_mm": [0.0, 0.0], "cut_start_mm": [0.0, 0.0]}]
}
```

You have 7200 seconds to complete this task. Do not cheat by using online solutions or hints specific to this task.
