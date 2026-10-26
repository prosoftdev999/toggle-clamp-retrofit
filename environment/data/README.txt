These are shop-floor exports from the carton-guide linkage investigation.

probe_runs.csv contains repeated CMM observations taken after the fixture was
moved between setups.  Coordinates are in each setup's local frame.  DATUM_A
and DATUM_D are the two fixed frame pivots; MOVING_B and MOVING_C are the link
pin centres.  A few rows carry the operator's original label rather than the
corrected feature label.

keepouts.csv describes circular keep-outs in machine coordinates.
load_cases.csv contains the measured peak pin loads used for the replacement
link check.

waterjet.csv contains the selected machine's measured kerf, required straight
lead-in length, and rapid-traverse home position for the CAM handoff.

rapid_keepouts.csv lists the rectangular fixture clamps that the waterjet head
must clear during rapid travel.  The listed head clearance expands each side.
