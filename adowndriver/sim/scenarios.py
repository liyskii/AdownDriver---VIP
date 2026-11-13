"""
The 16-scenario validation matrix (W12-W13 of the timeline).

Each scenario is used twice:
  * in the software-in-the-loop runner (``python -m adowndriver validate``), and
  * as the script for the physical arena trial (docs/VALIDATION_PROTOCOL.md);
    the same ids are selectable in the dashboard so real runs are labelled
    consistently.
"""
from __future__ import annotations

from dataclasses import dataclass, field

from .synthetic_driver import Segment


@dataclass
class Scenario:
    id: str
    name: str
    duration: float
    timeline: list
    actions: list = field(default_factory=list)          # (t, action, arg)
    expect_reach: list = field(default_factory=list)     # states that must occur
    expect_never: list = field(default_factory=list)     # states that must not occur
    expect_final: str = ""                               # state at the end
    expect_reason: str = ""                              # reason that must appear
    gaps: list = field(default_factory=list)             # missing tape (arc-length ranges)
    category: str = "driver"
    arena_procedure: str = ""


def _alert(t0, t1):
    return Segment(t0, t1, "alert", "alert")


SCENARIOS: list[Scenario] = [
    Scenario("S01", "Baseline cruise, natural blinking", 40, [_alert(0, 40)],
             expect_never=["WARNING", "SPEED_REDUCTION", "CONTROLLED_STOP", "EMERGENCY_STOP"],
             expect_final="NORMAL_CRUISE",
             arena_procedure="Volunteer looks at the road monitor and blinks normally for 40 s."),
    Scenario("S02", "Rapid blinking (300 ms every ~2 s)", 30, [Segment(0, 30, "blink_fast", "alert")],
             expect_never=["SPEED_REDUCTION", "CONTROLLED_STOP", "EMERGENCY_STOP"],
             expect_final="NORMAL_CRUISE",
             arena_procedure="Volunteer blinks deliberately every 2 s (metronome on screen)."),
    Scenario("S03", "Talking / reading aloud", 30, [Segment(0, 30, "talk", "alert")],
             expect_never=["WARNING", "SPEED_REDUCTION", "CONTROLLED_STOP", "EMERGENCY_STOP"],
             expect_final="NORMAL_CRUISE",
             arena_procedure="Volunteer reads the provided paragraph aloud."),
    Scenario("S04", "Single yawn", 25, [_alert(0, 8), Segment(8, 13, "yawn", "yawn"), _alert(13, 25)],
             expect_reach=["WARNING"], expect_never=["SPEED_REDUCTION", "CONTROLLED_STOP"],
             expect_final="NORMAL_CRUISE", expect_reason="YAWN",
             arena_procedure="At the cue, one natural/forced yawn of ~4 s."),
    Scenario("S05", "Repeated yawning (3 in 30 s)", 48,
             [_alert(0, 5), Segment(5, 9, "yawn", "yawn"), _alert(9, 15), Segment(15, 19, "yawn", "yawn"),
              _alert(19, 25), Segment(25, 29, "yawn", "yawn"), _alert(29, 48)],
             expect_reach=["WARNING", "SPEED_REDUCTION"], expect_never=["CONTROLLED_STOP"],
             expect_final="NORMAL_CRUISE", expect_reason="REPEATED_YAWNING",
             arena_procedure="Three yawns 10 s apart at the cues."),
    Scenario("S06", "Microsleep 1.8 s", 25, [_alert(0, 8), Segment(8, 9.8, "closed", "drowsy"), _alert(9.8, 25)],
             expect_reach=["WARNING"], expect_never=["SPEED_REDUCTION", "CONTROLLED_STOP"],
             expect_final="NORMAL_CRUISE", expect_reason="EYE_CLOSURE",
             arena_procedure="Eyes closed for 1.8 s at the cue (cue disappears when to open)."),
    Scenario("S07", "Microsleep 3.2 s", 28, [_alert(0, 8), Segment(8, 11.2, "closed", "drowsy"), _alert(11.2, 28)],
             expect_reach=["WARNING", "SPEED_REDUCTION"], expect_never=["CONTROLLED_STOP"],
             expect_final="NORMAL_CRUISE", expect_reason="SUSTAINED_EYE_CLOSURE",
             arena_procedure="Eyes closed for 3.2 s at the cue."),
    Scenario("S08", "Sustained closure -> graduated safe stop, then reset", 34,
             [_alert(0, 8), Segment(8, 20, "closed", "drowsy"), _alert(20, 34)],
             actions=[(25.0, "cmd", "reset"), (26.5, "cmd", "start")],
             expect_reach=["WARNING", "SPEED_REDUCTION", "CONTROLLED_STOP", "STANDBY"],
             expect_never=["EMERGENCY_STOP"], expect_final="NORMAL_CRUISE", expect_reason="PERSISTENT_DROWSINESS",
             arena_procedure="Eyes closed until the robot has stopped; operator presses RESET then START."),
    Scenario("S09", "Looking away 3 s (distraction)", 25,
             [_alert(0, 8), Segment(8, 11, "look_away", "distracted"), _alert(11, 25)],
             expect_reach=["WARNING"], expect_never=["SPEED_REDUCTION", "CONTROLLED_STOP"],
             expect_final="NORMAL_CRUISE", expect_reason="DRIVER_DISTRACTED",
             arena_procedure="Volunteer turns the head ~45 deg to the side for 3 s."),
    Scenario("S10", "Driver camera occluded 2 s", 25,
             [_alert(0, 8), Segment(8, 10, "occluded", "absent"), _alert(10, 25)],
             expect_reach=["SPEED_REDUCTION"], expect_never=["CONTROLLED_STOP", "EMERGENCY_STOP"],
             expect_final="NORMAL_CRUISE", expect_reason="LOW_DRIVER_CONFIDENCE", category="perception",
             arena_procedure="Cover the C270 lens with a hand for 2 s."),
    Scenario("S11", "Driver leaves the seat 7 s", 25,
             [_alert(0, 8), Segment(8, 15, "occluded", "absent"), _alert(15, 25)],
             expect_reach=["SPEED_REDUCTION", "CONTROLLED_STOP"], expect_never=["EMERGENCY_STOP"],
             expect_final="CONTROLLED_STOP", expect_reason="DRIVER_NOT_MONITORABLE", category="perception",
             arena_procedure="Volunteer leaves the camera view for 7 s."),
    Scenario("S12", "Dim lighting (lamp at 20 %)", 30, [Segment(0, 30, "dim", "alert")],
             actions=[(0.0, "brightness", 0.45)],
             expect_never=["SPEED_REDUCTION", "CONTROLLED_STOP", "EMERGENCY_STOP"],
             expect_final="NORMAL_CRUISE", category="lighting",
             arena_procedure="Room lights off, LED desk lamp at minimum aimed at the track."),
    Scenario("S13", "Glare (lamp reflection on mat + bright face)", 30, [Segment(0, 30, "glare", "alert")],
             actions=[(0.0, "glare", (210, 150, 22))],
             expect_never=["CONTROLLED_STOP", "EMERGENCY_STOP"], expect_final="NORMAL_CRUISE",
             category="lighting",
             arena_procedure="LED lamp at maximum, aimed low across the mat to create specular glare."),
    Scenario("S14", "Static obstacle on the lane", 26, [_alert(0, 26)],
             actions=[(6.0, "obstacle", 0.60), (15.0, "clear_obstacles", None)],
             expect_reach=["EMERGENCY_STOP"], expect_final="NORMAL_CRUISE", expect_reason="OBSTACLE",
             category="fault",
             arena_procedure="Place a 10 cm box on the lane centre ~60 cm ahead; remove after the stop."),
    Scenario("S15", "Wi-Fi / heartbeat loss 1 s", 22, [_alert(0, 22)],
             actions=[(10.0, "link_down", 1.0)],
             expect_reach=["EMERGENCY_STOP"], expect_final="NORMAL_CRUISE", expect_reason="HEARTBEAT_LOSS",
             category="fault",
             arena_procedure="Unplug Node 1's network (or `sudo ip link set wlan0 down` for 1 s)."),
    Scenario("S16", "Lane markings lost (tape gap)", 18, [_alert(0, 18)],
             gaps=[(1.70, 2.6)],
             expect_reach=["EMERGENCY_STOP"], expect_final="EMERGENCY_STOP", expect_reason="LANE_LOST",
             category="fault",
             arena_procedure="Remove 80 cm of both tapes on the straight (or cover the Pi Camera)."),
]

BY_ID = {s.id: s for s in SCENARIOS}
