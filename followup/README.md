# PX4 #29954 follow-up evidence

The local candidate shares pitch capture storage and interpolation in `VtolType`. Standard and Tailsitter supply their own final fixed-wing-frame transition pitch. The candidate removes the extra quaternion normalization and 100 ms expiry added by the existing PR. It clears the Tailsitter reference when it observes altitude control turn off. The stock transition-completion rule stays in the proposed diff.

The November 26 audit also reused `can_transition_on_ground()` in the Tailsitter capture hook. This keeps the armed and landed checks in one place. The base class uses its existing TECS readiness and blend timing, and Tailsitter uses its existing frame rotation. There is one shared interpolation using `math::lerp`. The optional frame-specific hooks keep the different capture and pilot-mode rules explicit.

- [Proposed firmware diff](PROPOSED.patch)
- [Final candidate nominal overlay](evidence/final-patch-nominal-handoff.png)
- [Timeout comparator overlay](evidence/same-build-timeout-comparison.png)
- [Angle comparison overlay](evidence/pitch-gate-comparison.png)

## What the runs showed

| Nominal Tailsitter Duo | Pitch-reference change around FW entry | Peak nose-down rate, first 1.5 s |
|---|---:|---:|
| Upstream stock, 3 runs | 4.65°–6.06° | 46.8–51.1°/s |
| Published PR head with 100 ms guard, 3 runs | 0.55°–0.73° | 17.7–31.7°/s |
| Shared prototype without guard, 3 runs | 0.56°–0.72° | 17.9–32.8°/s |
| November 25 candidate with mode-exit cleanup, 1 run | 0.58° | 16.2°/s |
| November 26 candidate with ground-check reuse, 1 run | 0.51° | 17.7°/s |

Pitch-reference change is the absolute difference from the last routed sample before −25 ms to the first sample at or after +50 ms. The negative-side Tailsitter reference is rotated into the FW frame. FW entry is the first logged FW VTOL state. Ranges show the spread across simulator runs.

### 100 ms expiry on and off in the same executable

TECS kept computing, and the real FW virtual attitude setpoints kept flowing. The diagnostic delayed publication of `tecs_status`. Each pair used the same diagnostic executable and toggled only the 100 ms pitch-expiry policy. Dynamics and exact handoff timing vary across runs.

| 550 ms status delay | Peak routed pitch step between +80 and +180 ms, expiry enabled → removed | Peak nose-down rate, expiry enabled → removed |
|---|---:|---:|
| Airspeed-gated entry, about +4–5° FW pitch | 8.89° → <0.001° | 111.7 → 17.8°/s |
| Angle-gated entry, about +29° FW pitch | 29.31° → <0.001° | 231.6 → 46.5°/s |

The existing 100 ms expiry released the captured pitch before TECS status arrived. In those tests the expiry created a second command step. With the existing TECS wait, the blend started from the held reference when status arrived. When status stayed absent for about eight seconds, both near-level variants descended by 4.4–4.5 m. At approximately +29° entry, keeping the pitch hold led to a 14.1 m climb, while the 100 ms expiry produced a 29.4° command step and ended 1.6 m below FW entry. The hold can remain active under prolonged status loss. A smooth, bounded release under a TECS failure remains a separate design question.

A separate whole-controller outage removed both TECS and FW virtual setpoints and restarted the controller with the `vtol` argument. With the final patch, the first routed FW sample resumed at +588 ms and still used the captured pitch, +1.49° versus a 0° input. The roughly 266°/s initial nose-down peak happened at the start of the outage, before fresh FW commands returned. The patch depends on the existing FW setpoint stream.

### Handoff angle with simulated airspeed gating disabled

These four trials put the angle rule in control. Each row is one run. The experimental +15° rule observes the FW-frame estimated attitude. It was enabled only in these diagnostic runs.

| PX4 pitch blend | Completion condition | Front-transition duration | FW pitch at entry | Pitch-reference change | Peak nose-down rate | Height loss from front start through FW +3 s |
|---|---|---:|---:|---:|---:|---:|
| Off | Stock | 3.30 s | +29.0° | 31.00° | 237.8°/s | 1.20 m |
| Off | +15° FW frame | 4.29 s | +14.9° | 12.14° | 110.2°/s | 0.90 m |
| On | Stock | 3.27 s | +29.1° | 1.90° | 46.6°/s | 0.79 m |
| On | +15° FW frame | 4.26 s | +15.0° | 0.91° | 25.1°/s | 1.00 m |

The later switch helped this fixture and extended the front transition by about one second. The blend at the stock gate reduced the reference step further than the angle change alone. Testing other airframes and entries with unreliable airspeed would be appropriate before proposing a default change.

### Other operating paths

- **Standard VTOL, SIH.** Exercised the shared path. Routed and incoming FW quaternions match after the blend. The nominal step and peak rate stayed small.
- **Tiltrotor, Gazebo.** The stock and final builds completed the transition and routed the FW quaternion unchanged in the sampled post-entry interval. The first four setup attempts lacked simulator sensor data. Supplying PX4's Gazebo server configuration fixed that setup.
- **Stabilized, SIH.** Positive and negative pilot pitch requests passed through directly. The test returned to multicopter.
- **Auto → Stabilized → Auto, SIH.** The earlier shared prototype resumed using the held pitch after returning to Auto. The final version cleared the Tailsitter's reference and followed the FW input when altitude control returned 552 ms after FW entry.
- **Aborted front transition, SIH.** The final candidate returned to multicopter without entering FW.
- **Quaternion input validity.** All 5,045 FW virtual attitude quaternions from the six published November 24 SITL logs were finite and nonzero. The maximum absolute deviation from unit length was 1.43 × 10⁻⁷. No failure in this input path supported adding repeated normalization.

Four November 26 smoke runs verified the exact revised patch. Tailsitter and Standard completed their nominal front transitions. Standard's routed quaternion matched its input after the blend. Tiltrotor matched the input throughout the sampled FW interval. Tailsitter also passed the Auto to Stabilized to Auto run with zero pitch-command difference after altitude control resumed.

The proposed revision was formatted using PX4's astyle wrapper and built as `px4_sitl_default`. The clean source comparison is against PR head `4072b456a3`, with upstream stock `5db5c623a1`. Each local dirty executable is identified by its source patch and executable SHA-256. The comparison builds include SITL-only fault and gate instrumentation. The final candidate executable is built without that instrumentation. A pre-entry loss of the MC setpoint stream and behavior on flight hardware remain unverified.

## Evidence and reproduction

[Download the SITL-only archive](sih-followup-evidence.zip) · [Metrics CSV](evidence/metrics.csv).

The tested source matches PR commit `614052724d14bad7d0b499eb738e9d26454a453a`. The build was produced from the approved local patch before the commit was created. Each run records the executable digest.

This ZIP retains 36 of the 47 completed SITL ULogs, simulator command events, plots, build provenance, and the source patches for each test executable. All 47 logs reported zero ULog dropouts. The per-run metrics record each log and executable digest. The configured logger topics use interval zero. The firmware tested here ran inside Docker with networking disabled. The running harness ends the nominal trace eight seconds after FW entry. The full ULogs are available for independent timing checks.

`run_case.sh` selects the fixture and scenario. It expects the named PX4 build’s `bin` and `etc` under `builds/<variant>/`. Build snapshots include the patch relative to their recorded git head. The clean stock and published guarded executables came from the November 24 exact-commit builds. The candidate source tree matches `PROPOSED.patch`. Apply the corresponding patch to its recorded head and build PX4 SITL to reproduce a source variant independently. Use `analyze_followup.py` to regenerate the tables and plots from the full log directory.
