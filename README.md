# PX4 PR #29954 · November 24 review evidence

The exact PR build reduced the routed pitch-reference change at nominal fixed-wing (FW) entry from **4.65° to 0.59°**, and the peak nose-down rate in the first 1.5 s from **47.22°/s to 17.32°/s**. Two corrected controller-restart tests verified that the first resumed fixed-wing attitude quaternion passed through the VTOL router unchanged. All six logs came from the built-in PX4 SIH Tailsitter Duo (`sihsim_xvert`, airframe 10042).

[Reviewer request](https://github.com/PX4/PX4-Autopilot/pull/29954#issuecomment-7107011912) · [Nominal plot](evidence/nominal-handoff.png) · [Delayed restart plot](evidence/delayed-handoff.png) · [Delayed repeat plot](evidence/delayed-repeat-handoff.png)

## Results

| Measurement | Stock | PR #29954 |
| --- | ---: | ---: |
| Nominal pitch reference change around FW entry | 4.65° | 0.59° |
| Nominal peak nose-down body rate, first 1.5 s | 47.22°/s | 17.32°/s |
| Nominal height loss from FW entry, first 3 s | 1.58 m | 1.47 m |
| Nominal motor rail samples, +0.6 to +3.0 s | 0 / 513 | 0 / 543 |
| First fresh matched quaternion, both delayed pairs | +588 ms | +596 ms |
| First routed-to-virtual quaternion difference, both delayed pairs | 0 | 0 |
| Delayed peak nose-down body rate, pair 1 / pair 2 | 260.96 / 265.40°/s | 264.19 / 257.62°/s |
| Delayed time of nose-down peak, pair 1 / pair 2 | +40 / +48 ms | +40 / +40 ms |
| Delayed height loss from FW entry, first 3 s, pair 1 / pair 2 | 7.17 / 6.99 m | 7.30 / 6.85 m |
| Delayed motor rail samples, +0.6 to +3.0 s, pair 1 / pair 2 | 0 / 554, 0 / 545 | 1 / 545, 0 / 550 |
| Recorded ULog dropouts across three runs per build | 0 | 0 |

The nominal figures describe one matched pair with identical initial parameters and mission. Measured pitch at FW entry differed by 1.21° between those runs. The pitch-reference change is the absolute difference between the last routed sample before −25 ms and the first sample at or after +50 ms. It measures the handoff neighborhood across a common window. Source timestamps on routed setpoints can precede the FW status event. The stock log contains a genuine FW setpoint stamped −4 ms. The plot shades ±25 ms where frame assignment based on status alone is ambiguous. Pitch is expressed along the aircraft’s FW axis in every panel.

The original PR's delayed test restarted `fw_lat_lon_control` without the `vtol` argument. The restarted controller then published to the final attitude topic, bypassing the router, so those results cannot establish the guard's behavior. The corrected tests use `fw_lat_lon_control start vtol`, as PX4’s VTOL startup script does. Both delayed pairs recovered within one 8 ms FW setpoint period of each other. Within each log, the first fresh routed and virtual setpoints share a source timestamp and all four quaternion components match. The late response follows the live controller after expiry of the 100 ms pitch hold. The observed peak rates arose before recovery in both builds. The injected outage changes the closed-loop trajectory, and the motor commands describe command rails only. The delayed logs establish routing behavior in this fixture.

A separate AI specialist independently checked the frames, sample timing, and quaternion matching against the source and CSV data. This was an internal review.

## Data and reproduction

[Download the complete evidence ZIP](sih-evidence.zip).

- [Summary CSV](evidence/summary.csv), [per-topic cadence CSV](evidence/logging-cadence.csv), [initial parameter comparison](evidence/initial-parameter-differences.csv), and [hash manifest](evidence/SHA256SUMS).
- The six original simulator ULogs and native, non-resampled CSV time series are under `evidence/`. CSV values cover −0.5 to +3.0 s at original timestamps. The ULogs retain full run duration. `timestamp_sample` is preserved where present.
- Stock ULogs identify commit `5db5c623a15dea085bfb788db587bfadfecec097`. Candidate ULogs identify the exact signed draft head `4072b456a39197a8b4c0f6a8651d528df5e9f105`. Each pair has the same initial parameters and simulator mission. The automatic barometer calibration update is in the ULogs. The nominal harness records eight seconds in FW after entry and then stops the log. The delayed harness first stops the FW lateral/longitudinal controller during front transition and starts it again after FW entry.
- The same [logger topic list](logger_topics.txt) requests interval zero for all 22 controller, navigation, attitude and actuator topics. In the nominal pair, FW virtual, TECS and both allocator-status instances recorded every 8 ms during the first three seconds of FW. Angular velocity and actuator outputs had median intervals of 4 ms and maximum intervals of 8 ms. Routed attitude setpoints had median intervals of 8 ms, with a 16 ms maximum in the PR run. [PX4 logging configuration](https://docs.px4.io/main/en/dev_log/logging#sd-card-configuration)

Build `px4_sitl_default` at the two commits above. Copy each build’s `bin` and `etc` directories into separate `stock` and `candidate` directories under the absolute path exported in `SITL_BINARY_DIRS`. The wrapper runs each case in an isolated Docker network with the PX4 build image pinned to its digest. [PX4 SIH model documentation](https://docs.px4.io/main/en/sim_sih/index)

```sh
export SITL_BINARY_DIRS=/absolute/path/to/sitl-builds
./run_sih.sh stock nominal
./run_sih.sh candidate nominal
./run_sih.sh stock delayed
./run_sih.sh candidate delayed
./run_sih.sh stock delayed stock-delayed-repeat
./run_sih.sh candidate delayed candidate-delayed-repeat
```

To regenerate the plots from the supplied logs, use Python with `numpy`, `matplotlib`, and `pyulog`, then run `python analyze.py evidence`. For new runs, copy the generated ULogs under `runs/<label>/output/fs/log/` to `evidence/<label>.ulg` in a fresh package directory. The analysis checks embedded commit IDs and refuses to treat the delay as a valid pass-through result without a matched virtual FW sample, or when the recovered quaternion differs.
