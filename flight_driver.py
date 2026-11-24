"""Exercise manual front/back, aborted front and loss of FW TECS publication in SITL."""

import json
import os
from pathlib import Path
import subprocess
import time

from pymavlink import mavutil

MAV = mavutil.mavlink
OUTPUT = Path("/output")


class SimLink:
    def __init__(self):
        self.link = mavutil.mavlink_connection("udpin:127.0.0.1:14540", source_system=245)
        heartbeat = self.link.wait_heartbeat(timeout=35)
        if heartbeat is None:
            raise TimeoutError("No simulator heartbeat")
        self.targets = heartbeat.get_srcSystem(), heartbeat.get_srcComponent()
        while True:
            self.position = self.link.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=15)
            if self.position is None:
                raise TimeoutError("No simulator global position")
            if self.position.lat != 0:
                break
        self.vtol = None
        self.mode = None
        self.command(MAV.MAV_CMD_SET_MESSAGE_INTERVAL, 245, 20000)
        self.command(MAV.MAV_CMD_SET_MESSAGE_INTERVAL, 33, 20000)

    def command(self, command, *args):
        self.link.mav.command_long_send(*self.targets, command, 0, *(args + (0.,) * (7-len(args))))

    def arm(self, manual=False):
        for _ in range(20):
            self.command(MAV.MAV_CMD_DO_SET_MODE, 1, 4, 3 if manual else 4)
            self.command(MAV.MAV_CMD_COMPONENT_ARM_DISARM, 1)
            deadline = time.monotonic()+1.6
            while time.monotonic() < deadline:
                if manual:
                    self.manual(value=0)
                msg = self.link.recv_match(type=["COMMAND_ACK", "STATUSTEXT"], blocking=True, timeout=.10)
                if msg is None:
                    continue
                if msg.get_type() == "STATUSTEXT":
                    print(msg.text, flush=True)
                elif msg.command == MAV.MAV_CMD_COMPONENT_ARM_DISARM and msg.result == MAV.MAV_RESULT_ACCEPTED:
                    return
        raise RuntimeError("Normal simulator arming was not accepted")

    def position_state(self, events):
        msg = self.link.recv_match(blocking=True, timeout=.05)
        if msg is None:
            return
        kind = msg.get_type()
        if kind == "GLOBAL_POSITION_INT":
            self.position = msg
        elif kind == "EXTENDED_SYS_STATE" and msg.vtol_state != self.vtol:
            self.vtol = msg.vtol_state
            record(events, "vtol_state", ms=self.position.time_boot_ms, state=self.vtol)
        elif kind == "HEARTBEAT":
            self.mode = (msg.custom_mode >> 16) & 255
        elif kind == "COMMAND_ACK" and msg.command in (
            MAV.MAV_CMD_DO_SET_MODE, MAV.MAV_CMD_DO_VTOL_TRANSITION, MAV.MAV_CMD_NAV_TAKEOFF
        ):
            record(events, "command_ack", command=msg.command, result=msg.result)
        elif kind == "STATUSTEXT":
            print(msg.text, flush=True)

    def upload_mission(self):
        lat, lon = self.position.lat, self.position.lon
        plan = [(MAV.MAV_CMD_NAV_VTOL_TAKEOFF, lat, lon + 280000, 50.),
                (MAV.MAV_CMD_NAV_WAYPOINT, lat, lon + 360000, 50.),
                (MAV.MAV_CMD_NAV_VTOL_LAND, lat, lon + 440000, 0.)]
        (OUTPUT/"mission.json").write_text(json.dumps(plan) + "\n")
        self.link.mav.mission_count_send(*self.targets, len(plan), 0)
        while True:
            msg = self.link.recv_match(type=["MISSION_REQUEST_INT", "MISSION_REQUEST", "MISSION_ACK"],
                                       blocking=True, timeout=8)
            if msg is None:
                raise TimeoutError("Simulator mission upload stalled")
            if msg.get_type() == "MISSION_ACK":
                if msg.type != MAV.MAV_MISSION_ACCEPTED:
                    raise RuntimeError(f"Simulator mission rejected {msg.type}")
                break
            cmd, lat, lon, height = plan[msg.seq]
            self.link.mav.mission_item_int_send(*self.targets, msg.seq, MAV.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                                               cmd, int(msg.seq == 0), 1, 0, 0, 0, float("nan"), lat, lon, height, 0)

    def manual(self, value):
        self.link.mav.manual_control_send(self.targets[0], 0, 0, value, 0, 0)


def record(events, event, **kwargs):
    events.write(json.dumps({"event": event, **kwargs}) + "\n")
    events.flush()


def fw_controller(command, events, now):
    args = ["/clean/bin/px4-fw_lat_lon_control", command]
    if command == "start":
        # Match rc.vtol_apps so FW goes through the virtual setpoint and VTOL router.
        args.append("vtol")
    process = subprocess.run(args, cwd=OUTPUT,
                             capture_output=True, text=True, timeout=4)
    record(events, "fw_control_" + command, ms=now, exit=process.returncode, args=args[1:])
    if process.returncode:
        raise RuntimeError(process.stderr or process.stdout)


def run():
    if os.environ.get("SIH_SIMULATION_ONLY") != "1":
        raise RuntimeError("Use the network-isolated SIH launcher")
    mission = os.environ["PB_SIM_MISSION"]
    if mission not in ("abort", "stale", "stale_longer", "stabilized", "manual_to_altitude", "generic"):
        raise ValueError(mission)
    flight = SimLink()
    manual_case = mission in ("stabilized", "manual_to_altitude")
    delayed_fw = mission in ("stale", "stale_longer")
    restart_delay_ms = int(os.environ.get("SIH_FW_RESTART_DELAY_MS", "550"))
    if manual_case:
        flight.command(MAV.MAV_CMD_DO_SET_MODE, 1, 4, 3)
        flight.arm(manual=True)
        flight.command(MAV.MAV_CMD_NAV_TAKEOFF, -1, 0, 0, float("nan"), float("nan"), float("nan"),
                       flight.position.alt * .001 + 55.)
    else:
        flight.upload_mission()
        flight.command(MAV.MAV_CMD_DO_SET_MODE, 1, 4, 4)
        flight.arm()

    started = time.monotonic()
    manual_at = mode_request = front_at = fw_at = action_at = returned_at = None
    fw_stopped = altitude_requested = False
    last_manual = 0.
    with (OUTPUT/"events.jsonl").open("w") as events:
        while time.monotonic() - started < 95:
            flight.position_state(events)
            now = flight.position.time_boot_ms
            state = flight.vtol
            if manual_case:
                if time.monotonic() - last_manual > .025:
                    flight.manual(value=620 if flight.mode == 7 else 0)
                    last_manual = time.monotonic()
            if manual_case and (mode_request is not None or flight.position.relative_alt > 44000):
                if mode_request is None:
                    mode_request = now
                if flight.mode != 7 and now - mode_request > 250 and manual_at is None:
                    flight.command(MAV.MAV_CMD_DO_SET_MODE, 1, 7, 0)
                    mode_request = now
                if flight.mode == 7 and manual_at is None:
                    manual_at = now
                    flight.command(MAV.MAV_CMD_DO_VTOL_TRANSITION, MAV.MAV_VTOL_STATE_FW)
                    record(events, "stabilized_front_requested", ms=now)

            if state == MAV.MAV_VTOL_STATE_TRANSITION_TO_FW and front_at is None:
                front_at = now
                if delayed_fw:
                    fw_controller("stop", events, now)
                    fw_stopped = True
            if state == MAV.MAV_VTOL_STATE_FW and fw_at is None:
                fw_at = now
                if mission == "abort":
                    raise RuntimeError("Aborted front unexpectedly reached FW")

            if mission == "generic" and fw_at is not None and now-fw_at >= 8000:
                record(events, "completed", ms=now)
                return

            if mission == "abort" and front_at is not None and action_at is None and now-front_at >= 1000:
                flight.command(MAV.MAV_CMD_DO_SET_MODE, 1, 4, 3)
                flight.command(MAV.MAV_CMD_DO_VTOL_TRANSITION, MAV.MAV_VTOL_STATE_MC)
                action_at = now
                record(events, "abort_requested", ms=now)

            if delayed_fw and fw_stopped and fw_at is not None and now-fw_at >= restart_delay_ms:
                fw_controller("start", events, now)
                fw_stopped = False
                action_at = now

            if mission == "manual_to_altitude" and fw_at is not None and not altitude_requested and now-fw_at >= 200:
                flight.command(MAV.MAV_CMD_DO_SET_MODE, 1, 4, 3)
                altitude_requested = True
                record(events, "altitude_mode_requested", ms=now)

            if manual_case and fw_at is not None and action_at is None and now-fw_at >= 1500:
                flight.command(MAV.MAV_CMD_DO_VTOL_TRANSITION, MAV.MAV_VTOL_STATE_MC)
                action_at = now
                record(events, "stabilized_back_requested", ms=now)

            if action_at is not None:
                if delayed_fw and state == MAV.MAV_VTOL_STATE_FW and now-action_at >= 3500:
                    record(events, "completed", ms=now)
                    return
                if not delayed_fw and state == MAV.MAV_VTOL_STATE_MC:
                    returned_at = now if returned_at is None else returned_at
                    if now - returned_at >= 900:
                        record(events, "completed", ms=now)
                        return
        raise TimeoutError(f"Scenario {mission} did not finish, front={front_at} fw={fw_at} action={action_at}")


if __name__ == "__main__":
    run()
