import can
import struct
import time
import threading
import subprocess

# ============================================================
# Interactive ODrive CAN controller
# Type commands at the prompt. Type 'help' to see all commands.
# ============================================================

CAN_IFACE = "can0"
BITRATE = 500000

NODE_IDS = [0, 1]
IDLE = 1
CLOSED_LOOP_CONTROL = 8
VELOCITY_CONTROL = 2
INPUT_MODE_PASSTHROUGH = 1

AXIS_STATE_NAMES = {
    0: "UNDEFINED", 1: "IDLE", 2: "STARTUP_SEQUENCE",
    3: "FULL_CALIBRATION_SEQUENCE", 4: "MOTOR_CALIBRATION",
    6: "ENCODER_INDEX_SEARCH", 7: "ENCODER_OFFSET_CALIBRATION",
    8: "CLOSED_LOOP_CONTROL", 9: "LOCKIN_SPIN",
    10: "ENCODER_DIR_FIND", 11: "HOMING",
    12: "ENCODER_HALL_POLARITY_CALIBRATION",
    13: "ENCODER_HALL_PHASE_CALIBRATION",
}

bus = None
latest_heartbeats = {}   # node_id -> dict(state, active_errors, ...)
listener_running = True


def run(cmd, check=False):
    """Run a shell command, swallow errors unless check=True (used for idempotent setup steps)."""
    result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
    if check and result.returncode != 0:
        print(f"  ! command failed: {cmd}")
        print(f"    {result.stderr.strip()}")
    return result


def bring_up_can():
    """Load CAN kernel modules and bring up can0 with the right bitrate, loopback off.
    Safe to run even if it's already up - down/reload first to guarantee a clean state."""
    print("Bringing up CAN interface...")
    run(f"sudo ip link set {CAN_IFACE} down")
    run("sudo modprobe can")
    run("sudo modprobe can_raw")
    run("sudo modprobe mttcan")

    result = run(f"ip link show {CAN_IFACE}")
    if CAN_IFACE not in result.stdout:
        print(f"  ! {CAN_IFACE} did not appear after loading modules. "
              f"Check 'lsmod | grep can' and dmesg manually.")
        return False

    run(f"sudo ip link set {CAN_IFACE} type can bitrate {BITRATE} loopback off", check=True)
    up_result = run(f"sudo ip link set {CAN_IFACE} up", check=True)

    status = run(f"ip -details -statistics link show {CAN_IFACE}")
    if "ERROR-ACTIVE" in status.stdout or "state UP" in status.stdout:
        print(f"  {CAN_IFACE} is up at {BITRATE} bps, loopback off.")
        return True
    else:
        print(f"  ! {CAN_IFACE} may not have come up cleanly - check status below:")
        print(status.stdout)
        return False


def heartbeat_listener():
    """Background thread - continuously updates latest_heartbeats from CAN traffic."""
    while listener_running:
        try:
            msg = bus.recv(timeout=0.2)
        except Exception:
            continue
        if msg is None:
            continue
        node_id = msg.arbitration_id >> 5
        cmd_id = msg.arbitration_id & 0x1F
        if cmd_id == 0x01 and len(msg.data) >= 5:  # Heartbeat
            active_errors = struct.unpack_from('<I', msg.data, 0)[0]
            axis_state = msg.data[4]
            latest_heartbeats[node_id] = {
                "state": axis_state,
                "active_errors": active_errors,
                "ts": time.time(),
            }


def send_reboot(node_id):
    bus.send(can.Message(arbitration_id=(node_id << 5) | 0x16, data=[0x00], is_extended_id=False))


def send_axis_state(node_id, state):
    bus.send(can.Message(arbitration_id=(node_id << 5) | 0x07, data=struct.pack('<I', state), is_extended_id=False))


def send_controller_mode(node_id, control_mode, input_mode=INPUT_MODE_PASSTHROUGH):
    bus.send(can.Message(arbitration_id=(node_id << 5) | 0x0b, data=struct.pack('<II', control_mode, input_mode), is_extended_id=False))


def send_input_vel(node_id, vel, torque_ff=0.0):
    bus.send(can.Message(arbitration_id=(node_id << 5) | 0x0d, data=struct.pack('<ff', vel, torque_ff), is_extended_id=False))


def send_input_pos(node_id, pos, vel_ff=0.0, torque_ff=0.0):
    # pos: float32 turns, vel_ff: int16 (0.001 turns/s units), torque_ff: int16 (0.001 Nm units)
    vel_ff_i = int(vel_ff * 1000)
    torque_ff_i = int(torque_ff * 1000)
    bus.send(can.Message(arbitration_id=(node_id << 5) | 0x0c,
                          data=struct.pack('<fhh', pos, vel_ff_i, torque_ff_i),
                          is_extended_id=False))


def send_clear_errors(node_id):
    bus.send(can.Message(arbitration_id=(node_id << 5) | 0x18, data=[0x00], is_extended_id=False))


def resolve_targets(arg):
    if arg.lower() == "all":
        return NODE_IDS
    try:
        n = int(arg)
        if n in NODE_IDS:
            return [n]
    except ValueError:
        pass
    print(f"  ! Unknown node '{arg}'. Use a node id ({NODE_IDS}) or 'all'.")
    return []


def print_status(node_id):
    hb = latest_heartbeats.get(node_id)
    if hb is None:
        print(f"  node {node_id}: no heartbeat seen yet")
        return
    age = time.time() - hb["ts"]
    state_name = AXIS_STATE_NAMES.get(hb["state"], str(hb["state"]))
    err_str = "no errors" if hb["active_errors"] == 0 else f"errors=0x{hb['active_errors']:x}"
    print(f"  node {node_id}: state={state_name} ({hb['state']})  {err_str}  (last seen {age:.1f}s ago)")


HELP_TEXT = """
Commands:
  status [node|all]              Show last known heartbeat state
  reboot <node|all>               Reboot ODrive(s)
  idle <node|all>                 Set IDLE
  closed <node|all>                Enter CLOSED_LOOP_CONTROL (sets velocity mode first)
  vel <node|all> <value>          Send Set_Input_Vel
  pos <node|all> <value>          Send Set_Input_Pos
  stop <node|all>                  Shortcut for vel ... 0
  clear <node|all>                 Clear errors
  help                             Show this message
  quit / exit                      Stop listener and quit
"""


def main():
    global bus, listener_running

    if not bring_up_can():
        print("CAN interface did not come up cleanly. Fix the issue above before continuing.")
        return

    bus = can.interface.Bus(CAN_IFACE, interface="socketcan")
    listener_thread = threading.Thread(target=heartbeat_listener, daemon=True)
    listener_thread.start()

    print("ODrive CAN interactive controller. Type 'help' for commands, 'quit' to exit.")

    try:
        while True:
            try:
                line = input("odrive> ").strip()
            except EOFError:
                break
            if not line:
                continue

            parts = line.split()
            cmd = parts[0].lower()
            args = parts[1:]

            if cmd in ("quit", "exit"):
                break

            elif cmd == "help":
                print(HELP_TEXT)

            elif cmd == "status":
                targets = resolve_targets(args[0]) if args else NODE_IDS
                for n in targets:
                    print_status(n)

            elif cmd == "reboot":
                if not args:
                    print("  usage: reboot <node|all>")
                    continue
                for n in resolve_targets(args[0]):
                    print(f"  rebooting node {n}...")
                    send_reboot(n)

            elif cmd == "idle":
                if not args:
                    print("  usage: idle <node|all>")
                    continue
                for n in resolve_targets(args[0]):
                    send_axis_state(n, IDLE)
                    print(f"  node {n} -> IDLE requested")

            elif cmd == "closed":
                if not args:
                    print("  usage: closed <node|all>")
                    continue
                for n in resolve_targets(args[0]):
                    send_controller_mode(n, VELOCITY_CONTROL)
                    send_axis_state(n, CLOSED_LOOP_CONTROL)
                    print(f"  node {n} -> CLOSED_LOOP_CONTROL requested")
                time.sleep(0.4)
                for n in resolve_targets(args[0]):
                    print_status(n)

            elif cmd == "vel":
                if len(args) < 2:
                    print("  usage: vel <node|all> <value>")
                    continue
                try:
                    value = float(args[1])
                except ValueError:
                    print("  ! velocity must be a number")
                    continue
                for n in resolve_targets(args[0]):
                    send_input_vel(n, value)
                    print(f"  node {n}: input_vel = {value}")

            elif cmd == "pos":
                if len(args) < 2:
                    print("  usage: pos <node|all> <value>")
                    continue
                try:
                    value = float(args[1])
                except ValueError:
                    print("  ! position must be a number")
                    continue
                for n in resolve_targets(args[0]):
                    send_input_pos(n, value)
                    print(f"  node {n}: input_pos = {value}")

            elif cmd == "stop":
                if not args:
                    print("  usage: stop <node|all>")
                    continue
                for n in resolve_targets(args[0]):
                    send_input_vel(n, 0.0)
                    print(f"  node {n}: stopped (vel=0)")

            elif cmd == "clear":
                if not args:
                    print("  usage: clear <node|all>")
                    continue
                for n in resolve_targets(args[0]):
                    send_clear_errors(n)
                    print(f"  node {n}: errors cleared")

            else:
                print(f"  ! unknown command '{cmd}'. Type 'help' for the list.")

    finally:
        listener_running = False
        time.sleep(0.3)
        bus.shutdown()
        print("Bus shut down. Goodbye.")


if __name__ == "__main__":
    main()