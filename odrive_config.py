#!/usr/bin/env python3
"""
odrive_can_tuner.py

Interactive + bulk ODrive S1 (fw 0.6.11) CAN tuning-parameter tool, using the
CAN "SDO"-style arbitrary parameter read/write commands (RxSdo / TxSdo),
addressed by the endpoint IDs from this axis's flat_endpoints.json.

INTERACTIVE MODE (default):
    $ python3 odrive_can_tuner.py
    >>> odrv0.axis0.controller.config.vel_gain = 0.15
      node 0: wrote axis0.controller.config.vel_gain = 0.15  (readback: 0.15)
    >>> odrv0.axis0.controller.config.vel_gain
      odrv0.axis0.controller.config.vel_gain = 0.15
    >>> quit

    "odrv<N>" selects the CAN node id N. Everything after the first "." is
    matched directly against a key in flat_endpoints.json. With " = <value>"
    it writes; without, it reads and prints the current value.

BULK MODE:
    $ python3 odrive_can_tuner.py bulk --nodes 0 1 2 3 --confirm

REQUIRES: python-can  (pip install python-can)
REQUIRES: a SocketCAN interface already up, e.g.:
    sudo ip link set can0 up type can bitrate 500000

IMPORTANT CAVEATS:
  - Endpoint IDs are tied to the exact firmware build (this table matches
    fw_version 0.6.11 / crc 52326). If a node runs different firmware, its
    endpoint IDs may not line up - verify with a read on one node first.
  - The RxSdo/TxSdo byte layout below (opcode, uint16 endpoint id, reserved
    byte, 4-byte value) matches the documented ODrive CAN protocol for fw
    0.6.x but hasn't been verified against your specific hardware.
  - Writes go to RAM only unless you explicitly call save_configuration
    (bulk mode: --save flag; interactive mode: not wired up, do it via
    bulk mode once you're happy with values).
  - Do this with wheels off the ground / axes idle. Changing vel_gain /
    vel_integrator_gain while CLOSED_LOOP_CONTROL is actively driving can
    cause a sudden jump in motor behavior.
"""

import argparse
import difflib
import json
import re
import struct
import time

import can  # python-can, socketcan backend


# ---------------------------------------------------------------------------
# CAN Simple protocol command IDs (arbitration_id = (node_id << 5) | cmd_id)
# ---------------------------------------------------------------------------
CMD_RXSDO = 0x04  # host -> odrive: read or write a parameter by endpoint id
CMD_TXSDO = 0x05  # odrive -> host: response to a read request

OPCODE_READ = 0x00
OPCODE_WRITE = 0x01

SAVE_CONFIGURATION_ENDPOINT_ID = 684  # from flat_endpoints.json, "function" type

# Maps flat_endpoints.json "type" strings to struct format codes.
# Only scalar types up to 4 bytes are handled - all tuning-relevant
# parameters (gains, limits, bools) fit this.
TYPE_MAP = {
    "bool": "?",
    "uint8": "B",
    "uint16": "H",
    "uint32": "I",
    "int32": "i",
    "float": "f",
}

# Command syntax: odrv<node_id>.<dotted.param.path>[ = <value>]
# The single space before/after "=" is required, matching odrivetool-style
# commands typed by hand.
#
# The node id is optional: "odrv0.foo.bar" targets node 0 only, while
# "odrv.foo.bar" (no digits between "odrv" and ".") targets every node in
# the configured all-nodes list - handy for pushing the same value to every
# wheel in one line instead of repeating the command per node id.
CMD_RE = re.compile(
    r'^odrv(?P<node>\d*)\.(?P<path>[A-Za-z0-9_.]+)(?:\s=\s(?P<value>.+))?$'
)


def load_endpoint_map(json_path: str) -> dict:
    with open(json_path, "r") as f:
        data = json.load(f)
    return data["endpoints"]


def parse_value(raw: str, odrive_type: str):
    """Cast a raw command-line value string to the type flat_endpoints.json
    declares for this endpoint, so e.g. '15' becomes int(15) or float(15.0)
    rather than being sent as text."""
    raw = raw.strip()
    if odrive_type == "bool":
        low = raw.lower()
        if low in ("true", "1", "yes", "on"):
            return True
        if low in ("false", "0", "no", "off"):
            return False
        raise ValueError(f"'{raw}' is not a recognizable bool (try true/false or 1/0)")
    if odrive_type in ("uint8", "uint16", "uint32", "int32"):
        return int(raw, 0)  # base 0 allows hex like 0x1A if ever needed
    if odrive_type == "float":
        return float(raw)
    raise ValueError(f"unsupported type '{odrive_type}'")


class OdriveCanTuner:
    def __init__(self, endpoint_json_path: str, can_interface: str = "can0"):
        self.endpoints = load_endpoint_map(endpoint_json_path)
        self.bus = can.interface.Bus(channel=can_interface, bustype="socketcan")

    def close(self):
        self.bus.shutdown()

    def _endpoint(self, name: str) -> dict:
        if name not in self.endpoints:
            raise KeyError(f"Unknown endpoint '{name}'")
        ep = self.endpoints[name]
        if ep.get("type") not in TYPE_MAP:
            raise TypeError(
                f"Endpoint '{name}' has type '{ep.get('type')}', which this "
                "tool doesn't support (only scalar <=4-byte types)"
            )
        return ep

    def write_param(self, node_id: int, endpoint_name: str, value, confirm: bool = False):
        ep = self._endpoint(endpoint_name)
        fmt = TYPE_MAP[ep["type"]]

        payload = struct.pack("<BHB", OPCODE_WRITE, ep["id"], 0)
        payload += struct.pack("<" + fmt, value).ljust(4, b"\x00")

        arb_id = (node_id << 5) | CMD_RXSDO
        msg = can.Message(arbitration_id=arb_id, data=payload, is_extended_id=False)
        self.bus.send(msg)

        if confirm:
            time.sleep(0.02)
            readback = self.read_param(node_id, endpoint_name)
            print(f"  node {node_id}: wrote {endpoint_name} = {value}  (readback: {readback})")

    def read_param(self, node_id: int, endpoint_name: str, timeout: float = 0.5):
        ep = self._endpoint(endpoint_name)
        fmt = TYPE_MAP[ep["type"]]

        payload = struct.pack("<BHB", OPCODE_READ, ep["id"], 0) + b"\x00" * 4
        arb_id = (node_id << 5) | CMD_RXSDO
        msg = can.Message(arbitration_id=arb_id, data=payload, is_extended_id=False)
        self.bus.send(msg)

        expected_id = (node_id << 5) | CMD_TXSDO
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            resp = self.bus.recv(timeout=remaining)
            if resp is None:
                break
            if resp.arbitration_id != expected_id or len(resp.data) < 8:
                continue
            resp_ep_id = struct.unpack_from("<H", resp.data, 1)[0]
            if resp_ep_id != ep["id"]:
                continue
            return struct.unpack_from("<" + fmt, resp.data, 4)[0]
        return None

    def apply_tuning(self, node_id: int, params: dict, confirm: bool = False):
        for name, value in params.items():
            self.write_param(node_id, name, value, confirm=confirm)

    def apply_tuning_all(self, node_ids, params: dict, confirm: bool = False, per_node_overrides=None):
        per_node_overrides = per_node_overrides or {}
        for node_id in node_ids:
            print(f"Tuning node {node_id} ...")
            self.apply_tuning(node_id, params, confirm=confirm)
            if node_id in per_node_overrides:
                print(f"  applying overrides for node {node_id}: {per_node_overrides[node_id]}")
                self.apply_tuning(node_id, per_node_overrides[node_id], confirm=confirm)

    def save_configuration(self, node_id: int):
        arb_id = (node_id << 5) | CMD_RXSDO
        payload = struct.pack("<BHB", OPCODE_WRITE, SAVE_CONFIGURATION_ENDPOINT_ID, 0) + b"\x00" * 4
        self.bus.send(can.Message(arbitration_id=arb_id, data=payload, is_extended_id=False))


# ---------------------------------------------------------------------------
# Interactive REPL
# ---------------------------------------------------------------------------
def handle_command(tuner: OdriveCanTuner, line: str, all_nodes: list):
    # save            -> save_configuration on every node in all_nodes
    # save<N>         -> save_configuration on node N only
    save_match = re.match(r'^save(?P<node>\d*)$', line.strip().lower())
    if save_match:
        node_str = save_match.group("node")
        targets = all_nodes if node_str == "" else [int(node_str)]
        for node_id in targets:
            print(f"  saving configuration on node {node_id} (this will reboot the axis) ...")
            tuner.save_configuration(node_id)
            time.sleep(0.1)
        return

    m = CMD_RE.match(line)
    if not m:
        print("  could not parse - expected: odrv<N>.<param.path> [ = <value> ]")
        return

    node_str = m.group("node")
    path = m.group("path")
    raw_value = m.group("value")

    # No digits after "odrv" -> broadcast to every configured node instead
    # of a single one.
    if node_str == "":
        target_nodes = all_nodes
        print(f"  (no node id given - applying to all nodes: {target_nodes})")
    else:
        target_nodes = [int(node_str)]

    if path not in tuner.endpoints:
        suggestion = difflib.get_close_matches(path, tuner.endpoints.keys(), n=1)
        hint = f" - did you mean '{suggestion[0]}'?" if suggestion else ""
        print(f"  unknown parameter '{path}'{hint}")
        return

    ep = tuner.endpoints[path]

    if raw_value is None:
        # No "=" present -> read and print current value from each target node.
        if ep.get("type") not in TYPE_MAP:
            print(f"  '{path}' has type '{ep.get('type')}' (function/unsupported for direct read here)")
            return
        for node_id in target_nodes:
            value = tuner.read_param(node_id, path)
            print(f"  odrv{node_id}.{path} = {value}")
        return

    # "=" present -> write to each target node.
    if ep.get("access") == "r":
        print(f"  '{path}' is read-only, cannot set")
        return
    if ep.get("type") not in TYPE_MAP:
        print(f"  '{path}' has unsupported type '{ep.get('type')}' for this tool")
        return

    try:
        value = parse_value(raw_value, ep["type"])
    except ValueError as e:
        print(f"  could not parse value '{raw_value}' as {ep['type']}: {e}")
        return

    for node_id in target_nodes:
        tuner.write_param(node_id, path, value, confirm=True)


def run_interactive(tuner: OdriveCanTuner, all_nodes: list):
    print("ODrive CAN interactive tuner.")
    print("  odrv0.axis0.controller.config.vel_gain = 0.15   -> writes node 0 only")
    print(f"  odrv.axis0.controller.config.vel_gain = 0.15    -> writes all nodes {all_nodes}")
    print("  odrv0.axis0.controller.config.vel_gain          -> reads node 0 only")
    print("  save             -> persist ALL nodes' current RAM config to flash (reboots them)")
    print("  save0            -> persist node 0 only to flash (reboots it)")
    print("Type 'quit' or 'exit' to leave.\n")
    while True:
        try:
            line = input(">>> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            break
        if not line:
            continue
        if line.lower() in ("quit", "exit"):
            break
        try:
            handle_command(tuner, line, all_nodes)
        except Exception as e:
            print(f"  error: {e}")


# ---------------------------------------------------------------------------
# Bulk mode - edit these before running `bulk`.
# ---------------------------------------------------------------------------
BASE_TUNING = {
    "axis0.controller.config.vel_gain": 0.15,
    "axis0.controller.config.vel_integrator_gain": 0.30,
    "axis0.controller.config.vel_limit": 5.0,
    "axis0.config.motor.current_soft_max": 20.0,
    "axis0.config.motor.current_hard_max": 30.0,
}

PER_NODE_OVERRIDES: dict = {
    # 1: {"axis0.controller.config.vel_integrator_gain": 0.40},
}


def main():
    parser = argparse.ArgumentParser(description="ODrive S1 CAN tuning tool")
    parser.add_argument("--endpoints", default="flat_endpoints.json", help="Path to flat_endpoints.json")
    parser.add_argument("--can-interface", default="can0")
    parser.add_argument(
        "--all-nodes", type=int, nargs="+", default=[0, 1, 2, 3],
        help="Node IDs targeted in interactive mode when a command omits the "
             "node number, e.g. 'odrv.axis0.controller.config.vel_gain = 0.15' "
             "(default: 0 1 2 3)",
    )

    subparsers = parser.add_subparsers(dest="mode")

    bulk_parser = subparsers.add_parser("bulk", help="Apply BASE_TUNING to multiple/all nodes")
    bulk_parser.add_argument("--nodes", type=int, nargs="+", default=[0, 1, 2, 3])
    bulk_parser.add_argument("--node", type=int, default=None)
    bulk_parser.add_argument("--confirm", action="store_true")
    bulk_parser.add_argument("--save", action="store_true")

    subparsers.add_parser("interactive", help="odrv<N>.<param> [ = value ] REPL (default)")

    args = parser.parse_args()

    tuner = OdriveCanTuner(args.endpoints, can_interface=args.can_interface)
    try:
        if args.mode == "bulk":
            if args.node is not None:
                print(f"Updating single node {args.node}")
                tuner.apply_tuning(args.node, BASE_TUNING, confirm=args.confirm)
                overrides = PER_NODE_OVERRIDES.get(args.node)
                if overrides:
                    tuner.apply_tuning(args.node, overrides, confirm=args.confirm)
                targets = [args.node]
            else:
                tuner.apply_tuning_all(
                    args.nodes, BASE_TUNING, confirm=args.confirm,
                    per_node_overrides=PER_NODE_OVERRIDES,
                )
                targets = args.nodes

            if args.save:
                for node_id in targets:
                    print(f"Saving configuration on node {node_id} ...")
                    tuner.save_configuration(node_id)
                    time.sleep(0.1)
        else:
            run_interactive(tuner, args.all_nodes)
    finally:
        tuner.close()


if __name__ == "__main__":
    main()