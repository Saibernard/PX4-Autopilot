#!/usr/bin/env python3
"""CI evidence flight: hover the x500 in headless gz with MC_NN_EN=1."""
import sys
import time
from pymavlink import mavutil

def fail(msg):
    print(f"FLIGHT FAIL: {msg}", flush=True)
    sys.exit(1)

m = mavutil.mavlink_connection("udpin:0.0.0.0:14540")
print("waiting for heartbeat...", flush=True)
if not m.wait_heartbeat(timeout=300):
    fail("no heartbeat within 300 s")
print(f"heartbeat from system {m.target_system}", flush=True)

# enable the neural controller before flight
m.mav.param_set_send(m.target_system, m.target_component, b"MC_NN_EN",
                     1.0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
deadline = time.time() + 30
confirmed = False
while time.time() < deadline:
    msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=5)
    if msg and msg.param_id == "MC_NN_EN":
        print(f"MC_NN_EN = {msg.param_value}", flush=True)
        confirmed = msg.param_value == 1.0
        break
if not confirmed:
    fail("MC_NN_EN not confirmed")

# wait for a sane EKF position estimate
print("waiting for position estimate...", flush=True)
deadline = time.time() + 120
ok = False
while time.time() < deadline:
    msg = m.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=5)
    if msg and msg.lat != 0:
        ok = True
        break
if not ok:
    fail("no position estimate")

print("arming...", flush=True)
m.mav.command_long_send(m.target_system, m.target_component,
                        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
ack = m.recv_match(type="COMMAND_ACK", blocking=True, timeout=10)
if not ack or ack.result != 0:
    fail(f"arm rejected: {ack}")

print("takeoff...", flush=True)
m.mav.command_long_send(m.target_system, m.target_component,
                        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 2.5)
ack = m.recv_match(type="COMMAND_ACK", blocking=True, timeout=10)
if not ack or ack.result != 0:
    fail(f"takeoff rejected: {ack}")

# require a stable hover: altitude within 1 m of 2.5 m for 30 continuous seconds
print("watching hover...", flush=True)
deadline = time.time() + 180
stable_since = None
while time.time() < deadline:
    msg = m.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=5)
    if msg is None:
        continue
    alt = -msg.z
    near = abs(alt - 2.5) < 1.0
    now = time.time()
    if near:
        if stable_since is None:
            stable_since = now
            print(f"reached hover band at alt {alt:.2f} m", flush=True)
        elif now - stable_since > 30:
            print(f"hover held 30 s at alt {alt:.2f} m", flush=True)
            break
    else:
        stable_since = None
else:
    fail("hover never held for 30 s")

print("landing...", flush=True)
m.mav.command_long_send(m.target_system, m.target_component,
                        mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0)
deadline = time.time() + 120
while time.time() < deadline:
    hb = m.recv_match(type="HEARTBEAT", blocking=True, timeout=5)
    if hb and not (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
        print("landed and disarmed", flush=True)
        print("FLIGHT PASS", flush=True)
        sys.exit(0)
fail("never disarmed after land")
