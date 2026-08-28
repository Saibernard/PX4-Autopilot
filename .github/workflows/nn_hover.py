#!/usr/bin/env python3
"""CI evidence flight: hover the x500 in headless gz with MC_NN_EN=1."""
import sys
import threading
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

# behave like a ground station: PX4's preflight checks require a live GCS link
def gcs_heartbeat():
    while True:
        m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                             mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        time.sleep(1)

threading.Thread(target=gcs_heartbeat, daemon=True).start()

# the neural config has no simulated power module: disable the supply check
m.mav.param_set_send(m.target_system, m.target_component, b"COM_POWER_COUNT",
                     0.0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)

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

# pre-flight checks need time to clear after the estimate appears: retry arming
print("arming...", flush=True)
deadline = time.time() + 90
armed = False
while time.time() < deadline and not armed:
    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
    end = time.time() + 3
    while time.time() < end:
        ack = m.recv_match(type="COMMAND_ACK", blocking=True, timeout=3)
        if ack and ack.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
            if ack.result == 0:
                armed = True
            break
    if not armed:
        time.sleep(2)
if not armed:
    fail("arm never accepted within 90 s")
print("armed", flush=True)

print("takeoff...", flush=True)
deadline = time.time() + 30
off = False
while time.time() < deadline and not off:
    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 2.5)
    end = time.time() + 3
    while time.time() < end:
        ack = m.recv_match(type="COMMAND_ACK", blocking=True, timeout=3)
        if ack and ack.command == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF:
            if ack.result == 0:
                off = True
            break
    if not off:
        time.sleep(1)
if not off:
    fail("takeoff never accepted within 30 s")

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
