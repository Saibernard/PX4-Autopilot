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
time.sleep(0.2)

# log the debug profile too: the neural_control topic lives there
m.mav.param_set_send(m.target_system, m.target_component, b"SDLOG_PROFILE",
                     33.0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)

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

# wait until airborne under the classic takeoff
print("waiting for liftoff...", flush=True)
deadline = time.time() + 60
while time.time() < deadline:
    msg = m.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=5)
    if msg and -msg.z > 1.0:
        print(f"airborne at {-msg.z:.2f} m", flush=True)
        break
else:
    fail("never lifted off")

# switch into the neural controller's external flight mode
# (registered as mode 23 = EXTERNAL1: custom main_mode 4, sub_mode 11)
print("switching to the neural mode...", flush=True)
deadline = time.time() + 30
in_nn_mode = False
while time.time() < deadline and not in_nn_mode:
    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, 1, 4, 11, 0, 0, 0, 0)
    end = time.time() + 3
    while time.time() < end:
        hb = m.recv_match(type="HEARTBEAT", blocking=True, timeout=3)
        if hb and hb.type != mavutil.mavlink.MAV_TYPE_GCS:
            main_mode = (hb.custom_mode >> 16) & 0xFF
            sub_mode = (hb.custom_mode >> 24) & 0xFF
            if main_mode == 4 and sub_mode == 11:
                in_nn_mode = True
                break
if not in_nn_mode:
    fail("neural mode never engaged")
print("neural mode engaged", flush=True)

# judge the NN hover on its own plateau: stay airborne for 45 s and hold a
# steady altitude over the last 30 s, wherever that plateau is
print("watching neural hover...", flush=True)
alts = []
start = time.time()
while time.time() - start < 45:
    msg = m.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=5)
    if msg is None:
        continue
    alt = -msg.z
    if alt < 0.5:
        fail(f"lost altitude in neural mode: {alt:.2f} m")
    alts.append((time.time(), alt))
tail = [a for ts, a in alts if ts > time.time() - 30]
mean = sum(tail) / len(tail)
var = sum((a - mean) ** 2 for a in tail) / len(tail)
std = var ** 0.5
print(f"neural hover: mean {mean:.2f} m, std {std:.3f} m over {len(tail)} samples", flush=True)
if std > 0.3:
    fail(f"neural hover unstable: std {std:.3f} m")
print("neural hover stable", flush=True)

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
