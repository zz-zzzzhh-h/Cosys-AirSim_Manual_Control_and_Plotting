"""
Author:      Zhu Zihua
Date:        2025-12-07
Version:     5.0
Description: Code for manual control of leader drone.
"""


#-----Imports------------

import time
import math
import cosysairsim as airsim
import ctypes


#-----Constants----------

# Name of the Drone
LEADER_NAME    = "Leader"

# Control period (increase to reduce command jitter)
DT = 0.05  # 20 Hz

# Leader max speed / angular speed
MAX_LEADER_SPEED_XY = 4.0   # m/s
MAX_LEADER_SPEED_Z  = 2.0   # m/s
YAW_RATE_DEG        = 90.0  # deg/s (max rate)

# Acceleration / Deceleration (hold to accelerate, release to slow down)
ACCEL_XY   = 3.0   # m/s^2
DECEL_XY   = 3.5   # m/s^2
ACCEL_Z    = 2.0   # m/s^2
DECEL_Z    = 2.5   # m/s^2
ACCEL_YAW  = 180.0 # deg/s^2
DECEL_YAW  = 220.0 # deg/s^2

# Smoothing (small amount to remove minor jitter)
SMOOTH_ALPHA        = 0.2  # 0~1, lower = smoother
SLEW_XY             = 3.0   # m/s^2
SLEW_Z              = 2.0   # m/s^2
SLEW_YAW            = 180.0 # deg/s^2

# WinAPI keyboard state
user32 = ctypes.windll.user32
VK_ESCAPE = 0x1B  # ESC


#-----Keyboard Input Functions-----

def is_pressed_char(ch: str) -> bool:
    """Check if a character key is pressed (case insensitive)"""
    vk = ord(ch.upper())
    return (user32.GetAsyncKeyState(vk) & 0x8000) != 0

def is_pressed_vkey(vk: int) -> bool:
    """Check if a virtual key is pressed"""
    return (user32.GetAsyncKeyState(vk) & 0x8000) != 0


#-----Utility Functions-----

def clamp(x, min_val, max_val):
    """Clamp x to the range [min_val, max_val]"""
    return max(min(x, max_val), min_val)

def quat_to_yaw(q):
    """
    Convert Cosys-AirSim's Quaternionr to yaw (radians)
    q: airsim.Quaternionr with fields x_val, y_val, z_val, w_val
    """
    w = q.w_val
    x = q.x_val
    y = q.y_val
    z = q.z_val

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return yaw

def slew_limit(target, current, max_rate, dt):
    """Simple slew-rate limiter"""
    delta = clamp(target - current, -max_rate * dt, max_rate * dt)
    return current + delta

def low_pass_filter(target, current, alpha):
    """Simple low-pass filter"""
    return current + alpha * (target - current)


def enable_api_control(client, vehicle_name):
    """Enable API control for a given vehicle"""
    client.enableApiControl(True, vehicle_name)
    client.armDisarm(True, vehicle_name)

def print_menu():
    """Print control menu"""
    print("\nControl Menu:")
    print("W/S: Forward/Backward")
    print("A/D: Left/Right")
    print("U/I: Up/Down")
    print("J/L: Yaw Left/Right")
    print("K:   Return to zero yaw rate")
    print("P:   Land")
    print("ESC: Exit\n")


#-----Main Function-----

def main():
    # Connect to Cosys-AirSim
    client = airsim.MultirotorClient()
    client.confirmConnection()
    print("\nConnected to Cosys-AirSim.")

    # Enable API control and arm drones
    enable_api_control(client, LEADER_NAME)
    print(f"{LEADER_NAME}: API control enabled & armed.")
    print("\nLEADER ARMED.")

    # Print control menu
    print_menu()

    # Takeoff
    print("Taking off LEADER drone...")
    client.takeoffAsync(vehicle_name=LEADER_NAME).join()
    print("LEADER Takeoff done.")
    has_taken_off = True
    running = True

    # Smoothed command states
    sm_vx = 0.0
    sm_vy = 0.0
    sm_vz = 0.0
    sm_yaw_rate = 0.0

    # Integrating target states (accelerate while pressed)
    tgt_vx = 0.0
    tgt_vy = 0.0
    tgt_vz = 0.0
    tgt_yaw_rate = 0.0

    # Control loop
    try:
        while running:
            t0 = time.time()
            # -------- Exit check --------
            if is_pressed_vkey(VK_ESCAPE):
                running = False
                break

            # ------- Accel/decel based on key press (hold to accelerate) --------
            # X forward/back
            if is_pressed_char('W') and not is_pressed_char('S'):
                tgt_vx = clamp(tgt_vx + ACCEL_XY * DT, -MAX_LEADER_SPEED_XY, MAX_LEADER_SPEED_XY)
            elif is_pressed_char('S') and not is_pressed_char('W'):
                tgt_vx = clamp(tgt_vx - ACCEL_XY * DT, -MAX_LEADER_SPEED_XY, MAX_LEADER_SPEED_XY)
            else:
                # decelerate toward 0
                if tgt_vx > 0:
                    tgt_vx = max(0.0, tgt_vx - DECEL_XY * DT)
                elif tgt_vx < 0:
                    tgt_vx = min(0.0, tgt_vx + DECEL_XY * DT)

            # Y left/right (body frame)
            if is_pressed_char('D') and not is_pressed_char('A'):
                tgt_vy = clamp(tgt_vy + ACCEL_XY * DT, -MAX_LEADER_SPEED_XY, MAX_LEADER_SPEED_XY)
            elif is_pressed_char('A') and not is_pressed_char('D'):
                tgt_vy = clamp(tgt_vy - ACCEL_XY * DT, -MAX_LEADER_SPEED_XY, MAX_LEADER_SPEED_XY)
            else:
                if tgt_vy > 0:
                    tgt_vy = max(0.0, tgt_vy - DECEL_XY * DT)
                elif tgt_vy < 0:
                    tgt_vy = min(0.0, tgt_vy + DECEL_XY * DT)

            # Z up/down (NED: z positive down)
            if is_pressed_char('I') and not is_pressed_char('U'):
                tgt_vz = clamp(tgt_vz + ACCEL_Z * DT, -MAX_LEADER_SPEED_Z, MAX_LEADER_SPEED_Z)
            elif is_pressed_char('U') and not is_pressed_char('I'):
                tgt_vz = clamp(tgt_vz - ACCEL_Z * DT, -MAX_LEADER_SPEED_Z, MAX_LEADER_SPEED_Z)
            else:
                if tgt_vz > 0:
                    tgt_vz = max(0.0, tgt_vz - DECEL_Z * DT)
                elif tgt_vz < 0:
                    tgt_vz = min(0.0, tgt_vz + DECEL_Z * DT)

            # Yaw rate (deg/s)
            if is_pressed_char('L') and not is_pressed_char('J'):
                tgt_yaw_rate = clamp(tgt_yaw_rate + ACCEL_YAW * DT, -YAW_RATE_DEG, YAW_RATE_DEG)
            elif is_pressed_char('J') and not is_pressed_char('L'):
                tgt_yaw_rate = clamp(tgt_yaw_rate - ACCEL_YAW * DT, -YAW_RATE_DEG, YAW_RATE_DEG)
            else:
                if tgt_yaw_rate > 0:
                    tgt_yaw_rate = max(0.0, tgt_yaw_rate - DECEL_YAW * DT)
                elif tgt_yaw_rate < 0:
                    tgt_yaw_rate = min(0.0, tgt_yaw_rate + DECEL_YAW * DT)

            # K: Return to zero yaw rate immediately
            if is_pressed_char('K'):
                tgt_yaw_rate = 0.0

            # P: Land
            if is_pressed_char('P'):
                if has_taken_off:
                    print("Landing LEADER drone...")
                    client.landAsync(vehicle_name=LEADER_NAME).join()
                    has_taken_off = False
                    print("LEADER Landing done.")

            # ---- Small smoothing to reduce jitter ----
            sm_vx = low_pass_filter(tgt_vx, sm_vx, SMOOTH_ALPHA)
            sm_vy = low_pass_filter(tgt_vy, sm_vy, SMOOTH_ALPHA)
            sm_vz = low_pass_filter(tgt_vz, sm_vz, SMOOTH_ALPHA)
            sm_yaw_rate = low_pass_filter(tgt_yaw_rate, sm_yaw_rate, SMOOTH_ALPHA)

            # ---- Send one combined motion command ----
            yaw_mode = airsim.YawMode(is_rate=True, yaw_or_rate=sm_yaw_rate)
            client.moveByVelocityBodyFrameAsync(
                sm_vx, sm_vy, sm_vz, DT,
                drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
                yaw_mode=yaw_mode,
                vehicle_name=LEADER_NAME
            ).join()

            # Keep loop rate stable
            elapsed = time.time() - t0
            sleep_time = max(0.0, DT - elapsed)
            time.sleep(sleep_time)

    finally:
        print("\nStopping, landing LEADER drone...")
        try:
            client.hoverAsync(vehicle_name=LEADER_NAME)

            time.sleep(0.5)
            client.landAsync(vehicle_name=LEADER_NAME).join()
            print("LEADER drone landed.")

        except Exception as e:
            print("Error while landing:", e)
            # If server closed, just attempt to disarm/disable below

        try:
            client.armDisarm(False, vehicle_name=LEADER_NAME)
            client.enableApiControl(False, vehicle_name=LEADER_NAME)
        except Exception:
            pass

if __name__ == "__main__":
    main()
