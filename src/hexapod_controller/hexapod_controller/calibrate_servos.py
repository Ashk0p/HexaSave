#!/usr/bin/env python3
import serial
import time
import json
import os

# Restored Context: GPIO UART and Baud Rate
PORT = '/dev/ttyAMA0'
BAUD = 115200
OUTPUT_FILE = os.path.expanduser('~/hexapod_ws/src/hexapod_controller/config/servo_offsets.json')

# Restored Context: Broken Channel 12 is physically remapped to 19
VALID_CHANNELS = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 19, 13, 14, 15, 16, 17, 18]

# Initialize offsets only for valid channels
offsets = {str(ch): 0 for ch in VALID_CHANNELS}

# Load existing calibration if present
if os.path.exists(OUTPUT_FILE):
    try:
        with open(OUTPUT_FILE, 'r') as f:
            loaded_offsets = json.load(f)
            offsets.update(loaded_offsets)
            print(f"Loaded existing calibration from {OUTPUT_FILE}")
    except Exception as e:
        print(f"Notice: Could not load calibration file ({e}). Using defaults.")

def send_servo_cmd(ser, channel, pwm_val, move_time=100):
    """Sends Torobot ASCII command for 1-based channel: #<ch>P<pwm>T<time>\r\n"""
    cmd = f"#{channel}P{pwm_val}T{move_time}\r\n"
    ser.write(cmd.encode('utf-8'))

def center_all_servos(ser):
    """Commands all valid channels to baseline 1500us + offsets."""
    cmd_str = ""
    for ch in VALID_CHANNELS:
        pwm = 1500 + offsets[str(ch)]
        cmd_str += f"#{ch}P{pwm}"
    cmd_str += "T1000\r\n"
    ser.write(cmd_str.encode('utf-8'))
    print("All active channels set to neutral (1500us + saved offsets).")

def save_calibration():
    os.makedirs(os.path.dirname(OUTPUT_FILE), exist_ok=True)
    with open(OUTPUT_FILE, 'w') as f:
        json.dump(offsets, f, indent=4)
    print(f"Offsets successfully saved to {OUTPUT_FILE}")

def main():
    try:
        # Changed timeout to match your new script logic
        ser = serial.Serial(PORT, BAUD, timeout=1)
        time.sleep(1)
        print(f"Connected to {PORT} at {BAUD} baud.")
    except Exception as e:
        print(f"Error opening port {PORT}: {e}")
        return

    center_all_servos(ser)

    active_ch = 1
    print("\n--- Interactive Servo Calibration ---")
    print(f"Active Channels: {VALID_CHANNELS}")
    print("Controls:")
    print("  <channel_num> : Select active channel (e.g., 1, 19, 15)")
    print("  w/s           : Adjust active offset +10us / -10us")
    print("  W/S           : Adjust active offset +50us / -50us")
    print("  r             : Reset active offset to 0")
    print("  c             : Recenter all servos")
    print("  save          : Save offset configuration to disk")
    print("  q             : Quit calibration")
    print("----------------------------------------------------\n")

    while True:
        current_pwm = 1500 + offsets[str(active_ch)]
        cmd = input(f"[Channel {active_ch} | Offset: {offsets[str(active_ch)]:+d}us | PWM: {current_pwm}us] > ").strip()

        if not cmd:
            continue
        elif cmd.isdigit() and int(cmd) in VALID_CHANNELS:
            active_ch = int(cmd)
            print(f"Switched to Channel {active_ch}")
        elif cmd.isdigit() and int(cmd) not in VALID_CHANNELS:
            print(f"Invalid! Channel {cmd} is not in the active list (Remember: 12 is mapped to 19).")
            continue
        elif cmd == 'w':
            offsets[str(active_ch)] += 10
        elif cmd == 's':
            offsets[str(active_ch)] -= 10
        elif cmd == 'W':
            offsets[str(active_ch)] += 50
        elif cmd == 'S':
            offsets[str(active_ch)] -= 50
        elif cmd == 'r':
            offsets[str(active_ch)] = 0
        elif cmd == 'c':
            center_all_servos(ser)
            continue
        elif cmd == 'save':
            save_calibration()
            continue
        elif cmd == 'q':
            print("Exiting calibration...")
            break
        else:
            print("Invalid command. Enter a valid channel number or a hotkey.")
            continue

        # Send command for the currently active channel
        new_pwm = 1500 + offsets[str(active_ch)]
        send_servo_cmd(ser, active_ch, new_pwm, move_time=100)

    ser.close()

if __name__ == '__main__':
    main()
