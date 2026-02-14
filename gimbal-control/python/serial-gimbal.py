import serial
import time

# =====================================================
# CONFIGURATION
# =====================================================
SERIAL_PORT = "COM6"  # Change to your port (e.g., "COM3" on Windows)
BAUD_RATE = 115200

# Toggle parameters
ANGLE_A = 6              # First position (radians)
ANGLE_B = 3              # Second position (radians)
DELAY = 2.0              # Delay between commands (seconds)
NUM_LOOPS = 0            # Number of loops (0 = infinite)

# =====================================================
# MAIN
# =====================================================
def send_command(ser, cmd):
    full_cmd = cmd + "\n"
    ser.write(full_cmd.encode())
    print(f"Sent: {cmd}")
    time.sleep(0.05)
    while ser.in_waiting:
        print(f"  <- {ser.readline().decode().strip()}")


def main():
    print(f"Connecting to {SERIAL_PORT} at {BAUD_RATE} baud...")
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    time.sleep(2)  # Wait for microcontroller to init

    # Flush startup messages
    while ser.in_waiting:
        print(ser.readline().decode().strip())

    print(f"\n=== Pan Toggle ===")
    print(f"Toggling between B{ANGLE_A} and B{ANGLE_B}")
    print(f"Delay: {DELAY}s  |  Loops: {'infinite' if NUM_LOOPS == 0 else NUM_LOOPS}\n")

    loop_count = 0
    try:
        while True:
            send_command(ser, f"B{ANGLE_A}")
            time.sleep(DELAY)

            send_command(ser, f"B{ANGLE_B}")
            time.sleep(DELAY)

            loop_count += 1
            print(f"  [Loop {loop_count} complete]")

            if NUM_LOOPS > 0 and loop_count >= NUM_LOOPS:
                break

    except KeyboardInterrupt:
        print("\nStopped by user.")

    ser.close()
    print("Done.")


if __name__ == "__main__":
    main()