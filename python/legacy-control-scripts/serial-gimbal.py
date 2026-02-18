import serial
import time

# =====================================================
# CONFIGURATION
# =====================================================
SERIAL_PORT = "COM6"      # Change to your port
BAUD_RATE = 115200

# Sweep parameters
ANGLE_A = 5               # Start position (radians)
ANGLE_B = 7               # End position (radians)
STEP_SIZE = 0.05           # Increment per step (radians)
STEP_DELAY = 0.05         # Delay between steps (seconds)
PAUSE_AT_ENDS = 1.0       # Pause at A and B before reversing (seconds)
NUM_LOOPS = 0             # Number of full A→B→A loops (0 = infinite)

# =====================================================
# MAIN
# =====================================================
def send_command(ser, cmd):
    full_cmd = cmd + "\n"
    ser.write(full_cmd.encode())
    time.sleep(0.02)
    while ser.in_waiting:
        print(f"  <- {ser.readline().decode().strip()}")

def sweep(ser, start, end, step, delay):
    """Smoothly step from start to end."""
    if start < end:
        pos = start
        while pos <= end:
            send_command(ser, f"B{pos:.4f}")
            pos += step
            time.sleep(delay)
        # Ensure we hit the exact endpoint
        send_command(ser, f"B{end:.4f}")
    else:
        pos = start
        while pos >= end:
            send_command(ser, f"B{pos:.4f}")
            pos -= step
            time.sleep(delay)
        send_command(ser, f"B{end:.4f}")

def main():
    print(f"Connecting to {SERIAL_PORT} at {BAUD_RATE} baud...")
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    time.sleep(2)

    while ser.in_waiting:
        print(ser.readline().decode().strip())

    steps = int(abs(ANGLE_B - ANGLE_A) / STEP_SIZE)
    sweep_time = steps * STEP_DELAY

    print(f"\n=== Smooth Sweep ===")
    print(f"Range: {ANGLE_A} <-> {ANGLE_B} rad")
    print(f"Step: {STEP_SIZE} rad every {STEP_DELAY}s  ({steps} steps, ~{sweep_time:.1f}s per sweep)")
    print(f"Loops: {'infinite' if NUM_LOOPS == 0 else NUM_LOOPS}\n")

    loop_count = 0
    try:
        while True:
            print(f"  >> Sweeping A -> B ({ANGLE_A} -> {ANGLE_B})")
            sweep(ser, ANGLE_A, ANGLE_B, STEP_SIZE, STEP_DELAY)
            time.sleep(PAUSE_AT_ENDS)

            print(f"  << Sweeping B -> A ({ANGLE_B} -> {ANGLE_A})")
            sweep(ser, ANGLE_B, ANGLE_A, STEP_SIZE, STEP_DELAY)
            time.sleep(PAUSE_AT_ENDS)

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