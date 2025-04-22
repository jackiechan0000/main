import serial
import time

# --- CONFIGURATION ---
port = 'COM5'
baud = 115200
timeout = 0.01
filename = 'loadcell_log.csv'
# ----------------------

ser = serial.Serial(port, baud, timeout=0.01)
time.sleep(2)  # Allows Arduino time to reset
start_time = time.time()

with open(filename, 'w') as f:
    print("Logging started... Press Ctrl+C to stop.")
    f.write("Timestamp,Weight\n")
    try:
        while True:
            line = ser.readline().decode('utf-8').strip()
            if "Weight:" in line:
                weight = line.split(":")[1].replace("grams", "").strip()
                elapsed = time.time() - start_time
                timestamp = f"{elapsed:.4f}"  # Seconds with milliseconds
                f.write(f"{timestamp},{weight}\n")
                print(f"{timestamp} s : {weight} grams")
    except KeyboardInterrupt:
        print("Logging stopped.")
ser.close()
