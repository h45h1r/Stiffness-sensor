import serial
import time
import timeit
import numpy as np
import csv
import re
from threading import Event

# ========== SETTINGS ==========
np.set_printoptions(formatter={'float_kind': '{:f}'.format})

# CNC Coordinates (X and Y only)
x_cord = -146
y_cord = -44
X_sensor = 'X' + str(x_cord)
Y_sensor = 'Y' + str(y_cord)

# CNC Serial Setup
s_machine = serial.Serial('/dev/tty.usbserial-11340', 115200)

# Arduino Serial Ports (optional)
motor_port = '/dev/tty.usbmodem101'        # Arduino handling X/Y
pressure_port = '/dev/tty.usbmodem113301'  # Arduino handling M/R/Z

# File for saving data
# file_name = '0_PSI_3mm_1.csv'

# ========== SENSOR FUNCTIONS (COMMENTED OUT) ==========
# def read_calibration_file():
#     cal_file = "FT26196.cal"
#     with open(cal_file, 'r') as f:
#         lines = f.read().splitlines()

#     calibration_raw_data = [lines[-9 + i] for i in range(6)]
#     calibration_matrix = [[float(s) for s in re.findall(r'[-+]?(?:\d{1,3}(?:,\d{3})+|\d+)(?:\.\d+)?', row)] for row in calibration_raw_data]

#     global calibration_matrix_trimmed
#     calibration_matrix_trimmed = np.delete(calibration_matrix, -1, 1)

# def sensor_bias():
#     global bias_reading
#     bias_reading = np.zeros(6)
#     for _ in range(10):
#         get_DAQ_data()
#         bias_reading += np.array(data_from_DAQ)
#     bias_reading /= 10

# def get_DAQ_data():
#     import nidaqmx
#     with nidaqmx.Task() as task:
#         for i in range(6):
#             task.ai_channels.add_ai_voltage_chan(f"Dev1/ai{i}")
#         global data_from_DAQ
#         data_from_DAQ = task.read()

# def get_FT_data():
#     get_DAQ_data()
#     raw = np.array(data_from_DAQ)
#     biased = raw - bias_reading
#     calibrated = np.transpose(np.matmul(calibration_matrix_trimmed, biased.T))
#     return calibrated

# def initialize_ATI_mini40():
#     read_calibration_file()
#     sensor_bias()

# ========== CNC CONTROL ==========
def wait_for_movement_completion():
    Event().wait(1)
    idle_counter = 0
    while True:
        Event().wait(0.01)
        s_machine.reset_input_buffer()
        s_machine.write(b'?\n')
        resp = s_machine.readline().strip().decode('utf-8')
        if 'Idle' in resp:
            idle_counter += 1
        if idle_counter > 5:
            break

def send_command(command):
    s_machine.write((command + '\n').encode())
    s_machine.flush()
    time.sleep(0.1)
    while s_machine.in_waiting:
        response = s_machine.readline().decode().strip()
        print("Response:", response)

def initialize_cnc():
    s_machine.write(b"\r\n\r\n")
    time.sleep(2)
    s_machine.flushInput()
    send_command('$$')
    send_command('$X')
    send_command('G90')
    send_command('G21')
    feed_rate(1000, 1000, 100)

def feed_rate(X_feed, Y_feed, Z_feed):
    send_command('$110=' + str(X_feed))
    send_command('$111=' + str(Y_feed))
    send_command('$112=' + str(Z_feed))

# ========== FORCE DATA ==========
def mean_force_data():
    Fx_sum = Fy_sum = Fz_sum = 0
    for _ in range(5):
        # Fx, Fy, Fz, *_ = get_FT_data()
        # Replace with dummy data for now
        Fx, Fy, Fz = 0.0, 0.0, -0.05
        Fx_sum += Fx
        Fy_sum += Fy
        Fz_sum += Fz
    return Fx_sum / 5, Fy_sum / 5, Fz_sum / 5

# ========== ARDUINO INTERACTIVE CONTROL ==========
def interactive_arduino_control():
    try:
        motor_ser = serial.Serial(motor_port, 57600, timeout=1)
        pressure_ser = serial.Serial(pressure_port, 57600, timeout=1)
        time.sleep(2)
        print("Ready. Use X0–X4, Y0–Y6, M#, R, Z. Type 'exit' to quit.")

        while True:
            cmd = input("Enter command: ").strip().upper()
            if cmd == 'EXIT':
                break
            if not cmd:
                print("Empty command.")
                continue

            target = None
            if cmd.startswith("X") and cmd[1:].isdigit():
                target = 'motor'
            elif cmd.startswith("Y") and cmd[1:].isdigit():
                target = 'motor'
            elif cmd.startswith("M") and cmd[1:].isdigit():
                target = 'pressure'
            elif cmd in ["R", "Z"]:
                target = 'pressure'
            else:
                print("Invalid command.")
                continue

            ser = motor_ser if target == 'motor' else pressure_ser
            ser.write((cmd + '\n').encode())
            print(f"[Python → {target}] Sent {cmd}")

            while True:
                line = ser.readline().decode().strip()
                if line:
                    print(f"[{target.upper()} Arduino] {line}")
                    if any(trigger in line for trigger in ["DoneX:", "DoneY:", "DoneM:", "Pressure:", "reset", "Error"]):
                        break

    except Exception as e:
        print(f"Serial error: {e}")

    finally:
        try: motor_ser.close()
        except: pass
        try: pressure_ser.close()
        except: pass
        print("Serial connections closed.")

# ========== MAIN SCRIPT ==========
# initialize_ATI_mini40()  # Uncomment when sensor is connected
initialize_cnc()
wait_for_movement_completion()
send_command('$H')  # Home machine
wait_for_movement_completion()
send_command(X_sensor)
send_command(Y_sensor)
wait_for_movement_completion()

# ========== TOUCHDOWN TO DETECT Z ZERO ==========
step_size = 0.05
incremented_step = 0.0

# Touchdown loop to detect contact
while True:
    Fx_mean, Fy_mean, Fz_mean = mean_force_data()
    print(f"Fz_mean: {Fz_mean}")

    # Simulate contact detection (adjust value once real sensor is connected)
    if Fz_mean > -0.03:  # ← Change this threshold once confirmed
        print("No Force Detected")
        send_command('Z' + str(incremented_step))
        wait_for_movement_completion()
        incremented_step = round(incremented_step - step_size, 5)
    else:
        print("Sensor Touching - Contact Made")
        break

# Retract to set Z zero (adjust value once confirmed)
retract_distance = 1.0  # ← Set this to desired retract height after contact
z_zero_value = round(incremented_step + retract_distance, 5)
send_command('Z' + str(z_zero_value))
wait_for_movement_completion()

print(f"Z zero set to {z_zero_value}")