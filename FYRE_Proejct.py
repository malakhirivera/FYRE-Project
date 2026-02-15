import time
import csv
from datetime import datetime
import re
import serial 
import serial.tools.list_ports 
import pyqtgraph as pg 
# from pyqtgraph.Qt import QtWidgets
import math
import random

#--------------------
# Configuration
#--------------------
ENABLE_PLOT = True
LOOP_PERIOD = 0.5
CSV_FILENAME = "susceptibility_log.csv"

LOCKIN_BAUDRATE = 9600
ARDUINO_BAUDRATE = 9600

USE_MOCK_LOCKIN = True
USE_MOCK_ARDUINO = True
TEST_MODE = True

MAX_POINTS = 200

#------------------------
# Find Serial Ports 
#------------------------
def find_port(keyword):
    for p in serial.tools.list_ports.comports():
        if keyword in p.description:
            return p.device
    raise RuntimeError(f"{keyword} not found")

if not TEST_MODE:
    lockin_port = find_port("PL2303")
    arduino_port = find_port("Arduino")
else:
    lockin_port = None
    arduino_port = None


#-----------------------
# Mock Lock-in Class
#-----------------------
class MockLockin:
    def __init__(self, port=None):
        self.t0 = time.time()
        print("Mock lock-in initialized")

    def read_xy(self):
        t = time.time() - self.t0
        X = math.sin(t) + 0.05 * random.random()
        Y = math.cos(t) + 0.05 * random.random()
        return X, Y
    
    def close(self):
        print("Mock lock-in closed")


#------------------------
# Lock-in Class
#------------------------
class SerialLockin:
    def __init__(self, port):

        self.ser = serial.Serial(
            port = port,
            baudrate = LOCKIN_BAUDRATE,
            timeout = 0.1
        )

        time.sleep(1)
        self.ser.reset_input_buffer()
        print(f"Lock-in connected on {port}")

    def query(self, cmd):
        self.ser.write((cmd + "\n").encode())
        return self.ser.readline().decode(errors="ignore").strip()

    def read_xy(self):
        resp = self.query("SNAP? 1,2")
        try:
            x_str, y_str = resp.split(",")
            return float(x_str), float(y_str)
        except Exception:
            raise ValueError(f"Bad lock-in response: {resp}")


    def close(self):
        self.ser.close()

#--------------------------
# Mock Arduino Class
#--------------------------
class MockArduinoTemp:
    def __init__(self, port = None):
        self.t0 = time.time()
        print("Mock Arduino Initialized")
    
    def read_temp(self):
        t = time.time() - self.t0
        temp = 25 + 5 * math.sin(t/20) + 0.2 * random.random()
        return temp
    
    def close(self):
        print("Mock Arduino Close")

#--------------------------
# Arduino Temperature Class
#--------------------------
class ArduinoTemp:
    def __init__(self, port):
        self.ser = serial.Serial(
            port = port,
            baudrate = ARDUINO_BAUDRATE,
            timeout = 0.1
        )
        time.sleep(2)
        self.ser.reset_input_buffer()
        print(f"Arduino connected on {port}")

    def read_temp(self):
        line = self.ser.readline().decode(errors = 'ignore').strip()
        match = re.search(r"([-+]?\d*\.\d+|\d+)", line)
        return float(match.group(1)) if match else None
    
    def close(self):
        self.ser.close()


if ENABLE_PLOT:
    app = pg.mkQApp("Susceptibility Plot")
    win = pg.GraphicsLayoutWidget(show = True, title = "Real-Time Susceptibility vs Temperature")
    plot = win.addPlot(title = "Susceptibility vs Temperature")
    plot.setLabel('bottom', 'Temperature (C)')
    plot.setLabel('left', 'Real Susceptiblity X')
    curve = plot.plot([], [], pen = None, symbol = 'o', symbolSize = 6)

    temps = []
    real_vals = []

    last_time = time.time()


#------------------------
# Main Loop
#------------------------
def main():
    if USE_MOCK_LOCKIN:
        lockin = MockLockin()
    else:
        lockin = SerialLockin(lockin_port)

    if USE_MOCK_ARDUINO:
        arduino = MockArduinoTemp()
    else:
        arduino = ArduinoTemp(arduino_port)
    

    with open(CSV_FILENAME, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["timestamp", "temperature_C", "X", "Y"])

        print("Starting acquisition (CTRL+C to stop)")

        try:
            while True:
                timestamp = datetime.now().isoformat()
                
                try:
                    X, Y = lockin.read_xy()
                except Exception as e:
                    print(f"lock-in read error: {e}")
                    X, Y = float('nan'), float('nan')

                temp = arduino.read_temp()

                print(timestamp, temp, X, Y)
                writer.writerow([timestamp, temp, X, Y])
                f.flush()

                # loop_now = time.time()
                # loop_dt = loop_now - last_time
                # last_time = loop_now
                # print(f"[DEBUG] loop_dt={loop_dt:.f} seconds")

                if ENABLE_PLOT and temp is not None:
                    temps.append(temp)
                    real_vals.append(X)

                    temps[:] = temps[-MAX_POINTS:]
                    real_vals[:] = real_vals[-MAX_POINTS:]

                    curve.setData(temps, real_vals)
                    pg.QtWidgets.QApplication.processEvents()

                time.sleep(LOOP_PERIOD)


        except KeyboardInterrupt:
            print("Stopping acquisition...")

        finally:
            f.close()
            lockin.close()
            arduino.close()


#------------------------
# Program Entry Point
#------------------------
if __name__ == "__main__":
    main()
