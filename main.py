import serial
import threading
import queue
from drawnow import drawnow
import matplotlib.pyplot as plt

SERIAL_PORT = '/dev/cu.usbmodem1301'
BAUD_RATE = 9600
ser = serial.Serial(SERIAL_PORT, BAUD_RATE)

data1 = []
data2 = []
data3 = []
time = []
data_queue = queue.Queue()


def make_fig():
    plt.subplot(3, 1, 1)
    plt.plot(time, data1, 'mo-')
    plt.ylabel('Pressure (hPa)')

    plt.subplot(3, 1, 2)
    plt.plot(time, data2, 'mo-')
    plt.ylabel('Temperature (C)')

    plt.subplot(3, 1, 3)
    plt.plot(time, data3, 'mo-')
    plt.xlabel('Time (s)')
    plt.ylabel('Altitude (m)')


def read_and_update_serial_data():
    while True:

        line1 = ser.readline().decode('utf-8').strip()
        line2 = ser.readline().decode('utf-8').strip()
        line3 = ser.readline().decode('utf-8').strip()
        line4 = ser.readline().decode('utf-8').strip()
        data_queue.put((line1, line2, line3, line4))


def update_data():
    while not data_queue.empty():
        line1, line2, line3, line4 = data_queue.get()
        try:
            if line1:
                data1.append(float(line1))
            if line2:
                data2.append(float(line2))
            if line3:
                data3.append(float(line3))
            if line4:
                time.append(float(line4))
        except ValueError:
            print("Error: Non-numeric data found in the line.")


def send_commands_to_arduino():
    while True:
        command = input("Enter command for the rover: ")
        ser.write((command + '\n').encode())


thread_read = threading.Thread(target=read_and_update_serial_data)
thread_read.daemon = True
thread_read.start()

thread_command = threading.Thread(target=send_commands_to_arduino)
thread_command.daemon = True
thread_command.start()

try:
    while True:
        update_data()
        drawnow(make_fig)
except KeyboardInterrupt:
    print("Exiting...")
    plt.savefig("results.pdf")