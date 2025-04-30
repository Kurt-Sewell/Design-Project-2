import tkinter as tk
import serial

ser = serial.Serial(port='COM7', timeout=0.5)

looping = None


def getData():
    while True:
        raw = ser.readline().decode('ascii', errors='ignore').strip()
        if raw == ("GPS Data:"):
            break
    long = ser.readline().decode('ascii', errors='ignore').strip()
    lat = ser.readline().decode('ascii', errors='ignore').strip()
    alt = ser.readline().decode('ascii', errors='ignore').strip()
    sat = ser.readline().decode('ascii', errors='ignore').strip()
    while True:
        raw = ser.readline().decode('ascii', errors='ignore').strip()
        if raw == "IMU Data:":
            break

    accel = ser.readline().decode('ascii', errors='ignore').strip()
    mag = ser.readline().decode('ascii', errors='ignore').strip()
    vel = ser.readline().decode('ascii', errors='ignore').strip()
    gyro = ser.readline().decode('ascii', errors='ignore').strip()

    line1.set(f"{long}\n{lat}\n{alt}\n{sat}")
    line2.set(f"{accel}\n{mag}\n{vel}\n{gyro}")

    global looping
    looping = root.after(500, getData)


def start():
    global looping
    startBut.lower()
    stopBut.lift()
    if looping is None:
        getData()


def stop():
    global looping
    if looping is not None:
        root.after_cancel(looping)
        looping = None
    stopBut.lower()
    startBut.lift()
    line1.set("")
    line2.set("")


root = tk.Tk()
root.title("GPS App")

line1 = tk.StringVar()
line2 = tk.StringVar()

# GPS fields
tk.Label(root, text='GPS:').grid(row=0, column=0, padx=5, pady=5, sticky='e')
tk.Label(root, textvariable=line1).grid(row=0, column=1, padx=40, sticky='w')

# IMU fields
tk.Label(root, text='IMU:').grid(row=1, column=0, padx=5, pady=5, sticky='e')
tk.Label(root, textvariable=line2).grid(row=1, column=1, padx=5, sticky='w')

startBut = tk.Button(root, text="Start Display", command=start)
startBut.grid(row=2, column=0, columnspan=2, pady=10)

stopBut = tk.Button(root, text="Stop Display", command=stop)
stopBut.grid(row=2, column=0, columnspan=2, pady=10)

startBut.lift()

root.mainloop()
