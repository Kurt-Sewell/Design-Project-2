import tkinter as tk
import serial
import serial.tools.list_ports

# Open your serial port (make sure COM7 is free)
ser = serial.Serial(port='COM7', timeout=0.5)

def getData():
    raw = ser.readline().decode('ascii', errors='ignore').strip()
    if raw:
        line = f'"{raw}" read\n'
        # Insert at end, then scroll to the end
        txt.insert(tk.END, line)
        txt.see(tk.END)
    root.after(500, getData)

root = tk.Tk()
root.title("Serial Reader")

# Create a Text widget with a vertical scrollbar
frame = tk.Frame(root)
frame.pack(fill=tk.BOTH, expand=True)

scrollbar = tk.Scrollbar(frame)
scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

txt = tk.Text(frame, wrap=tk.NONE, yscrollcommand=scrollbar.set)
txt.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

scrollbar.config(command=txt.yview)

# Kick off the polling loop
getData()

root.mainloop()
