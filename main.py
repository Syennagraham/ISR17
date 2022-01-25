import re
import time
import serial
from tkinter import *
from tkinter import ttk
from math import sqrt, pow
from numpy import interp
import threading
import random

arduino = serial.Serial('/dev/cu.usbmodem1201', 9600, timeout=.1)

# MINOR DISPLAY SETUP ITEMS
root = Tk()  # initialize root variable
root.geometry("800x480")  # root sized to hdmi monitor
root.title('Nautilus HUD')


# STYLE CONFIGURATION
root.style = ttk.Style(root)
root.style.configure('title.TLabel', font=('Times', 14, 'bold')) # For gauge labels
root.style.configure('.TLabel', font=('Times', 14)) # gauge readings

# GRID MANAGEMENT (2x3)
root.columnconfigure(0, weight=3)
root.columnconfigure(1, weight=1)
root.rowconfigure(1, weight=3)
root.rowconfigure(2, weight=2)


# HEADING DISPLAY SETUP
HEIGHT = 200
WIDTH = 200
RADIUS = 30
TAG = "cir"

decode_rpms = 0
gyro_coord = [0,0] 
rpmA = 0
depth = 0
depth_indicator = 0
decode_ps_voltage = 0
psv = 0

# labels
heading_label = ttk.Label(root, text='HEADING', style='title.TLabel').grid(column=0, row=0, sticky='n')
heading_up = ttk.Label(root, text='UP', style='.TLabel').grid(column=0, row=1, sticky='n')
heading_port = ttk.Label(root, text='PORT', style='.TLabel').grid(column=0, row=1, sticky='w', padx=100)
heading_starboard = ttk.Label(root, text='STARBOARD', style='.TLabel').grid(column=0, row=1, sticky='e', padx=40)
heading_down = ttk.Label(root, text='DOWN', style='.TLabel').grid(column=0, row=1, sticky='s')
# canvas items
heading_canvas = Canvas(root, width=WIDTH, height=HEIGHT)
heading_canvas.create_line((WIDTH/2)+20, (HEIGHT/2), (HEIGHT/2)-20, (HEIGHT/2))
heading_canvas.create_line((WIDTH/2), (HEIGHT/2)+20, (HEIGHT/2), (HEIGHT/2)-20)
heading_canvas.create_oval((WIDTH/2)-RADIUS, (HEIGHT/2)-RADIUS, (WIDTH/2)+RADIUS,
        (HEIGHT/2)+RADIUS, fill='green', tags=TAG)
heading_canvas.grid(column=0, row=1)


# DEPTH DISPLAY SETUP
# labels
depth_label = ttk.Label(root, text='DEPTH', style='title.TLabel').grid(column=1, row=0, sticky='n')
# canvas items
depth_canvas = Canvas(root, height=400, width=100)
depth_canvas.create_rectangle(3, 400, 100, 3, width='3')
depth_canvas.grid(column=1, row=1, rowspan=2)
# (x0, y0, x1, y1) = (over 3, down 400, over 100, go down to depth #)
depth_bar = depth_canvas.create_rectangle(3, 400, 100, depth, fill='#FFCC00')


# RPM DISPlAY SETUP
#labels
RPM_label = ttk.Label(root, text='RPM', style='title.TLabel').grid(column=0, row=2, sticky="nw")
# canvas items
RPM_canvas = Canvas(root, height=100, width=550)
RPM_canvas.create_rectangle(550, 3, 3, 100, width='3')
RPM_bar = RPM_canvas.create_rectangle(rpmA, 3, 3, 100, fill='#FFCC00')
RPM_canvas.grid(column=0, row=2)


# READ ARDUINO 
# read arduino data
def read_arduino():
    return arduino.readline()[:-2] #the last bit gets rid of the new-line chars


# PRESSURE SENSOR
# read pressure sensor data
def pressure_sensor():
    while True:
        ps_voltage = read_arduino()
        if ps_voltage == b'':
            continue
        else:
            decode_ps_voltage = int(ps_voltage.decode('utf8')) #0 -1023
            print("Pressure Sensor Voltage Bytes= ", decode_ps_voltage)
            depth_coord = convert_volts_to_coord(decode_ps_voltage) # 400 - 0
            return depth_coord, decode_ps_voltage


# convert pressure sensor voltage to coordinates for canvas display
def convert_volts_to_coord(psv_data):
    if psv_data >= 1023:
        return 100
    elif psv_data <= 0:
        return 400
    else:
        print('psv_data ', psv_data)
        # change these voltages for the pool voltages 0(0.5V) - 1023(4.5V)
        depth_indicator = interp(psv_data,[0,1023],[400,100])
        return int(depth_indicator)


# calculate depth based on pressure 
def calculate_depth(psv_data):
    # where to get the formula - https://www.youtube.com/watch?v=AB7zgnfkEi4
    gravity = 9.8 #m/s
    density = 997.453 #kg/m^3
    voltage= 0.5 + 4.5 * ((psv_data-0)/(1023-0)) #convert bytes to voltage
    #print("Pressure Sensor Voltage = ", voltage)

    pressure_kpascal = (3.0*(voltage-0.47))*1000.0
    #print("Pressure (kPA)= ", pressure_kpascal)   

    depth_meters = round(pressure_kpascal/( gravity * density), 2)
    #print("Depth Value (m) = ", depth_meters)
    return depth_meters


# RPM SENSOR
# read rpm sensor data
def rpm_sensor():
    while True:
        rpm_value = read_arduino()
        if rpm_value == b'':
            continue
        else:
            decode_rpms = int(rpm_value.decode('utf8')) #0-250
            rpms = convert_rpms_to_coord(decode_rpms) #3-550
            print(rpms)
            return rpms


# convert rpm value to coordinate for display
def convert_rpms_to_coord(data):
    if data >= 250:
        return 550
    elif data <= 0:
        return 3
    else:
        # change these rpm ranges for estimated RPM range
        rpm_indicator = interp(data,[0,250],[3,550])
        return int(abs(rpm_indicator))


# GYRO SENSOR
# read gyro sensor data
def gyro_sensor():
    global gyro_coord
    while True:
        data = read_arduino()
        print(data)
        gs_string = (data.decode('utf8'))
        if (data == b'') or (re.match( r'^\.' or r'^\>', gs_string)):  
            print('data2 ', data)
            continue
        else: 
            round_inc = map_gyro_to_graphic(gs_string)
            gyro_coord = convert_gyro_to_coord(round_inc)
            print('x, y', gyro_coord)


# change color of circle on display according to value
def get_circle_color(x, y, radius):
    #check if coordinate is with in radius of origin
    if  radius > int(sqrt( pow(abs(x-(WIDTH/2)), 2) + pow(abs(y-(HEIGHT/2)), 2))): 
        return 'green'
    elif   radius * 2 < int(sqrt( pow(abs(x-(WIDTH/2)), 2) + pow(abs(y-(HEIGHT/2)), 2))): 
        return 'red'
    else:
        return 'yellow'


# create circle element for display
def create_circle(x, y, r, canvasName, t):
    color = get_circle_color(x,y,r)
    canvasName.create_oval(x-r, y-r, x+r, y+r, fill=color, tags=t)
    root.update()


# delete circle element for display
def delete_circle(canvasName, tag):
    canvasName.delete(tag)
    root.update()


# map incoming gryo bytes to 1 - 9 
def map_gyro_to_graphic(data):
    gs_list = []
    for x in data.split('#'):
        if float(x) > 90.0:
            gs_list.append(90)
        elif float(x) < -90.0:
            gs_list.append(-90)
        else:
            gs_list.append(float(x))
    
    print('gs_list', gs_list)
    rounded_gs_indicator = [int(round(gs_list[2])/10), int(round(gs_list[3])/10)]
    print('rounded_gs_indicator = ', rounded_gs_indicator)
    return rounded_gs_indicator 


# convert mapped gyro data to coordinates for display
def convert_gyro_to_coord(data):
    y = interp(data[0],[-9,9],[50,150])
    x = interp(data[1],[-9,9],[150,50])
    print('pitch, yaw', int(x), int(y))
    return int((x)), int((y))


# update dashboard
def gyro_dashboard():
    delete_circle(heading_canvas, TAG)
    create_circle(gyro_coord[0], gyro_coord[1], RADIUS, heading_canvas, TAG)


def rpm_dashboard():
    #decode_rpms, rpmA = rpm_sensor()
    RPM_canvas.coords(RPM_bar, rpmA, 3, 3, 100)
    RPM_value = ttk.Label(root, text=decode_rpms, style='.TLabel').place(x=20, y=410, anchor='w')


def depth_dashboard():
    #depth_indicator, psv = pressure_sensor()
    depth = calculate_depth(psv)
    depth_canvas.coords(depth_bar, 3, 400, 100, depth_indicator)
    depth_value = ttk.Label(root, text=str(depth), style='.TLabel').place(x=730, y=25, anchor='n')


# DYNAMIC ELEMENTS LOOP
def update_gui():
    while True:
        gyro_dashboard()
        rpm_dashboard()
        depth_dashboard()
        time.sleep(0.1)


def get_random_xy_coord():
    global gyro_coord
    global rpmA
    global decode_rpms
    global depth_indicator
    global decode_ps_voltage
    while True:
        time.sleep(0.5)
        data = [random.randrange(-9,9,1), random.randrange(-9,9,1)]
        gyro_coord = convert_gyro_to_coord(data)

        decode_rpms = random.randrange(0, 250)
        rpmA = interp(decode_rpms,[0,250],[3,550])

        decode_ps_voltage = random.randrange(0,1023)
        depth_indicator = convert_volts_to_coord(decode_ps_voltage) 


       

if '__main__' == __name__:
    # heading elements
    th = threading.Thread(target=gyro_sensor, args=(),  daemon=True)
    th.start()

    update_gui()


    root.mainloop()  # run hud
    test.mainloop()
