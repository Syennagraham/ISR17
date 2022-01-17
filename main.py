import time
import serial
from tkinter import *
from tkinter import ttk

arduino = serial.Serial('/dev/cu.usbmodem1201', 9600, timeout=.1)

########################################################################################################################
# test scale
########################################################################################################################
test = Tk()
test.title('Test')
scalenum=DoubleVar()
test_scale = Scale(test, from_=0, to=400, length=140, orient=VERTICAL, variable=scalenum).pack()

rpmA = 0
depth = 0
########################################################################################################################


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
# labels
heading_label = ttk.Label(root, text='HEADING', style='title.TLabel').grid(column=0, row=0, sticky='n')
heading_up = ttk.Label(root, text='UP', style='.TLabel').grid(column=0, row=1, sticky='n')
heading_port = ttk.Label(root, text='PORT', style='.TLabel').grid(column=0, row=1, sticky='w', padx=100)
heading_starboard = ttk.Label(root, text='STARBOARD', style='.TLabel').grid(column=0, row=1, sticky='e', padx=40)
heading_down = ttk.Label(root, text='DOWN', style='.TLabel').grid(column=0, row=1, sticky='s')
# canvas items
heading_canvas = Canvas(root, height=250, width=300)
heading_canvas.create_oval(25, 3, 275, 250, width='3')
heading_canvas.create_line(25, 125, 275, 125, width='3')
heading_canvas.create_line(150, 0, 150, 250, width='3')
heading_arrow = heading_canvas.create_line(150, 125, 150, 0, fill='#FFCC00', width='5',arrow='last')
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

def read_arduino():
    return arduino.readline()[:-2] #the last bit gets rid of the new-line chars

def convert_volts_to_coord(psv):
    if psv >= 1023:
        return 100
    elif psv <= 980:
        return 400
    else:
        # change these voltages for the pool voltages 0(0.5V) - 1023(4.5V)
        y1_coord_depth_bot = 400  # bottom coor of depth rect window 
        y2_coord_depth_top = 100  # top coor of depth rect window     
        low_pressure_depth = 980  # bottom voltage range              
        high_presure_depth = 1023 # top voltage range                 
        depth_indicator = y1_coord_depth_bot + ((y2_coord_depth_top - y1_coord_depth_bot) / (high_presure_depth - low_pressure_depth)) * (psv - low_pressure_depth)
        return int(depth_indicator)


def get_data_from_arduino():
    while True:
        ps_voltage = read_arduino()
        if ps_voltage == b'':
            continue
        else:
            decode_voltage = int(ps_voltage.decode('utf8'))
            print("Pressure Sensor Voltage Bytes= ", decode_voltage)
            depth_coord = convert_volts_to_coord(decode_voltage)
            return depth_coord, decode_voltage


def calculate_depth(data):
    # where to get the formula - https://www.youtube.com/watch?v=AB7zgnfkEi4
    gravity = 9.8 #m/s
    density = 997.453 #kg/m^3
    voltage= 0.5 + 4.5 * ((data-0)/(1023-0)) #convert bytes to voltage
    print("Pressure Sensor Voltage = ", voltage)

    pressure_kpascal = (3.0*(voltage-0.47))*1000.0
    print("Pressure (kPA)= ", pressure_kpascal)   

    depth_meters = round(pressure_kpascal/( gravity * density), 2)
    print("Depth Value (m) = ", depth_meters)
    return depth_meters, pressure_kpascal


def convert_rpms_to_coord(data):
    if data >= 250:
        return 550
    elif data <= 0:
        return 3
    else:
        # change these rpm ranges for estimated RPM range
        x2_coord_rpm_right = 550  # right coor of rpm rect window #OE
        x1_coord_rpm_left = 3     # left coor of rpm rect window  #OS   
        low_rpms = 0  # bottom rpm range #IS              
        high_rpms = 250 # top rpm range  #IE               
        rpm_indicator = x1_coord_rpm_left + ((x2_coord_rpm_right - x1_coord_rpm_left) / (low_rpms - high_rpms)) * (data - low_rpms)
        return int(abs(rpm_indicator))


def rpm_sensor():
    while True:
        data = read_arduino()
        if data == b'':
            continue
        else:
            decode_rpms = int(data.decode('utf8'))
            rpms = convert_rpms_to_coord(decode_rpms)
            print(rpms)
            return decode_rpms, rpms


# DYNAMIC ELEMENTS LOOP
while True:
    # depth elements
    depth_indicator, psv = get_data_from_arduino()
    depth, pressure = calculate_depth(psv)
    depth_canvas.coords(depth_bar, 3, 400, 100, depth_indicator)
    depth_value = ttk.Label(root, text=str(depth), style='.TLabel').place(x=730, y=25, anchor='n')
    
    decode_rpms, rpmA = rpm_sensor()
    RPM_canvas.coords(RPM_bar, rpmA, 3, 3, 100)
    RPM_value = ttk.Label(root, text=decode_rpms, style='.TLabel').place(x=20, y=410, anchor='w')

    # heading elements
    heading_x = int(scalenum.get())
    heading_y = int(scalenum.get())
    heading_canvas.coords(heading_arrow, 150, 125, heading_x, heading_y)
    time.sleep(0.01)
    root.update()

root.mainloop()  # run hud
test.mainloop()

