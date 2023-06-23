import serial
from tkinter import *
from tkinter import filedialog
from tkinter.messagebox import showerror, showinfo
from PIL import Image, ImageTk
import gxipy as gx
import os
from serial.tools import list_ports
from threading import Thread
import cv2
import numpy as np
import datetime
import tifffile as tif
from fitter import Fitter
from scipy import stats
import tk_tools
import csv

# create a device manager
device_manager = gx.DeviceManager()
dev_num, dev_info_list = device_manager.update_device_list()
cam_available = True
if dev_num == 0:
    print("No camera detected")
    cam_available = False
else:
    # open the first device
    cam = device_manager.open_device_by_index(1)

    # enable frame rate mode
    cam.AcquisitionFrameRateMode.set(gx.GxSwitchEntry.ON)

    # if camera is color
    if cam.PixelColorFilter.is_implemented() is True:
        print("Color camera not supported.")
        cam.close_device()
        exit()

streaming = False
recordSpeckle = False
recordNoise = False
motor_arduino_ready = False
pump = -1
pressure_arduino_ready = False
cam_pos_check = False
LED = False
roi = [0,0,0,0]
prevSSE = -1
prevSTD = -1
resetROI = False

def startStopCamera():
    global streaming
    global exposure_time 
    global recordSpeckle
    global numpy_image
    global cam_pos_check
    global roi
    exposure_time = float(exposure_box.get())*1000
    frame_rate = float(frame_entry.get())
    if streaming:
        cam.stream_off()
        streaming = False
        return
    elif cam_available:
        streaming = True
        # set continuous acquisition
        cam.TriggerMode.set(gx.GxSwitchEntry.OFF)

        # set auto exposure time
        cam.ExposureAuto.set(0)

        # start data acquisition
        cam.stream_on()
  
        while streaming:
            # set exposure
            cam.ExposureTime.set(exposure_time)

            # set frame rate
            cam.AcquisitionFrameRate.set(frame_rate)
            if recordSpeckle or recordNoise:
                all_imgs = []
                transducer_vals = []
                manometer_vals = []
                fields = ['Transducer', 'Manometer']
                mydir = folderPath.get()
                if os.path.exists(os.path.dirname(mydir)):
                    numOfFrames = int(numFrames_entry.get())
                    IOP = pressure_entry.get()
                    if float(IOP) < 10:
                        IOP = '0'+IOP
                    time = str(datetime.datetime.now().today()).replace('-',"_")[0:10]
                    exposure_box.configure(state=DISABLED)
                    frame_entry.configure(state=DISABLED)
                    numFrames_entry.configure(state=DISABLED)
                    pressure_entry.configure(state=DISABLED)
                    for _ in range(numOfFrames):
                        frame = cam.data_stream[0].get_image()
                        if frame is None:
                            print("Getting image failed.")
                            continue
                        numpy_image = frame.get_numpy_array()
                        numpy_image = numpy_image[int(roi[1]):int(roi[1]+roi[3]), int(roi[0]):int(roi[0]+roi[2])]
                        all_imgs.append(numpy_image)
                        img = Image.fromarray(numpy_image)
                        img = img.resize((720, 540))
                        img = ImageTk.PhotoImage(img)
                        l1['image'] = img
                        manometer_vals.append([float(manometer_setpoint.get())])
                        transducer_vals.append([float(transducer_IOP.get())])
                        all_pressures = [a + b for a, b in zip(transducer_vals, manometer_vals)]
                        root.update()
                    if recordNoise:
                        tif.imwrite(f"{mydir}/{time}_noise.tiff", np.array(all_imgs))
                    else:
                        tif.imwrite(f"{mydir}/{time}_{IOP}mmHg.tiff", np.array(all_imgs))
                        with open(f"{mydir}/{time}_{IOP}mmHg", 'w', newline='') as f:
                            write = csv.writer(f)
                            write.writerow(fields)
                            write.writerows(all_pressures)
                        showinfo(title="Complete", message="Recording Complete")
                        exposure_box.configure(state=NORMAL)
                        frame_entry.configure(state=NORMAL)
                        numFrames_entry.configure(state=NORMAL)
                        pressure_entry.configure(state=NORMAL)
                else:
                    showerror(title="Error", message="Invalid Directory")
                recordSpeckle = False

            else:
                # get raw image
                frame = cam.data_stream[0].get_image()
                if frame is None:
                    print("Getting image failed.")
                    continue
                numpy_image = frame.get_numpy_array()
                numpy_image = numpy_image[int(roi[1]):int(roi[1]+roi[3]), int(roi[0]):int(roi[0]+roi[2])]
                speckle_entry.configure(state=NORMAL)
                speckle_entry.delete(0, END)
                speckle_entry.insert(0, round(np.std(numpy_image.flatten()) / np.mean(numpy_image.flatten()), 3)) 
                speckle_entry.configure(state=DISABLED)
                img = Image.fromarray(numpy_image)
                img = img.resize((720, 540))
                img = ImageTk.PhotoImage(img)
                l1['image'] = img
                root.update()

                if cam_pos_check:
                    check_camera_position(numpy_image)

                if cam_check_bool.get():
                    prevSSE, prevSTD = check_laser(numpy_image, prevSSE, prevSTD)

def recordSpeckleFrames():
    global recordNoise
    global recordSpeckle
    global LED
    if numFrames_entry.get().isdigit() and pressure_entry.get().isdigit():
        if LED:
            if not recordSpeckle:
                recordSpeckle = not recordSpeckle
                recordNoise = not recordSpeckle
        else:
            showerror(title="Error", message="Invalid Error Margin for Pressure Values")
    else:
        showerror(title="Error", message="Invalid Entry") 
    return

def recordNoiseFrames():
    global recordNoise
    global recordSpeckle
    global LED
    if numFrames_entry.get().isdigit() and pressure_entry.get().isdigit():
        if not recordNoise:
            recordNoise = not recordNoise
            recordSpeckle = not recordNoise
    else:
        showerror(title="Error", message="Invalid Entry") 
    return

def selectROI():
    global numpy_image
    global roi
    global resetROI
    if not streaming:
        showerror(title="Error", message="No Live Feed")
        return
    try:
        if resetROI:
            roi = [0,0,0,0]
            roi_btn.config(text="Select ROI")
        else:
            roi = cv2.selectROI(numpy_image)
            roi_btn.config(text="Reset ROI")
    except NameError:
        showerror(title="Error", message="No Live Feed")
        return
    cv2.destroyAllWindows()

def start_thread():
    thread = Thread(target=selectROI, daemon=True)
    thread.start()

def check_camera_position(img: np.ndarray):
    global cam_pos_check
    img_mean = np.mean(img.flatten())
    left_img = img[:, 0:int(img.shape[1]*0.2)].flatten()
    right_img = img[:, int(img.shape[1]*0.8)::].flatten()
    left_mean = np.mean(left_img)
    right_mean = np.mean(right_img)
    left_range = np.ptp(left_img).astype(np.int16)
    right_range = np.ptp(right_img).astype(np.int16)
    line_profile = img[int(img.shape[0]/2), :]
    first_point = line_profile[0]
    sde = abs(np.sum(line_profile-first_point)) # sum of difference error
    if (
        (img_mean < 60) & (abs(left_mean - right_mean) < 10) & (abs(left_range - right_range) < 10)
        & (sde < 8000) # TODO: experiment with values
        ):
        cam_check_bool.set(True)
    else:
        cam_check_bool.set(False)

    cam_pos_check = False
    return

def check_laser(img: np.ndarray, prevSSE: float, prevSTD: float):
    img = img.flatten()
    f = Fitter(img, distributions='cauchy')
    f.fit()
    dist_type = list(f.get_best(method='sumsquare_error').keys())[0]
    SSE = f.summary().loc[dist_type]['sumsquare_error']
    std_val = f.fitted_param[dist_type][1]
    img_mean = np.mean(img)
    if (prevSSE != -1) & (prevSTD != -1):
        if (
            ((SSE > 0.0015) and (np.abs(prevSSE - SSE) > 5e-4)) 
            or ((std_val < 7) and (np.abs(prevSTD - std_val) > 2))
        ): # we confirm that the distribution is cauchy
            laser_ready_bool.set(False)
            return SSE, std_val
    else:
        if (SSE > 0.0015) or (std_val < 7):
            laser_ready_bool.set(False)
            return SSE, std_val

    if img_mean < 90: # TODO: mess around with value
        upper_percent, lower_percent = cauchy_analysis(img)
        if (upper_percent <= 0.01) & (lower_percent <= 0.01):
            laser_ready_bool.set(True)

    return SSE, std_val

def cauchy_analysis(img: np.ndarray):
    unique, counts = np.unique(img, return_counts=True)
    mode = stats.mode(img, keepdims=False)[0]
    halfmax = np.floor(stats.mode(img, keepdims=False)[1] / 2)
    loc = counts[np.where(unique == mode)[0][0]]
    idx = (np.abs(counts[0:loc] - halfmax)).argmin()
    Q1 = unique[idx]
    idx = (np.abs(counts[loc+1::] - halfmax)).argmin() + loc + 1
    Q3 = unique[idx]
    IQR = Q3 - Q1
    upper_limit = np.floor(Q3 + 1.5*IQR)
    lower_limit = np.ceil(Q1 - 1.5*IQR)
    upper_idx = (np.abs(unique - upper_limit)).argmin()
    lower_idx = (np.abs(unique - lower_limit)).argmin()
    upper_percent = np.sum(counts[upper_idx::]) / np.sum(counts)
    lower_percent = np.sum(counts[0:lower_idx+1]) / np.sum(counts)
    return upper_percent, lower_percent

def changeExposureTime():
    global exposure_time
    exposure_time = float(exposure_box.get())*1000

def changeFrameRate():
    global frame_rate
    frame_rate = float(frame_entry.get())

def getFolderPath():
    folder_selected = filedialog.askdirectory()
    folderPath.set(folder_selected)
    dir_entry.configure(state=NORMAL)
    dir_entry.delete(0, END)
    dir_entry.insert(0, folderPath.get())
    dir_entry.configure(state=DISABLED)

def camera_fcn_left():
    global motor_arduino
    if (com_motor.get() == 'Select a COM Port') or (com_motor.get() == "No COM Port Available"):
        showerror(title="Error", message=com_motor.get())
    else:
        if (value_inside.get() == "Left Eye" or (value_inside.get() == "Right Eye")):
            motor_arduino.write("A\n".encode('utf-8'))
            motor_arduino.flush()

def camera_fcn_right():
    global motor_arduino
    if (com_motor.get() == 'Select a COM Port') or (com_motor.get() ==  "No COM Port Available"):
        showerror(title="Error", message=com_motor.get())
    else:
        if (value_inside.get() == "Left Eye" or (value_inside.get() == "Right Eye")):
            motor_arduino.write("B\n".encode('utf-8'))
            motor_arduino.flush()

def mirror_fcn_cw():
    global motor_arduino
    if (com_motor.get() == 'Select a COM Port') or (com_motor.get() ==  "No COM Port Available"):
        showerror(title="Error", message=com_motor.get())
    else:
        if (value_inside.get() == "Left Eye" or (value_inside.get() == "Right Eye")):
            motor_arduino.write("C\n".encode('utf-8'))
            motor_arduino.flush()

def mirror_fcn_ccw():
    global motor_arduino
    if (com_motor.get() == 'Select a COM Port') or (com_motor.get() == "No COM Port Available"):
        showerror(title="Error", message=com_motor.get())
    else:
        if (value_inside.get() == "Left Eye" or (value_inside.get() == "Right Eye")):
            motor_arduino.write("D\n".encode('utf-8'))
            motor_arduino.flush()

def laser_fcn_cw():
    global motor_arduino
    if (com_motor.get() == 'Select a COM Port') or (com_motor.get() == "No COM Port Available"):
        showerror(title="Error", message=com_motor.get())
    else:
        if (value_inside.get() == "Left Eye" or (value_inside.get() == "Right Eye")):
            motor_arduino.write("E\n".encode('utf-8'))
            motor_arduino.flush()

def laser_fcn_ccw():
    global motor_arduino
    if (com_motor.get() == 'Select a COM Port') or (com_motor.get() == "No COM Port Available"):
        showerror(title="Error", message=com_motor.get())
    else:
        if (value_inside.get() == "Left Eye" or (value_inside.get() == "Right Eye")):
            motor_arduino.write("F\n".encode('utf-8'))
            motor_arduino.flush()

def camera_calibrate():
    global streaming
    global cam_pos_check
    if streaming:
        cam_pos_check = True

def eyeSelection(event):
    if (com_motor.get() == 'Select a COM Port') or (com_motor.get() == "No COM Port Available"):
        showerror(title="Error", message=com_motor.get())
    else:
        if (value_inside.get() == "Left Eye"):
            motor_arduino.write("L\n".encode('utf-8'))
            motor_arduino.flush()
        elif (value_inside.get() == "Right Eye"):
            motor_arduino.write("R\n".encode('utf-8'))
            motor_arduino.flush()

def motorPortSelection(event):
    global motor_arduino
    global motor_arduino_ready
    motor_arduino = serial.Serial(port=com_motor.get(), baudrate=9600, timeout=.1)
    while not motor_arduino_ready:
        arduino_read = motor_arduino.readline().decode('utf-8').rstrip()
        if arduino_read == "<Arduino is ready>":
            motor_arduino_ready = True
            motor_arduino.write("L\n".encode('utf-8'))
            motor_arduino.flush()

def pressurePortSelection(event):
    global pressure_arduino
    global pressure_arduino_ready
    pressure_arduino = serial.Serial(port=com_pressure.get(), baudrate=9600, timeout=.1)
    Thread(target=pressure_read).start()

def pressure_read():
    global pressure_arduino
    global LED
    while True:
        arduino_read = pressure_arduino.readline().decode('utf-8').rstrip()
        try:
            transducer_IOP.configure(state=NORMAL)
            transducer_IOP.delete(0, END)
            transducer_IOP.insert(0, float(arduino_read)) 
            transducer_IOP.configure(state=DISABLED)

            if manometer_setpoint.get().replace(".", "").isnumeric():
                if (abs(float(manometer_setpoint.get()) - float(arduino_read)) <=1) and (not LED):
                    in_range.to_green(on=True)
                    LED = True
                elif (abs(float(manometer_setpoint.get()) - float(arduino_read)) > 1) and (LED):
                    in_range.to_grey()
                    LED = False
            elif LED:
                in_range.to_grey()
                LED = False
        except ValueError:
            pass  

def convertHeight():
    if manometer_height_entry.get().replace(".", "").isnumeric():
        height = float(manometer_height_entry.get())
        manometer_presure = 1*height*9.81 # TODO: check Bernoulli equation
        manometer_setpoint.configure(state=NORMAL)
        manometer_setpoint.delete(0, END)
        manometer_setpoint.insert(0, manometer_presure) 
        manometer_setpoint.configure(state=DISABLED)
    else:
        showerror(title="Error", message="Invalid Manometer Height")

def exitWindow():
    global streaming
    global pump
    pump = -1
    streaming = False

    if dev_num != 0:
        # close stream
        cam.stream_off()

        # close device
        cam.close_device()
   
    # close GUI
    root.destroy()
    root.quit()  

root = Tk()
root.title('forMED Technologies')
root.resizable(width=True, height=True)
root.config(bg="skyblue")

################## LIVE FEED ##################
video_frame = Frame(root, width=720, height=540, bg='gray')
video_frame.grid(row=0, column=1, padx=10, pady=5, rowspan=540, columnspan=720)

l1 = Label(video_frame)
l1.grid(row=0, column=0, padx=5, pady=5)
################## LIVE FEED ##################

############### CAMERA SETTINGS ###############
video_control_frame = Frame(root, width=200, height=400)
video_control_frame.grid(row=0, column=0, padx=10, pady=5)

video_label = Label(video_control_frame, text="Camera Settings")
video_label.grid(row=0, column=0, padx=5, pady=5)

video_buttons = Frame(video_control_frame, width=180, height=185)
video_buttons.grid(row=1, column=0, padx=5, pady=5)

start_stop_cam_btn = Button(video_buttons, text='Start/Stop Camera', 
                        relief=RAISED, command=startStopCamera)
start_stop_cam_btn.grid(row=0, column=0, padx=5, pady=5)

exposure_text = Label(video_buttons, text='Exposure (ms)')
exposure_text.grid(row=1, column=0, padx=5, pady=5)

var = StringVar(root)
var.set("60")
exposure_box = Spinbox(video_buttons, from_=0.02, to=1000, command=changeExposureTime, 
                       textvariable=var)
exposure_box.grid(row=2, column=0, padx=5)

frame_text = Label(video_buttons, text="Frame Rate (fps)")
frame_text.grid(row=3, column=0, padx=5, pady=5)

frame_var = StringVar(root)
frame_var.set("15")
frame_entry = Spinbox(video_buttons, from_=1, to=60, command=changeFrameRate,
                      textvariable=frame_var)
frame_entry.grid(row=4, column=0, padx=5)
############### CAMERA SETTINGS ###############

############# RECORDING SETTINGS ##############
video_save_frame = Frame(root, width=200, height=400)
video_save_frame.grid(row=1, column=0, padx=10, pady=5)

record_label = Label(video_save_frame, text="Recording Settings")
record_label.grid(row=0, column=0, padx=5, pady=5, columnspan=3)

folderPath = StringVar()
dir_text = Label(video_save_frame, text='Directory')
dir_text.grid(row=1, column=0, pady=5, sticky=W)

dir_entry = Entry(video_save_frame, state=DISABLED)
dir_entry.grid(row=1, column=1, pady=5, columnspan=2)

dir_btn = Button(video_save_frame, text="...", command=getFolderPath)
dir_btn.grid(row=1, column=3, pady=5, padx=5)

numFrames_text = Label(video_save_frame, text='# of Frames')
numFrames_text.grid(row=2, column=0, columnspan=4)

numFrames_entry = Entry(video_save_frame)
numFrames_entry.grid(row=3, column=0, pady=5, columnspan=4)

pressure_text = Label(video_save_frame, text="IOP (mmHg)")
pressure_text.grid(row=4, column=0, columnspan=4)

pressure_entry = Entry(video_save_frame)
pressure_entry.grid(row=5, column=0, pady=5, columnspan=4)

record_btn=Button(video_save_frame, text='Record Speckle', 
                  relief=RAISED,command=recordSpeckleFrames)
record_btn.grid(row=6, column=0, padx=5, pady=5, columnspan=2)

record_noise_btn=Button(video_save_frame, text='Record Noise', 
                  relief=RAISED,command=recordNoiseFrames)
record_noise_btn.grid(row=6, column=2, padx=5, pady=5, columnspan=2)

roi_text = Label(video_save_frame, text="ROI")
roi_text.grid(row=7, column=0, padx=5, columnspan=4)

roi_btn = Button(video_save_frame, text="Select ROI", 
                 relief=RAISED, command=start_thread)
roi_btn.grid(row=8, column=0, padx=5, pady=5, columnspan=4)
############# RECORDING SETTINGS ##############

########## CAMERA CONTROL SETTINGS ############
camera_control_frame = Frame(root, width=200, height=400)
camera_control_frame.grid(row=541, column=1, padx=10, pady=5)

camera_label = Label(camera_control_frame, text="Camera Control")
camera_label.grid(row=0, column=0, padx=5, pady=5, columnspan=2)

camera_left_btn=Button(camera_control_frame, text='Left',
          relief=RAISED,command=camera_fcn_left)
camera_left_btn.grid(row=1, column=0, padx=5, pady=5)

camera_right_btn=Button(camera_control_frame, text='Right',
          relief=RAISED,command=camera_fcn_right)
camera_right_btn.grid(row=1, column=1, padx=5, pady=5)

camera_cal = Button(camera_control_frame, text='Check Camera Position', 
                          relief=RAISED, command=camera_calibrate)
camera_cal.grid(row=2, column=0, padx=5, pady=5, columnspan=2)
########## CAMERA CONTROL SETTINGS ############

########## MIRROR CONTROL SETTINGS ############
mirror_control_frame = Frame(root, width=200, height=400)
mirror_control_frame.grid(row=541, column=2, padx=10, pady=5)

mirror_label = Label(mirror_control_frame, text="Mirror Control")
mirror_label.grid(row=0, column=0, padx=5, pady=5, columnspan=2)

mirror_cw_btn=Button(mirror_control_frame, text='CW',
          relief=RAISED,command=mirror_fcn_cw)
mirror_cw_btn.grid(row=1, column=0, padx=5, pady=5)

mirror_ccw_btn=Button(mirror_control_frame, text='CCW',
          relief=RAISED,command=mirror_fcn_ccw)
mirror_ccw_btn.grid(row=1, column=1, padx=5, pady=5)
########## MIRROR CONTROL SETTINGS ############

########## LASER CONTROL SETTINGS #############
laser_control_frame = Frame(root, width=200, height=400)
laser_control_frame.grid(row=541, column=3, padx=10, pady=5)

laser_label = Label(laser_control_frame, text="Laser Control")
laser_label.grid(row=0, column=0, padx=5, pady=5, columnspan=2)

laser_cw_btn=Button(laser_control_frame, text='CW',
          relief=RAISED,command=laser_fcn_cw)
laser_cw_btn.grid(row=1, column=0, padx=5, pady=5)

laser_ccw_btn=Button(laser_control_frame, text='CCW',
          relief=RAISED,command=laser_fcn_ccw)
laser_ccw_btn.grid(row=1, column=1, padx=5, pady=5)
########## LASER CONTROL SETTINGS #############

########## LIVE SPECKLE CONTRAST ##############
speckle_contrast_frame = Frame(root, width=200, height=400)
speckle_contrast_frame.grid(row=541, column=4, padx=10, pady=5)

speckle_text=Label(speckle_contrast_frame, text="Speckle Contrast")
speckle_text.grid(row=0, column=0, padx=5, pady=5)

speckle_entry = Entry(speckle_contrast_frame, state=DISABLED)
speckle_entry.grid(row=1, column=0, padx=5, pady=5)
########## LIVE SPECKLE CONTRAST ##############

############## EYE SELECTION ##################
eye_select_frame = Frame(root, width=200, height=400)
eye_select_frame.grid(row=2, column=0, padx=5, pady=5)

value_inside = StringVar(eye_select_frame)
value_inside.set("Left Eye")
left_or_right = OptionMenu(eye_select_frame, value_inside, 
                           *["Left Eye", "Right Eye"], command=eyeSelection)
left_or_right.grid(row=0, column=0, padx=5, pady=5, columnspan=3)
############## EYE SELECTION ##################

################## EXIT #######################
exit_frame = Frame(root, width=200, height=400)
exit_frame.grid(row=3, column=0, padx=5, pady=5)

exit_btn=Button(exit_frame,fg='white',bg='red',activebackground='white',
          activeforeground='red',text='Exit',
          relief=RAISED,command=exitWindow)
exit_btn.grid(row=0, column=0, padx=5, pady=5)
################## EXIT #######################

################ COM PORT #####################
com_frame = Frame(root, width=200, height=400)
com_frame.grid(row=541, column=0, padx=5, pady=5)

com_motor = StringVar(com_frame)
#com_pump = StringVar(com_frame)
com_pressure = StringVar(com_frame)
ports = [p.device for p in serial.tools.list_ports.comports()]

motor_text = Label(com_frame, text='Motor COM Port')
motor_text.grid(row=0, column=0, padx=5, pady=5)
#pump_text = Label(com_frame, text='Pump COM Port')
#pump_text.grid(row=2, column=0, padx=5, pady=5)
pressure_text = Label(com_frame, text='Pressure Transducer COM Port')
pressure_text.grid(row=2, column=0, padx=5, pady=5)
if ports != []:
    com_motor.set("Select a COM Port")
    com_motor_options = OptionMenu(com_frame, com_motor, 
                        *ports, command=motorPortSelection)
    com_motor_options.grid(row=1, column=0, padx=5)

    com_pressure.set("Select a COM Port")
    com_pressure_options = OptionMenu(com_frame, com_pressure, 
                    *ports, command=pressurePortSelection)
    com_pressure_options.grid(row=3, column=0, padx=5)
else:
    com_motor.set("No COM Port Available")
    com_motor_options = OptionMenu(com_frame, com_motor, '')
    com_motor_options.configure(state=DISABLED)
    com_motor_options.grid(row=1, column=0, padx=5)

    #com_pump.set("No COM Port Available")
    #com_pump_options = OptionMenu(com_frame, com_pump, '')
    #com_pump_options.configure(state=DISABLED)
    #com_pump_options.grid(row=3, column=0, padx=5)

    com_pressure.set("No COM Port Available")
    com_pressure_options = OptionMenu(com_frame, com_pressure, '')
    com_pressure_options.configure(state=DISABLED)
    com_pressure_options.grid(row=5, column=0, padx=5)
################ COM PORT #####################

############# CALIBRATION CHECKS ##############
cal_frame = Frame(root, width=200, height=400)
cal_frame.grid(row=541, column=5, padx=10, pady=5)

laser_ready_bool = BooleanVar()
laser_ready_bool.set(False)

cam_check_bool = BooleanVar()
cam_check_bool.set(False)

cal_check_bool = BooleanVar()
cal_check_bool.set(False)
on_color = "green"
off_color = "red"

laser_ready = Checkbutton(cal_frame, text="Laser Ready?", variable=laser_ready_bool, 
                           onvalue=1, offvalue=0, fg=off_color, state=DISABLED)
laser_ready.grid(row=1, column=0, padx=5, pady=5)

cam_check = Checkbutton(cal_frame, text="Eye in FOV?", variable=cam_check_bool, 
                           onvalue=1, offvalue=0, fg=off_color, state=DISABLED)
cam_check.grid(row=0, column=0, padx=5, pady=5)

cal_check = Checkbutton(cal_frame, text="Calibrated?", variable=cal_check_bool, 
                           onvalue=1, offvalue=0, fg=off_color, state=DISABLED)
cal_check.grid(row=2, column=0, padx=5, pady=5)
############# CALIBRATION CHECKS ##############

############# TRANSDUCER SETTINGS #############
pressure_frame = Frame(root, width=200, height=400)
pressure_frame.grid(row=541, column=6, padx=10, pady=5)

pressure_text = Label(pressure_frame, text="Transducer Settings")
pressure_text.grid(row=0, column=0, padx=5, pady=5, columnspan=2)

manometer_height_text = Label(pressure_frame, text="Manometer Height (cm)")
manometer_height_text.grid(row=1, column=0, padx=5, pady=5)

manometer_height_entry = Entry(pressure_frame)
manometer_height_entry.grid(row=2, column=0, padx=5)

manometer_pressure_text = Label(pressure_frame, text="Manometer IOP (mmHg)")
manometer_pressure_text.grid(row=1, column=1, padx=5, pady=5)

manometer_setpoint = Entry(pressure_frame, state=DISABLED)
manometer_setpoint.grid(row=2, column=1, padx=5, pady=5)

manometer_btn = Button(pressure_frame, text='Set Height',
          relief=RAISED, command=convertHeight)
manometer_btn.grid(row=3, column=0, padx=5)

transducer_pressure_text = Label(pressure_frame, text="Transducer IOP (mmHg)")
transducer_pressure_text.grid(row=3, column=1, padx=5)

transducer_IOP = Entry(pressure_frame, state=DISABLED)
transducer_IOP.grid(row=4, column=1, padx=5, pady=5)

in_range = tk_tools.Led(pressure_frame, size=50)
in_range.grid(row=4, column=0, padx=5, pady=5, rowspan=2)
in_range.to_grey()
############# TRANSDUCER SETTINGS #############
root.mainloop()
