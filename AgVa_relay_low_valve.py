#!/user/bin/env python
import datetime
from SM9541 import *
from ctypes import *
import csv
import numpy as np
import Adafruit_ADS1x15
import os
import serial
from time import time,sleep
from smbus import SMBus
from math import *
import RPi.GPIO as GPIO
air=70
sensor = SM9541()
a = 12
toggle_switch = "0"
calib_flag = 0
rise_time = 0
rise_flag = '0'
primitive_inhale_time = 2.0
# -----------------------------------------
IHold = '0'
EHold = '0'
init_flow_int = 0.0
IH_time = 0.0
EH_time = 0.0
starting_flag = 0
change_setting = '0'
trigflow_comp = 0
comp_flag = '0'
running_avg = 0.0
volume_comp = 0
inhale_array = []
time_elapsed_exhale_flow = 0
time_elapsed_inhale_flow = 0
ADDRESS_ABP=0x28
ADDRESS_SDP=0x25
# PIP, VTi, PEEP, RR, Insp time, Trif flow, Plat, inhale_time
# -----------------------------------------
#ser = serial.Serial(port='/dev/ttyAMAO',baudrate=9600,parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE,bytesize=serial.EIGHTBITS,timeout=1)
backup_flag = 0
BIPAP_backup_flag = 0
try :
    ser = serial.Serial(
        port='/dev/ttyS0',
        baudrate = 9600,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=1
)
except:
    print('BT initilaization error')
# Volume Calculation Constants
sensor_found_P = "MPX"
while(os.path.isfile('/home/pi/AgVa_5.0/PRESSURE') == False):
    try:
	os.system("sudo python /home/pi/AgVa_5.0/configure.py")
    except:
	print("unable to run configure file")
    sleep(2)
try:
    f = open("/home/pi/AgVa_5.0/PRESSURE","r")
    data = f.read()
    if(data == "MPRLS0001PG00001C"):
	sensor_found_P = "MPR"
    elif(data == "AMS5812-0015D"):
	sensor_found_P = "AMS"
    elif(data == "SM5852-015D-3-LR"):
	sensor_found_P = "SMI"
    elif(data == "MPX5010"):
	sensor_found_P = "MPX"
except:
	sensor_found_P = "mpx"
sensor_found_P = "AMS"
area_1 = 0.000950625 #151755924         #in metres square
area_2 = 0.000085785
dt=0.004
rho=1.225
try:
    pressure_AMS = CDLL("/home/pi/AgVa_5.0/AMS.so")
    pressure_AMS.start_bus()
except:
    print("unable to start AMS sensor")
volume_divisor=((1/(area_2**2))-(1/(area_1**2)))
#-----------------------------------------
prev_mode = 11
SDP_last = -1.0
compliance = 0
pressure_low_count = 0
time_elapsed_inhale = 0
patient_trigger_flow = 0
compliance_array = []
BIPAP_trigger = 0
leak_comp_flag = '0'
CPAP_trigger = 0
lock = 0
ratio =0
back_backup = 0
motor_factor = 0.5
diff = 0
trigflow = 3.0          #in litres per minute
start_time =0
P_plat = 18.0
P_plat_value = 18.0
pump_pressure_array = [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63,64,65,66,67,68,69,70,71,72,73,74,75,76,77,78,79,80,81,82,83,84,85,86,87,88,89,90,91,92,93,94,95,96,97,99,100]
#P_plat_array = [-0.1,-0.1,-0.1,-0.1,-0.1,-0.1,-0.1,-0.1,-0.1,-0.1,-0.1,-0.1,-0.1,-0.1,-0.09,-0.09,-0.04,0.06,0.19,0.38,0.59,0.85,1.16,1.57,1.84,2.3,2.66,3.13,3.77,4.34,4.86,5.46,6.04,6.64,7.12,7.83,8.28,8.97,9.52,10.15,10.84,11.51,11.97,12.48,13.34,13.57,14.24,14.85,15.44,15.72,16.29,16.71,17.38,17.84,18.3,18.55,19.25,19.8,20.26,21.17,21.54,22.24,22.95,23.41,24.23,25.13,25.63,26.82,27.09,28.67,29.25,30.38,31.44,32.36,33.4,34.29,36.08,36.34,38.04,39.12,40.24,41.7,42.15,43.31,44.88,45.33,47.02,48.94,49.91,49.88,50.74,50.83,49.72,49.87,49.78,50.03,50.06,50.01,50.04,50.41]
#P_plat_array = [0.4035,0.4075,0.2085,0.2145,0.213,0.2315,0.2225,0.224,0.221,0.21,0.2075,0.205,0.2075,0.209,0.217,0.2545,0.334,0.4505,0.6845,0.941,1.1755,1.477,1.868,2.3745,2.7425,3.2655,3.709,4.063,4.475,5.0825,5.5235,6.0795,6.426,7.1935,7.693,8.0885,8.838,9.5835,10.091,10.041,10.752,11.366,11.7645,12.533,12.886,13.483,14.1725,14.2555,12.9815,13.075,14.7235,15.0265,15.9435,16.078,17.132,18.14,19.0845,19.406,19.8975,20.914,21.6035,22.662,23.4835,24.5605,25.004,25.3285,26.657,27.3145,28.805,29.971,30.381,32.247,32.4725,33.264,35.2145,36.1555,36.6975,38.079,38.535,39.966,40.505,42.1015,43.302,44.8605,46.6285,48.1345,48.8025,49.9935,50.471,50.836,50.5825,51.134,50.8335,50.4995,51.093,50.9345,50.0425,50.469,50.442,51.0895]
P_plat_array = [0.08,0.08,0.08,0.08,0.34,0.47,0.66,1.06,1.42,1.76,2.09,2.67,3.14,3.71,4.38,5.0,5.74,6.57,7.3,8.16,8.97,9.58,10.0,10.53,10.95,11.44,11.99,12.41,13.0,13.56,14.09,14.61,14.98,15.46,16.02,16.58,17.24,17.81,18.41,18.93,19.51,20.05,20.72,21.08,21.85,22.54,23.08,23.71,24.27,24.78,25.39,26.05,26.65,27.34,27.74,28.75,29.25,29.74,30.5,31.21,32.0,32.52,33.23,34.14,34.43,35.29,35.91,36.63,37.35,38.06,38.74,39.6,40.37,41.01,41.66,42.39,43.21,43.76,44.76,45.26,45.77,46.71,47.44,47.89,49.11,50.26,51.37,52.61,53.58,54.85,55.81,57.33,58.37,59.5,60.61,62.24,63.13,64.42,65.83,66.78]
PIP = 20               #in cmh2o
pump_pressure = 0 # variable initialzed values changes as according to P_plat
time_elapsed = 0
inhale_time = 2.0       #in seconds
peep_hole = [0,2,4,8.9,10.3,13,16.2,20,23,24,27]
indiff = 0
#peep_array_hole = [-0.08,-0.08,-0.08,-0.07,-0.08,-0.07,-0.07,-0.07,-0.08,-0.08,-0.08,-0.08,-0.08,-0.09,-0.07,-0.07,-0.03,0.05,0.18,0.37,0.61,1.04,1.44,1.88,2.23,2.72,3.19,3.76,4.12,4.63,5.13,5.58,6.06,6.62,7.03,7.63,8.13,8.75,9.23,9.64,10.33,10.75,11.13,11.79,12.16,12.64,13.1,13.79,14.45,14.91,15.21,15.85,16.3,16.66,17.36,17.78,18.29,18.66,19.29,20.24,20.81,21.16,22.02,22.71,23.13,23.87,25.05,26.29,27.43,28.08,28.95,30.18,31.28,31.44,33.21,34.28,35.04,36.14,36.97,38.87,39.63,40.93,42.14,43.38,44.47,45.72,46.55,47.71,48.99,49.79,49.37,49.58,49.68,49.67,49.94,49.47,50.14,50.14,49.62,49.56]
#peep_array = [-0.1,-0.1,-0.1,-0.1,-0.1,-0.1,-0.1,-0.09,-0.1,-0.1,-0.1,-0.1,-0.1,-0.1,-0.09,-0.08,-0.05,0.02,0.14,0.28,0.44,0.57,0.78,0.94,1.17,1.31,1.55,1.69,1.98,2.16,2.35,2.53,2.77,2.9,3.12,3.34,3.58,3.8,3.97,4.23,4.37,4.64,4.88,5.04,5.32,5.7,5.95,6.17,6.54,6.77,7.32,7.54,7.84,8.23,8.52,9.04,9.47,9.72,10.25,10.93,11.31,11.59,12.36,12.59,13.18,13.66,14.16,14.6,15.19,15.87,16.17,16.51,17.41,17.62,18.4,18.96,19.2,20.03,20.56,20.82,20.95,21.22,22.58,21.35,21.75,23.47,24.85,26.01,26.54,26.91,26.04,26.32,26.34,26.61,26.38,26.49,26.55,26.2,26.48,26.83]
peep_array_hole = [0.08,0.08,0.08,0.08,0.34,0.47,0.66,1.06,1.42,1.76,2.09,2.67,3.14,3.71,4.38,5.0,5.74,6.57,7.3,8.16,8.97,9.58,10.0,10.53,10.95,11.44,11.99,12.41,13.0,13.56,14.09,14.61,14.98,15.46,16.02,16.58,17.24,17.81,18.41,18.93,19.51,20.05,20.72,21.08,21.85,22.54,23.08,23.71,24.27,24.78,25.39,26.05,26.65,27.34,27.74,28.75,29.25,29.74,30.5,31.21,32.0,32.52,33.23,34.14,34.43,35.29,35.91,36.63,37.35,38.06,38.74,39.6,40.37,41.01,41.66,42.39,43.21,43.76,44.76,45.26,45.77,46.71,47.44,47.89,49.11,50.26,51.37,52.61,53.58,54.85,55.81,57.33,58.37,59.5,60.61,62.24,63.13,64.42,65.83,66.78]
peep_array = [0.06,0.06,0.07,0.07,0.28,0.39,0.53,0.68,0.84,0.98,1.15,1.37,1.57,1.8,2.03,2.29,2.57,2.88,3.12,3.38,3.75,3.99,4.16,4.32,4.52,4.69,4.88,5.05,5.23,5.41,5.65,5.81,6.0,6.22,6.41,6.64,6.86,7.04,7.27,7.52,7.71,7.91,8.18,8.38,8.6,8.88,9.12,9.35,9.58,9.8,10.03,10.25,10.47,10.69,10.83,11.13,11.35,11.53,11.75,12.03,12.26,12.43,12.68,12.91,13.17,13.38,13.6,13.81,14.04,14.24,14.49,14.72,14.9,15.07,15.34,15.55,15.81,16.08,16.27,16.54,16.76,16.97,17.14,17.43,17.63,17.97,18.32,18.68,19.0,19.34,19.77,20.11,20.44,20.79,21.21,21.55,21.92,22.32,22.58,23.15]
threshold = 1
flag=0
peep=0
peep_open = 0
BPM=15                #breaths per minute
cycle_time=60/BPM
MPR_data = [0x00,0x00]
current_time=0
VTi_max=500
ABP_flag = 0
ABP_fail = 0
SDP_fail = 0
peep_pwm = 0
inhale_loop = "0"
#-----------------------------------------
peep_val = 0
adc = Adafruit_ADS1x15.ADS1115()
GAIN = 1
prev_FiO2 = 0
TITOT = 0
#-----------------------------------------
peep_factor = 2.0
GPIO.setmode(GPIO.BOARD)
GPIO.setup(18,GPIO.OUT)
GPIO.setup(35,GPIO.OUT)
#GPIO.setup(21,GPIO.OUT)
motor_1=GPIO.PWM(18,50)
GPIO.setup(40,GPIO.OUT)
sol = GPIO.PWM(40, 1000)
sol.start(85)
GPIO.setup(33, GPIO.OUT)
GPIO.output(33, GPIO.LOW)
GPIO.setup(19,GPIO.OUT)
exp_flow_array = [] 			#motor_2=GPIO.PWM(18,50)
exp_press_array = []
motor_1.start(0)
initial_pump_pressure = 0
GPIO.setup(37, GPIO.OUT)
GPIO.setup(7, GPIO.OUT)
peep_first = 0
#-------------------------------
FiO2_array = []
ABP_set = 0
plateau_pressure = 0
setting_set = 0
exhale_time_last = 0
TOT_last = 0
mode_set = 0
SDP_set = 0
time_error_set = 0
ratio_set = 0
patient_set = 0
#-------------------------------
time_error = 0
time_error_last = 0
#------------------------------------------
thread_mode_status=False
flow_error_compensation = 0
flow_error_compensation_array = [0.9515,0.9335,0.976,0.9875,0.958,1.03,0.9155,1.048,0.976,1.001,1.03,1.03,1.012,1.8795,1.012,1.819,1.6375,1.1275,1.663,2.1525,2.298,3.2155,3.9665,3.942,4.6765,5.19,5.379,5.9965,6.7935,6.8405,7.766,8.252,8.14,8.8585,9.1455,10.033,10.2025,10.248,10.7025,9.9325,10.78,11.4935,12.2505,11.7475,11.178,13.5785,14.2295,13.196,13.2685,13.6155,13.3595,14.4435,14.446,14.17,16.0745,15.0535,15.3115,16.348,15.6795,17.3875,18.088,18.334,18.5115,18.6905,18.4465,19.9345,20.0825,20.2425,20.3805,21.626,20.8635,21.603,20.884,22.077,21.8985,22.268,23.038,22.453,24.271,24.5925,24.519,25.0515,25.8975,26.573,26.4235,26.588,27.2285,25.0765,27.5635,27.054,27.2485,26.941,28.658,28.8155,26.5745,26.7815,27.5405,26.3745,27.1165,27.5255]
data= ''
#------------------------------------------
flow_plat = 40
loop = 1
#------------------------------------------
volume_flag = 0
volume=0
first = 0
#------------------------------------------
previous_data = ''
patient_status = 0
pump_pressure_high = 0
P_plat_high = 0
P_plat_low = 0
pump_pressure_low = 0
volume_count = 0
peep_count = 0
pressure_count = 0
#--------------------------------------------
RR=[]
RR_time=0
SDP_flag = 0
#-----------------------------------------
MVi_array=[]
MVi=0
MVe_array=[]
MVe=0
#------------------------------------------
volume_peak_inhale=0
volume_peak_exhale=0
trigger = '0'
Pmean_array=[]
Pmean=0
clock_t2 = 0
def buzzer(position, bit):
    try:
	f = open('/home/pi/AgVa_5.0/buzzer.txt', 'r')
	data = f.readline()
	f.close()
#	print(data)
#	print(position)
#	print(bit)
	if(len(data) > 10):
	    data = data.split(',')
	    data = map(int,data)
	    data[position] = bit
	    data = map(str,data)
	    data = ','.join(data)
	else:
	    data = '0,0,0,0,0,0,0,0,0,0'
#	print(data)
        try:
            f = open('/home/pi/AgVa_5.0/buzzer.txt','w')
	    f.write(str(data))
            f.close()
        except:
            print('unable to write to file')
    except:
	print('unable to read file')
def hold_check():
    global IHold,EHold, IH_time, EH_time
    try:
	f = open("/home/pi/AgVa_5.0/hold.txt")
	data = f.readline()
	f.close()
#	print(data[0:3])
	if(data[0:2] == 'IH'):
	    IHold = '1'
	    IH_time = float(data[2:5])
	elif(data[0:2] == 'EH'):
	    EHold = '1'
	    EH_time = float(data[2:5])
	try:
	    f = open("/home/pi/AgVa_5.0/hold.txt","w")
	    f.write("x")
	    f.close()
	except:
	    print("unable to write hold file")
    except:
	print("hold file doesnt exist")
def find_nearest(array, value):
    array = np.asarray(array)
    idx = (np.abs(array - value)).argmin()
    return array[idx]
def toggle():
    global toggle_switch
#    print("printing the tofle switchhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhh")
    try:
        f = open('/home/pi/AgVa_5.0/mode.txt',"r")
	data = f.readline()
	f.close()
	return data[2:3]
    except:
	print("error reading file")
        return toggle_switch
def leak_comp():
    try:
	f = open("/home/pi/AgVa_5.0/flowComp.txt","r")
	data = f.readline()
	f.close()
#	print("THE DATA WHAT WE RAED IS")
#	print(data)
	return data
    except:
	return "0"
class Ventilator():
#    global clock_t2
    def __init__(self):
        #-------------------------------------
        #BUS READ AND WRITE
	try:
            self.bus = SMBus(1)  # default i2c bus
  #          self.bus.write_i2c_block_data(ADDRESS_SDP, 0x3F, [0xF9])
            sleep(0.8)
    #        self.bus.write_i2c_block_data(ADDRESS_SDP, 0x36, [0x03])
     #       sleep(0.1)
            print("INITAILIZED RUN")
        #-------------------------------------
	except:
#	    self.SDP_initialization()
	    motor_1.ChangeDutyCycle(0)

#    def SDP_initialization(self):
 #       try:
  #          self.bus.write_i2c_block_data(ADDRESS_SDP, 0x3F, [0xF9])
   #         sleep(0.8)
    #        self.bus.write_i2c_block_data(ADDRESS_SDP, 0x36, [0x03])
     #       sleep(0.1)
      #  except:
       #     print('failed----')
        #    return 0
    def power(self):
	try:
	    battery = adc.read_adc(0,1)
	    oxygen = adc.read_adc_difference(3, gain=1)
	    f = open("/home/pi/AgVa_5.0/power.txt","w")
	    f.write(str(battery)+","+str(oxygen))
	    f.close()
	except:
	    return 0
    #-----------------------------------------
    def FiO2(self):
	global prev_FiO2
        try:
            f = open('/home/pi/AgVa_5.0/FiO2.txt', "r")
            FiO2_val = f.readline()
	    f.close()
	    prev_FiO2 = FiO2_val
            return FiO2_val
        except:
            return prev_FiO2
#-------------------------------------------
    #ABP presure sensor i2c read
    def ABP_pressure(self):
	global ABP_set, ABP_flag,MPR_data,sensor_found_P
        try:
	    if(sensor_found_P == "MPR"):
	        self.bus.write_i2c_block_data(0x18,0xAA,MPR_data)
	        sleep(0.005)
	        val = self.bus.read_i2c_block_data(0x18,0)
	        byte = val[1] << 8 | val[2]
	        byte = byte << 8 | val[3]
	        press_cmH2O = (((byte - 3355443.0)/(13421772.0 - 3355443.0))*70.307)
	    elif(sensor_found_P == "MPX"):
	        reading = adc.read_adc(1, gain= 2/3)
                voltage = (6144 * reading)/32678.0
    	        pressure = float((voltage - 250.0)/450.0)
    	        press_cmH2O = (pressure * 10.1972)*1.056
	    elif(sensor_found_P == "AMS"):
                value = pressure_AMS.pressure()
                press_cmH2O = (((value - 3277.0) / ((26214.0) / 1.5)))*70.30;
	    elif(sensor_found_P == "SMI"):
                LSB = self.bus.read_byte_data(0x5f,128)
                MSB = self.bus.read_byte_data(0x5f, 129)
                pressRaw = MSB << 8
                pressLSB = LSB << 2
                pressRaw = pressRaw | pressLSB
                pressRaw = pressRaw >> 2
                if(pressRaw > 2045):
                    press_cmH2O = (np.interp(pressRaw, [2045,3671],[0.0,1.5])*70.307)
                else:
                    press_cmH2O = 0.0
	    else:
		press_cmH2O = 0.0
	    if(press_cmH2O  < 80 and ABP_set ==1):
		ABP_set = 0
		buzzer(8,0)
		try:
		    ser.write('ACK21')
		except:
		    print('not able to send data')
	    if(press_cmH2O >= 81.0 and ABP_set == 0):
		ABP_set = 1
		try:
		    ser.write('ACK11')
		except:
		    print('not able to send data')
		ABP_fail = 0
		return 0
	    ABP_fail = 0 
#	    print(press_cmH2O)
            return press_cmH2O
        except:
	    print('failed ABP')
	    if(ABP_set == 0):
		buzzer(8,1)
	        ABP_set = 1
		ABP_fail = 1
	        try:
		    ser.write('ACK11')
	        except:
		    print('not able to send data')
	    #GPIO.output(22,1)
	    return 0

	
########################################################
    #ABP presure sensor i2c read
    #----------------------------------------
    # setting file read function 
    def read_data(self):
	global peep_first,peep_factor,initial_pump_pressure, motor_factor,change_setting,primitive_inhale_time,compliance_array, compliance_flag,toggle_switch,peep_open,setting_set,peep_pwm,peep_array,P_plat_high,P_plat_low,flow_plat,previous_data,pump_pressure_low,pump_pressure_high, mode_set,inhale_time,P_plat_value,P_plat_array,peep_val,prev_mode,pump_pressure_array,BPM,PIP,VTi_max,peep,P_plat,trigflow,cycle_time,data,pump_pressure
	try:
	    f = open('/home/pi/AgVa_5.0/setting.txt',"r")
	    data = f.readline()
	    f.close()
#	    print("THE READING DATA IS")
#	    print(data)
#	    print(previous_data)
	    if(data != previous_data or change_setting == '1'):
#		change_setting = '0'
	        data_split= data.split(',')
		compliance_flag = 0
		del compliance_array[:]
#	    print(data_split)
#	    print(float(data_split[0]))
	        flow_plat = float(data_split[7])
#	        print('the recived peeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeep is')
	   # print()
	        inhale_time = float(data_split[6])
	        BPM = float(data_split[3])
	        cycle_time= (60.0/BPM)
	        PIP = float(data_split[0])
	        VTi_max = float(data_split[1])
	        peep_val = float(data_split[2])
		primitive_inhale_time = inhale_time
#		if(toggle_switch == 1):
		peep_nearest = find_nearest(peep_array_hole, peep_val)
		peep_index = peep_array_hole.index(peep_nearest)
		peep = pump_pressure_array[peep_index]
	        peep_nearest = find_nearest(peep_array,peep_val)
	        peep_index = peep_array.index(peep_nearest)
		peep_open = pump_pressure_array[peep_index]
#	        peep = pump_pressure_array[peep_index]
		peep_pwm = peep
	        P_plat = float(data_split[5])
	        trigflow = float(data_split[4])+peep_val
		if((prev_mode == 31 or prev_mode == 34) and P_plat >= 20):
		    P_plat = 20.0
#		print("TRIG FLOW SUBRACTION IS")
		peep_factor = peep_val/3
		if(peep_val > 20):
		    peep_factor = (P_plat - peep_val)/3.0
	#        trigflow = trigflow - 1.5
	        P_plat_value=find_nearest(P_plat_array,P_plat)
	        index= P_plat_array.index(P_plat_value)
	        pump_pressure = pump_pressure_array[index]
		pump_pressure_low = pump_pressure - (pump_pressure * 0.15)
		pump_pressure_high = pump_pressure + (pump_pressure * 0.15)
		if(prev_mode == 23 or prev_mode == 32 or prev_mode == 35):
		    pump_pressure_high_value = find_nearest(P_plat_array, PIP)
		    index = P_plat_array.index(pump_pressure_high_value)
		    pump_pressure_high = pump_pressure_array[index]
		    initial_pump_pressure = pump_pressure
# 		if(prev_mode == 32):
# 		    trigflow = trigflow + peep_val + 10
		if(prev_mode == 31 or prev_mode == 34):
	            peep_nearest = find_nearest(peep_array_hole,P_plat)
	            peep_index = peep_array_hole.index(peep_nearest)
		    pump_pressure = pump_pressure_array[peep_index]
		    peep_nearest = find_nearest(peep_array_hole, P_plat)
		    peep_index = peep_array_hole.index(peep_nearest)
	 	    peep = pump_pressure_array[peep_index]
		    peep_val = P_plat
		if(prev_mode == 21 or prev_mode == 22 or prev_mode == 24 or prev_mode == 25 or prev_mode == 26 or prev_mode == 35):
		    if(VTi_max <= 100):
			P_plat = 5.0 + peep_val
		    elif(VTi_max <=200):
			P_plat = 10.0 + peep_val
		    elif(VTi_max <= 300):
			P_plat = 13.0 + peep_val
		    elif(VTi_max <= 400):
			P_plat = 16 + peep_val
		    else:
			P_plat = 18 + peep_val
		    P_plat_value = find_nearest(P_plat_array, P_plat)
#		    print("???????????????????????")
#		    print(P_plat_value)
		    index = P_plat_array.index(P_plat_value)
#		    print(index)
		    pump_pressure = pump_pressure_array[index]
#		    print(pump_pressure)
		    pump_pressure_high_value = find_nearest(P_plat_array, PIP)
		    index = P_plat_array.index(pump_pressure_high_value)
		    pump_pressure_high = pump_pressure_array[index]
		    pump_pressure_low = pump_pressure -2
		if(pump_pressure_high >= 100):
		    pump_pressure_high = 100
		if(pump_pressure_low <= 0):
		    pump_pressure_low = 0
		P_plat_high = P_plat + (P_plat * 0.15)
		P_plat_low = P_plat - (P_plat * 0.15)
		if(P_plat_high >= 100):
		    P_plat_high = 100
		if(P_plat_low <= 0):
		    P_plat_low = 0
   	        peep_first = peep
		previous_data = data
	    #print(peep)
#	    print(BPM)
	        cycle_time=60.0/BPM
#		trigflow = trigflow + peep_val - 1
#	    print("TRIG FLOW IN THE MAIN LOOP IS")
#	    print(trigflow)
#	    trigflow = trigflow + peep_val
#	    print("TRIG FLOWWWWWWWWWWWWWWWWW IS")
#	    print(trigflow)
#	    print(trigflow)
#	    print(cycle_time)
#	    print('reading the data')
#	    print(data)
	    f.close()
	    change_setting = '0'
	    motor_factor = 0.5
	    if(setting_set == 1):
	        setting_set = 0
	        try:
	            ser.write('ACK25')
	        except:
		    print('unable to send data')
#	    return data
	except:
	    print('error reading file')
	    if(setting_set == 0):
		setting_set = 1
	        try:
	            ser.write('ACK15')
	        except:
		    print('unable to send data')
	try:
#	    print("yes i have got in hereeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee")
	    f = open('/home/pi/AgVa_5.0/mode.txt',"r")
	    data_read = f.readline()
#	    toggle_switch = data_read[2:3]
	    data = data_read[0:2]
#	    print("the data mode i have got in here is" + data)
	  #  data = f.readline()
	    if(mode_set == 1):
	        mode_set = 0
	        try:
	            ser.write('ACK25')
	        except:
		    print('unable to send data')
	    if(data == "11" or data == "12" or data == "13" or data == "14" or data == "21" or data == "22" or data == "23" or data == "24" or data == "25" or data == "26" or data == "31" or data == "32" or data == "33" or data == "34" or data == "35"):
	        mode_read = int(data)
	        if(mode_read != prev_mode):
		    change_setting = '1'
		    return 1
	        else: 
#		    print('i was here')
		    return 0
	    else:
		return 0
	except:
	    print("got an error here in mode fileeeeeeeeeeeeeeee")
	    if(mode_set == 0):
		mode_set = 1
	        try:
	            ser.write('ACK15')
	        except:
		    print('unable to send data')
	    print('error reading mode file')

    #------------------------------------------
    # -----------------------------------------
    # SDP810_500Pa presure sensor i2c read
    def SDP_pressure(self):
	global SDP_last,SDP_set, SDP_flag 
        try:
	    read = sensor.read_all()
	    diff_press = (read[1])*98.022
#	    read = self.bus.read_i2c_block_data(ADDRESS_SDP, 0)
 #           pressure_value = read[0] + float(read[1]) / 255
  #          diff_press=0
   #         if 0 <= pressure_value < 128:
    #            diff_press = pressure_value *255/60
     #       elif 128 < pressure_value <= 256:
      #          diff_press = -(256 - pressure_value) *255/60
       #     elif pressure_value == 258 or pressure_value == 128 :
        #        if(SDP_last > 0):
	#	    diff_press = 500
	 #       elif(SDP_last < 0):
	#	    diff_press = -500
# 	    if(diff_press > 500):
# 		diff_press = 500.0
# 	    elif(diff_press < -500):
# 		diff_press = diff_press = -500.0
	    SDP_last = diff_press
	    if(SDP_set == 1):
		SDP_set = 0
		buzzer(9,0)
	        try:
	            ser.write('ACK20')
	        except:
		    print('unable to send data')
	    SDP_flag = 0
#	    print("......................................." + str(diff) + "......" + str(pressure_value))
            return round(diff_press,2)
        except IOError:
	    print('failed')
#	    sleep(1)
#	    motor_1.ChangeDutyCycle(40)
#	    self.SDP_initialization()
	    SDP_flag = 1
	    if(SDP_set == 0):
		buzzer(9,1)
		SDP_set = 1
	        try:
	            ser.write('ACK10')
	        except:
		    print('unable to send data')
	   # GPIO.output(22,1)
	    return -9999


    #----------------------------------------------
    # Calculate Volume Flow From SDP810_500Pa Sensor
    def Flow(self):

        global volume_flag,volume,clock_t2,P_plat,flow_error_compensation,indiff,inhale_loop
        diff=self.SDP_pressure()
	if(diff == -9999):
	    print('we are in  a mess')
	    return -9999
#	print("Yoooooooooooooooooooooooooooooooo go the difffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff")
 #       print(diff)
        massFlow=1000*(sqrt((abs(diff)*2*rho)/volume_divisor))
        volFlow =( massFlow/rho)*1000
        volFlow_min=(volFlow*60)
        last = time() - first
       # volFlow = volFlow *1000
	clock_t1= time()
	clock = (clock_t1 - clock_t2)
#	print('diff is')
#	print(diff)
#	print('clock_t2 is')
#	print(clock_t2)
#	print('clock is')
#	print(clock)
#	print('We have got a P_plat of')
#	print(P_plat)
# 	if(P_plat < 10):
# 	    volFlow_min = volFlow_min - 2500
# 	elif(P_plat >= 10 and P_plat <=15):
# 	    volFlow_min = volFlow_min - 3500
# 	elif(P_plat >15 and P_plat <= 20):
# #	    print('git in here')
# 	    volFlow_min = volFlow_min - 9300
# 	elif(P_plat > 20):
# 	   volFlow_min = volFlow_min - 8700
# 	if(diff < 0):
#	    volFlow_min = -1*volFlow_min
#	    print('came i here')
#	try:
#	    flow_P_plat_value=find_nearest(P_plat_array,indiff)
#	    flow_index= P_plat_array.index(flow_P_plat_value)
#	    pump_pressure_index = pump_pressure_array[flow_index]
#	    flow_error_compensation = flow_error_compensation_array[pump_pressure_index]
#	    volFlow_min = volFlow_min # (flow_error_compensation*1000)
#	except:
#	    print('compensation error')
#	    volFlow_min = volFlow_min #- (27*1000)
	if((volFlow_min/1000) > 0.1):
	    if(diff >= 0 and inhale_loop == "1" and volume_flag == 0):
                volume=(volFlow*(clock*0.56))+volume
	    elif(diff < 0 and inhale_loop == "0"):
		volume = (volFlow*(clock*0.68)) + volume
	clock_t2= time()
#	print('volume is')
#	print(volume)
        return volume

    def rate(self):
	global peep_val,peep_hole,P_plat,indiff,loop,init_flow_int
	diff=self.SDP_pressure()
	compensation = 0
	if( diff == -9999):
	    print('rate cannot be calculated SDP failed')
	    return -9999
#	print('SDP reading is')
#	print(diff)
	massFlow=1000*(sqrt((abs(diff)*2*rho)/volume_divisor))
	volFlow= massFlow/rho
	volFlow_min=(volFlow*60)*0.38 #- 6.0 #+ #(init_flow_int*-1)
	if(volFlow_min > 80):
	    volFlow_min=(volFlow*60)*1.1
# 	if(P_plat < 10):
# 	    volFlow_min  = volFlow_min -  2.5
# 	elif(P_plat >= 10 and P_plat <=15):
# 	    volFlow_min = volFlow_min - 3.5
# 	elif(P_plat > 15 and P_plat <=20):
# 	    volFlow_min = volFlow_min - 5.3
 #	print("VOLUME FLOW IN HERE IS")
 #	print(volFlow_min)
#	print(init_flow_int)
	try:
	    flow_P_plat_value=find_nearest(P_plat_array,indiff)
	    flow_index= P_plat_array.index(flow_P_plat_value)
	    pump_pressure_index = pump_pressure_array[flow_index]
	    flow_error_compensation = flow_error_compensation_array[pump_pressure_index]
	    if(loop == 1):
	        volFlow_min = volFlow_min #- (flow_error_compensation)
	    elif(loop == 0):
		volFlow_min = volFlow_min #+ (flow_error_compensation)
	except:
	    volFlow_min = volFlow_min #- (27)
 	    print('compenation error')
	volFlow_min = abs(volFlow_min)
	if(diff < 0):
	    volFlow_min = -2*volFlow_min
#	if(diff >= 0.0 and diff <= 0.02):
#	    volFlow_min = 0.0
	return volFlow_min

    def Vol_Flow(self,volume):
        diff=self.SDP_pressure()
        massFlow=1000*((abs(diff)*volume_divisor)**(0.5))
        volFlow = massFlow/rho
        print('volflow')
        print(volFlow)
	clock_t1=time()
	clock= (clock_t1 - clock_t2)*1.1
	print('clock is')
	print(clock,5)
        volume=volFlow*clock+volume
	clock_t2= time()
        volume_ml=volume*1000
        volFlow_min=volFlow*60
        return volume_ml
    #------------------------------------------------
    def measure_temp(self):
        temp = os.popen("vcgencmd measure_temp").readline()
	temp = temp.replace("temp=","")
        return (temp.replace("'C\n",""))
    #-------------------------------------------------
    # Pressure Control Intermitent Mode of Ventilation
    def PC_IMV(self):
        global peep_first,volume_flag,lock,peep_factor,IH_time, EH_time, IHold, EHold,exp_flow_array, exp_press_array , peep,peep_open,rise_flag,rise_time,inhale_array,motor_factor,time_elapsed_inhale_flow,time_elapsed_exhale_flow,time_elapsed_inhale,plataeu_pressure,patient_trigger_flow,pressure_low_count,leak_comp_flag,running_avg ,comp_flag,trigflow_comp,back_backup,inhale_loop,toggle_switch,loop,peep,patient_set,exhale_time_last,TOT_last,ABP_flag,TITOT,pressure_count,peep_count, SDP_flag,flow_plat,indiff,patient_status,pump_pressure,pump_pressure_low,pump_pressure_high,ratio_set,time_error_set,thread_mode_status,time_error, time_error_last,ABP_flag,P_plat_value,prev_mode,data,ratio,RR_time,indiff,peep_val, peep_hole, RR, BPM, MVi_array, MVi,volume, volume_peak_inhale, trigger, max_value, volume_peak_exhale, MVe_array, MVe,clock_t2
        thread_mode_status = True
	inhale_array = []
	lock = 0
	volume_flag = 0
	peep_factor = 2.0
	motor_factor = 0.5
	rise_time = 0
	rise_flag = '0'
	time_elapsed_inhale_flow = 0
	time_elapsed_exhale_flow =0
	pressure_low_count = 0
	plataeu_pressure = 0
	time_elapsed_inhale = 0
	del MVi_array[:]
	del MVe_array[:]
	patient_trigger_flow = 0
	patient_set = 0
	comp_flag = '0'
	trigflow_comp = 0
	exhale_time_last = time()
	TOT_last = time()
	peep_count = 0
	loop = 1
	backup_counter = 0
	inhale_loop = "0"
	pressure_count = 0
	VTi_volume = 0
	leak_percentage = 0
	ratio_set = 0
	running_avg = 0.0
	flag = 0
	GPIO.output(35,GPIO.LOW)
	peep_set = 0
	time_error_set = 0
	sol.ChangeDutyCycle(100)
        start_time=time() - 5000
	time_error_last = time()
	sending_time = 0.2
	peep_val_send = 0
	temp_peep = 0
        try:
            while True:
		prev_mode = 12
		ABP_flag = 0
#		print('we are in PC_IMV mode')
		hold_check()
		data= self.read_data()
#		print('mode is')
#		print(data)
		if(data == 1):
		    break;
#		print(inhale_time)
#		print(BPM)
#		print(PIP)
#		print(VTi_max)
#		print('peep in main loop is')
#		print(peep)
#		print(peep_val)
#		print(P_plat)
#		print(P_plat_value)
#		print('here the pump pressure is')
#		print(pump_pressure)
#		GPIO.output(22, GPIO.LOW)
		volFlow_rate = self.rate()
		indiff= self.ABP_pressure()
		if(indiff == -8888):
			ABP_flag = 1
			indiff = 0
		packet_exhalation = ('C@' + str(round(indiff,2)) + ',' + str(round(volFlow_rate,2)) + ',' + str(int(volume)) + ',' + str(int((TITOT)*100))+'#')
		if(sending_time > 0.02):
		    try:
			ser.write(packet_exhalation)
		    except:
			print('BT Error sending in Exhalation conditional')
		    send_last_time = time()
		sending_time =time() - send_last_time 
		indiff= self.ABP_pressure()
		exp_press_array.append(indiff)
                flow=self.rate()
	        if(indiff <= peep_val+1 and  comp_flag == "1" and leak_comp_flag == "1"):
		    trigflow_comp = trigflow + running_avg + 3
		    comp_flag = '0'
#		    print("HERE COMES THE PARAMETRS")
#		    print(trigflow)
#		    print(running_avg)
#		    print(trigflow_comp)
		    running_avg = 0
		if(leak_comp_flag == '0'):
		    trigflow_comp = trigflow
#                if(indiff <= peep_val+2 and flow >= -10):
 #                   sol.ChangeDutyCycle(100)
#		trig_comp = trigflow+peep_val_send
#		print('before flow')
#		print(comp_flag)
	#	flow = flow - peep_hole[int(peep_val)]
#		print(leak_comp_flag)
#		print("indiff is")
		print(trigflow)
		print(flow)
                current_time=time()
	        if(comp_flag == '0' and flow > 1):
		    running_avg = (running_avg + flow)/2
#                print(peep_val+1)
		TOT_last = current_time - exhale_time_last
#		peep_val_send = indiff
		SDP_flag = 0
		time_error = time() - time_error_last
		loop = 1
                if(flow >= trigflow or (current_time-start_time)>cycle_time):
                    motor_1.ChangeDutyCycle(100)
		    if(P_plat <= 22):
			motor_1.ChangeDutyCycle(80)
		    sol.ChangeDutyCycle(100)
		    RR_time=current_time-start_time
		    patient_trigger_flow = flow
		    GPIO.output(33, GPIO.LOW)
		    GPIO.output(35,GPIO.LOW)
		    volume = 0
		    leak_comp_flag = leak_comp()
		    pump_pressure_now = pump_pressure
# 		    sol.ChangeDutyCycle(100)
		    inhale_loop = "1"
		    indiff = self.ABP_pressure()
		    flow_flag = 0
		    comp_flag = '1'
#		    del exp_press_array[:]
		    peep_val_send = temp_peep
		    if(len(exp_press_array) > 10):
			peep_val_send = exp_press_array[len(exp_press_array) -5]
		    del exp_press_array[:]
		    print("ENTERED INHALE LOOP")
		    print(indiff)
#                     motor_1.ChangeDutyCycle(100)
# 		    if(P_plat <= 22):
# 			motor_1.ChangeDutyCycle(80)
		    print(peep_val_send - trigflow)
#			motor_1.ChangeDutyCycle(65)
#		        flow_flag = 0
#		    else:
#			motor_1.ChangeDutyCycle(pump_pressure)
#			flow_flag = 1
#			pump_pressure_now = pump_pressure /2
		    if(time_error < 0.7):
			time_error_set = 1
#			try:
#			    ser.write('ACK24')
#			except:
#			    print('unable to send data')
#		    if(time_error > 0.7):
#			time_error_set = 0
#			try:
#			    ser.write('ACK24')
#			except:
#			    print('unable to send data')
		    if(float(ratio) <= 0.25):
			ratio_set = 1
#			try:
#			    ser.write('ACK12')
#			except:
#			    print('unable to send data')
		    if(float(ratio) >= 0.75):
			ratio_set = 1
#			try:
#			    ser.write('ACK13')
#			except:
#			    print('unable to send data')
		    if(float(ratio) >0.25 and float(ratio) < 0.75):
			ratio_set = 0
#			try:
#			    ser.write('ACK22')
#			except:
#			    print('unable to send data')
		    if(RR_time > cycle_time):
			trigger='0'
		    else:
			trigger='1'
                 #   indiff= self.ABP_pressure()
                  #  print(indiff)
                    time_elapsed_inhale=0
                    indiff = 0
		    Pmean=0
		    del Pmean_array[:]
		    clock_t2= time()
		    del inhale_array[:]
#		    print(clock_t2)
		    volume=0
		    volume_peak_inhale=0
		    sending_time = 0.2
		    send_last_time=0
		    volFlow_rate = self.rate()
		    VTi_volume=0
		    pump_flag = 0
		   # flow_flag = 0
		    current_time = time()
		    peak_insp_pressure = 0
		    pump_pressure_now = pump_pressure /2
#		    flow_zero_intialize = time() # clock starts to detect zero flow
#		    flow_zero_flag = 0
#		    flow_zero_elapsed = 0
		    peak_flow = volFlow_rate
		    rise_flag = '0'
                    while(indiff <= PIP and time_elapsed_inhale <= inhale_time):
                    #    motor_1.ChangeDutyCycle(pump_pressure)
                        #motor_2.ChangeDutyCycle(pump_pressure)
                        t3 = time()
			GPIO.output(37, GPIO.HIGH)
#			if(volFlow_rate <= 2):
#			    flow_zero_flag = 1
#			    flow_zero_initialize = time()
		        if(volFlow_rate > volume_peak_inhale * 0.4):
			    time_elapsed_inhale_flow = time_elapsed_inhale
#			    print("INSIDE")
#			    flow_zero_elapsed = time() - flow_zero_initialize 
			if(ABP_flag == 0):
                            indiff= self.ABP_pressure()
			if(indiff >= P_plat * 0.8 and rise_flag == '0'):
				rise_time = time_elapsed_inhale
				rise_flag = '1'
			if(volFlow_rate < volume_peak_inhale * 0.3):
			    plataeu_pressure = indiff
			#    time_elapsed_inhale_flow = time_elapsed_inhale
                        time_elapsed_inhale= t3-current_time
 #                       print(volFlow_rate)
			Pmean_array.append(indiff)
			if(SDP_flag == 0):
			    volume=self.Flow()
		#	    volume = volume - (volume*leak_percentage)/100)
			    volFlow_rate=self.rate()
    			    if(volFlow_rate > peak_flow):
			        peak_flow = volFlow_rate
			if(volFlow_rate <  peak_flow * 0.70 and time_elapsed_inhale >= 0.5 and trigger == '1'):
			    break 
			if((indiff >= P_plat*motor_factor or time_elapsed_inhale >= 0.5) and flow_flag == 0): #(time_elapsed_inhale > 0.3 and flow_flag == 0) or indiff >= 10): #or indiff >= P_plat - 4): #or volFlow_rate >= flow_plat - 10):
			    flow_flag = 1
			    motor_1.ChangeDutyCycle(pump_pressure)
#			    print('CMHO reacged')
#			    print(indiff)
# 			if(P_plat <= 17 and (indiff >= P_plat/2 or time_elapsed_inhale >= 0.1) and flow_flag == 0): #(time_elapsed_inhale > 0.3 and flow_flag == 0) or indiff >= 10): #or indiff >= P_plat - 4): #or volFlow_rate >= flow_plat - 10):
# 			    flow_flag = 1
# 			    motor_1.ChangeDutyCycle(pump_pressure)
# 			if(flow_flag == 1 and volFlow_rate <= flow_plat):
# 			    motor_1.ChangeDutyCycle(pump_pressure)
			if(volFlow_rate >= flow_plat and flow_flag ==1):
			    flow_flag = 2
			if(volume == -9999):
			    SDP_flag = 1;
			if(sending_time > 0.02):
			    packet_inhalation =('A@' + str(round(indiff,2)) + ','+str(round(volFlow_rate,2)) + ',' + str(int(volume)) + ',' + str(trigger) + '#')
			    try:
				ser.write(packet_inhalation)
			    except:
				print('BT error send Inhalation')
#			    print('packet size isssssssssss')
#			    print(len(packet_inhalation))
#			    print('packet conatins ')
#			    print(packet_inhalation)
			    send_last_time= time()
#			if(volFlow_rate < flow_plat -3 and pump_pressure_now < pump_pressure):
#			    pump_pressure_now = pump_pressure_now + 4
#			    print('in 1st loop')
#			    motor_1.ChangeDutyCycle(pump_pressure_now)
#			elif(volFlow_rate > flow_plat + 3 and pump_pressure_now > pump_pressure):
#			    pump_pressure_now = pump_pressure_now -4
#			    print('in 2nd loop')
#			    motor_1.ChangeDutyCycle(pump_pressure_now)
			if(flow_flag == 3):
			    if(volFlow_rate <= flow_plat -5 and pump_pressure_now <= P_plat):
		#	   # if(pump_flag == 0):
			      #      pump_pressure_now  = pump_pressure_now + 5
			     #   else:
				pump_pressure_now = pump_pressure_now + 1
	#		    print('the pump_pressure now is')
	#		    print(pump_pressure_now)
	#		    print('pump pressure is')
	#		    print(pump_pressure)
			        motor_1.ChangeDutyCycle(pump_pressure_now)
			    if(volFlow_rate >= flow_plat and pump_pressure_now >= P_plat/2): #and pump_pressure_now >( pump_pressure /2)):
			        pump_pressure_now = pump_pressure_now - 1
			        motor_1.ChangeDutyCycle(pump_pressure_now)
			      #  pump_flag = 1
			sending_time=t3 - send_last_time
#			print('flow is')
#			print(volFlow_rate)
#			print(time_elapsed_inhale)
			inhale_array.append(indiff)
			VTi_volume = max(volume,VTi_volume)
			peak_insp_pressure = max(peak_insp_pressure, indiff)
			volume_peak_inhale=max(volume_peak_inhale,volFlow_rate)
		    if(IHold == '1'):
			motor_1.ChangeDutyCycle(int(pump_pressure*0.9))
			GPIO.output(33, GPIO.HIGH)
			print("entering inspiratory hold")
			print(IH_time)
			ser.write("ACK66")
			instant_time = time()
		        sending_time = 0.2
		        send_last_time=0
			while(time() - instant_time < IH_time):
			    print(sending_time)
			    indiff= self.ABP_pressure()
			    volFlow_rate=self.rate()
			    if(sending_time > 0.02):
			        packet_inhalation =('A@' + str(round(indiff,2)) + ','+str(round(volFlow_rate,2)) + ',' + str(int(volume)) + ',' + str(trigger) + '#')
			        try:
				    ser.write(packet_inhalation)
			        except:
				    print('BT error send Inhalation')
			        send_last_time= time()
			    sending_time=time() - send_last_time
		    IHold = '0'
		    GPIO.output(33, GPIO.LOW)
		    if(indiff >= (P_plat+3)):
			pressure_count = pressure_count +1
			if(pressure_count >= 3):
		            try:
		  	        ser.write('ACK07')
			    except:
		      	        print('unable to send')
		    else:
			pressure_count = 0
			try:
			    ser.write('ACK57')
			except:
			    print('unable to send')
		    if(indiff <= (P_plat)*0.5):
			pressure_low_count = pressure_low_count +1
			if(pressure_low_count >= 3):
		            try:
		  	        ser.write('ACK62')
			    except:
		      	        print('unable to send')
		    else:
			pressure_low_count = 0
			try:
			    ser.write('ACK72')
			except:
			    print('unable to send')
                    flag=1
		    loop = 0
		    inhale_array_length = len(inhale_array)
		    if(inhale_array_length % 2 != 0):
			inhale_array.append(0)
			inhale_array_length = inhale_array_length + 1
		    inhale_first_half = inhale_array[0:inhale_array_length/2]
		    inhale_second_half = inhale_array[inhale_array_length/2:]
		    first_max = max(inhale_first_half)
		    second_max = max(inhale_second_half)
		    if(first_max - P_plat > 0.5 and motor_factor > 0.2):
		        motor_factor = motor_factor - 0.01
			if(first_max - P_plat > 2):
			    motor_factor = motor_factor - 0.01
		    if(P_plat - first_max > 0.5 and motor_factor <= 1.0):
			motor_factor = motor_factor + 0.05
			if(P_plat - first_max > 2):
			    motor_factor = motor_factor + 0.05
		    toggle_switch = str(toggle())
		    if(EHold == '1'):
			toggle_switch ='1'
#		    print("MOTOR FACTOR ISSSSSSSSSSSSSSSSSSSSSSS")
#		    print(motor_factor)
#		    print(first_max)
#		    print(second_max)
		    if(toggle_switch == "0"):
#			print("got in the first statement")
		        motor_1.ChangeDutyCycle(peep_open)
			sol.ChangeDutyCycle(0)
		    if(toggle_switch == "1"):
	#		GPIO.output(33, GPIO.HIGH)
		        motor_1.ChangeDutyCycle(peep)
			sol.ChangeDutyCycle(0)
		    try:
		        Pmean= sum(Pmean_array)/len(Pmean_array)
		    except:
			print("zero exception error")
#		    print('P_plat is')
#		    print(int(peak_insp_pressure))
#		    print(P_plat)
                    RR.append(RR_time)
		    peak_insp_pressure = int(peak_insp_pressure)
		    if(indiff > P_plat and patient_status == 1 and ABP_flag == 0 and indiff < PIP and pump_pressure >= 6 and pump_pressure <= 97):
			pump_pressure = pump_pressure - 1
#			print('-1')
		    elif(indiff < P_plat and patient_status == 1 and ABP_flag == 0 and  indiff < PIP and  pump_pressure >= 5 and pump_pressure <= 95):
			pump_pressure = pump_pressure + 1
		    if(patient_status == 1 and peep_val_send < peep_val and peep <= peep_first + 10 and peep >= 4 and  peep < 90 and toggle_switch == '1'):
			peep = peep + 1
		    elif(patient_status == 1 and peep_val_send > peep_val and peep >= peep_first - 10 and peep > 5 and peep < 92 and toggle_switch == '1'):
			peep = peep - 1
#			print('+1')
#		    print('indiff')
#		    print(indiff)
#		    print('peak_insp_pressure')
#		    print(peak_insp_pressure)
		    MVi_array.append(VTi_volume)
#		    print('P_plat')
#		    print(P_plat)
#		    print('Pmean is')
#		    print(Pmean)
		    GPIO.output(37, GPIO.LOW)
		if(flag == 1):
		    patient_set = 0
		    if((leak_percentage > 90 or indiff <= 5)  and patient_set == 0 ):
			patient_set = 1
			buzzer(5,1)
			patient_status = 0
		        print('................disconnection................')
#			trigger = '0'
			try:
			    ser.write('ACK05')
			except:
			    print('not able to send')
		    if((leak_percentage <= 90 or indiff > 5) and patient_set == 0):
			buzzer(5,0)
			patient_set = 1
			patient_status = 1
			try:
			    ser.write('ACK06')
			except:
			    print('not able to send')
                    if(len(RR) >= 5):
                        BPM = ( RR[1] + RR[2] + RR[3] + RR[4]) /4
                        BPM = ceil(60/BPM)
			if(BPM > 70):
			    try:
				ser.write('ACK09')
			    except:
				print('unable to send')
			else:
			    try:
				ser.write('ACK59')
			    except:
				print('unable to send')
 #                      print('BPM is')
  #                     print(BPM)
                        RR.pop(0)
		    if(len(MVi_array) >= 5):
		        MVi= (MVi_array[1] + MVi_array[2] + MVi_array[3] + MVi_array[4])/4
			MVi = MVi * BPM
#		        print('MVi value is')
#		        print(MVi)
		        MVi_array.pop(0)
		        packet_end_inhalation = ('B@' + str(round(peak_insp_pressure,2)) + "," + str(int(VTi_volume - (VTi_volume*leak_percentage)/200)) + "," + str(round(volume_peak_inhale,2)) + "," + str(round(Pmean,2)) + "," + str(round((MVi/1000),2)) + "," + str(round(patient_trigger_flow,2)) + "," + str(round(time_elapsed_inhale,2)) + "," + str(round(indiff,2)) + "," + str('0') + "," + str(round(rise_time,2)) + '#')
#		        print('packet length is')
#		        print(packet_end_inhalation)
		        try:
		            ser.write(packet_end_inhalation)
		        except:
			    print('BT Errror in end of inhalation loop')
                if(flag==1):
		    time_error_last = time()
		    time_elap = 0
		    inhale_loop = "0"
		    buzzer(7,0)
#		    del exp_flow_array[:]
#		    del exp_press_array[:]
		    q= time()
		    volume = 0
		    sending_time = 200
		    send_last_time = 0
		    volume_peak_exhale = 0
		    exhale_break = '0'
		    sol_flag = 0
		    while(time_elap < 0.1 or (toggle_switch == "1" and time_elap < 0.3)):
#			exp_flow = self.rate()
			print(time_elap)
			w=time()
			time_elap=w-q
                        indiff = self.ABP_pressure()
			#
			exp_press_array.append(indiff)
			peak_insp_pressure = int(peak_insp_pressure)
			if(peak_insp_pressure <= 0):
			    peak_insp_pressure = 1
			ratio = 1.0
                        if(ratio > 0.8 and time_elap >= 0.2  and toggle_switch == '1' and exhale_break == '0'):
                            GPIO.output(35, GPIO.LOW)
                            sleep(0.05)
    #                        motor_1.ChangeDutyCycle(peep)
			    exhale_break = '1'
                        elif(ratio >= 0.15 and ratio <= 0.4 and time_elap >= 0.4 and P_plat < 28  and toggle_switch == '1' and exhale_break == '0'):
                            GPIO.output(35, GPIO.LOW)
			    print("Brakingggggggggggggggggggggg")
                            sleep(0.05)
     #                       motor_1.ChangeDutyCycle(peep)
			    exhale_break = '1'
                        elif(ratio >= 0.15 and ratio <= 0.4 and time_elap >= 0.4 and P_plat >=28 and toggle_switch == '1' and exhale_break == '0'):
                            GPIO.output(35, GPIO.LOW)
                            sleep(0.05)
      #                      motor_1.ChangeDutyCycle(peep)
			    exhale_break = '1'
                        elif(ratio > 0.4 and ratio <= 0.8 and time_elap >= 0.3 and toggle_switch == '1' and exhale_break == '0'):
                            GPIO.output(35, GPIO.LOW)
                            sleep(0.05)
       #                     motor_1.ChangeDutyCycle(peep)
			    exhale_break = '1'
                        elif(ratio <0.15 and ratio >=0.00 and time_elap >= 0.4 and toggle_switch == '1' and exhale_break == '0'):
                            GPIO.output(35, GPIO.LOW)
                            sleep(0.05)
        #                    motor_1.ChangeDutyCycle(peep)
			    exhale_break = '1'
			if(ABP_flag == 0):
			    indiff = self.ABP_pressure()
# 			if((indiff < peep_val*2.0 or time_elap > 0.4) and exhale_break == '0'):
 #			    GPIO.output(35, GPIO.LOW)
#			    sleep(0.05)
#			    motor_1.ChangeDutyCycle(peep)
#			    exhale_break = '1'
#			    print("BRAKES APPLIEDDDDDDDDDDDD")
			diff = self.SDP_pressure()
			if(SDP_flag == 0):
			    volFlow_rate = self.rate()
			    volume=self.Flow()
#			print(peep_val)
#			print(indiff)
			if(diff == 700 or diff == -700 or SDP_flag == 1):
			    try:
				f = open("/home/pi/AgVa_5.0/mode.txt","w")
				f.write(str('11'))
				f.close()
				try:
				    ser.write('ACK30')
				except:
				    print('unable to send data')
			    except:
				print('unable to write mode')
			packet_exhalation= ('C@' + str(round(indiff,2)) + ',' + str(round(volFlow_rate,2)) +',' + str(int(volume)) + ',' + str(int((TITOT)*100)) + '#')
			if(sending_time > 0.02):
			    try:
				ser.write(packet_exhalation)
			    except:
				print('BT exhalation error')
			    send_last_time = time()
			sending_time = w- send_last_time
# 		        if(toggle_switch == "1" and indiff <= peep_val+2 and volFlow_rate >= -5 and sol_flag == 0):
#                             sol.ChangeDutyCycle(100)
# 			    sol_flag = 1
# 			    holding_now = time()
# 			    holding_time = 0.3
# 			    peep_check = peep_val - trigflow - 1 #- 2.0
# 			    if(peep_check < 0):
# 			        peep_check = 0
# 			    while( time() - holding_now <= holding_time and time_elap <0.3):
# 				time_elap = time() - q
# 				indiff = self.ABP_pressure()
# 				GPIO.output(33, GPIO.LOW)
# 				if(indiff <= peep_check):
# 				    break;
# 			        volFlow_rate = self.rate()
# 			        volume=self.Flow()
# 				packet_exhalation= ('C@' + str(round(indiff,2)) + ',' + str(round(0,2)) +',' + str(int(volume)) + ',' + str(int((TITOT)*100)) + '#')
# 				if( 1 == 1):
# 			    	    try:
# 					ser.write(packet_exhalation)
# 			    	    except:
# 					print('BT exhalation error')
# 			    	    send_last_time = time()
# 				sending_time = w- send_last_time
# 				sleep(0.02)
#			print('packet exhalation size  is')
#			print(volFlow_rate)
			volume_peak_exhale= min(volume_peak_exhale, volFlow_rate)
#			print('packet_exhal;ation conatins is')
		    temp_peep = self.ABP_pressure()
                    time_elapsed_exhale_flow = 0
                    new_time=time()
                    time_elapsed_exhale=0.0
                    diff=self.SDP_pressure()
		    indiff = self.ABP_pressure()
	#	    motor_1.ChangeDutyCycle(pump_pressure)
		    start_time=current_time
# 		    if(toggle_switch == "1" and indiff <= peep_val-peep_factor and sol_flag == 0):
#                         sol.ChangeDutyCycle(100)
# 			holding_now = time()
# 			holding_time = 0.3
# 			sol_flag = 1
# 			sending_time = 2.0
# 			send_last_time = 0
# 			while( time() - holding_now <= holding_time):
# 			    indiff = self.ABP_pressure()
# 			    volFlow_rate = self.rate()
# 			    volume=self.Flow()
# 			    packet_exhalation= ('C@' + str(round(indiff,2)) + ',' + str(round(0,2)) +',' + str(int(volume)) + ',' + str(int((TITOT)*100)) + '#')
# 			    if(1 == 1):
# 			        try:
# 				    ser.write(packet_exhalation)
# 			    	except:
# 				    print('BT exhalation error')
# 			    	send_last_time = time()
# 			    sending_time = time()- send_last_time
# 			    sleep(0.02)
#		    print("Time elapsed exhale  " + str(time_elapsed_exhale))
#		    print("ABP                  " + str(indiff))
#		    print("SDP                  " + str(diff))
		    sending_time=200
		    send_last_time = 0
#		    sol_flag = 0
                    while((time_elapsed_exhale<((cycle_time-inhale_time)-0.3) and (diff <= 0)) or (SDP_flag ==1 and time_elapsed_exhale<(cycle_time-inhale_time-0.5))):
			exp_flow = self.rate()
			print("Time elapsed exhale  " + str(diff))
#			print("ABP                  " + str(indiff))
#			print("SDP                  " + str(diff))
 #                       exp_flow_array.append(exp_flow)
                        time_elapsed_exhale=time()-new_time
			if(ABP_flag == 0):
			    indiff = self.ABP_pressure()
			exp_press_array.append(indiff)
			if(SDP_flag == 0):
           		    volume=self.Flow()
			    volFlow_rate=self.rate()
			    diff = self.SDP_pressure()
			if((toggle_switch == "1" and sol_flag == 0 and ((indiff <= peep_val+2 and volFlow_rate >= -10) or (indiff <= peep_val-2))) or (time_elapsed_exhale>((cycle_time-inhale_time)-0.3)) ):
			    peep_check = peep_val - trigflow - 1
			    if(peep_check < 0):
			        peep_check = 0
# 			    if(volFlow_rate >= trigflow):
# 				break;
			    sol.ChangeDutyCycle(100)
			    holding_now = time()
			    holding_time = 0.3
# 			    peep_check = trigflow - peep_val - 2.0
# 			    if(peep_check < 0):
# 			        peep_check = 0
			    while( time() - holding_now <= holding_time and (time_elapsed_exhale<((cycle_time-inhale_time)-0.3))):
				time_elapsed_exhale = time() - new_time
				indiff = self.ABP_pressure()
				GPIO.output(33, GPIO.LOW)
				if(volFlow_rate > trigflow+peep_val):
				    break;
			        volFlow_rate = self.rate()
			        volume=self.Flow()
				packet_exhalation= ('C@' + str(round(indiff,2)) + ',' + str(round(0,2)) +',' + str(int(volume)) + ',' + str(int((TITOT)*100)) + '#')
				if(1 == 1):
			    	    try:
					ser.write(packet_exhalation)
			    	    except:
					print('BT exhalation error')
			    	    send_last_time = time()
			        sending_time = time()- send_last_time
				sleep(0.02)
			    sol_flag = 1
 			if(volFlow_rate < -5):
 			    time_elapsed_exhale_flow = time_elapsed_exhale
 	#		print()
			volume_peak_exhale=min(volume_peak_exhale,volFlow_rate)
			packet_exhalation = ('C@' + str(round(indiff,2)) + ',' + str(round(volFlow_rate,2)) + ',' + str(int(volume)) + ',' + str(int((TITOT)*100))+'#')
			if(sending_time > 0.02):
			    try:
				ser.write(packet_exhalation)
			    except:
				print('BT Error sending in Exhalation conditional')
			    send_last_time = time()
			sending_time =time() - send_last_time 
                        start_time=current_time
#                    print("CE time is ....." + str(time() - new_time ))
                    if(int(peep_val_send) >=  peep_val - 1 and int(peep_val_send) <= peep_val+1):
                        lock = 1
                    elif(int(peep_val_send) > peep_val+1 or int(peep_val_send) < peep_val -1):
                        lock = 0
		    if(EHold == '1'):
			GPIO.output(33, GPIO.HIGH)
			print("Entering expiratory hold")
			instant_time = time()
		        sending_time = 0.2
		        send_last_time=0
			ser.write("ACK67")
			while(time() - instant_time < EH_time):
			    indiff= self.ABP_pressure()
			    print(time() - instant_time)
			    volFlow_rate=self.rate()
			    if(sending_time > 0.02):
			        packet_exhalation = ('C@' + str(round(indiff,2)) + ',' + str(round(volFlow_rate,2)) + ',' + str(int(volume)) + ',' + str(int((TITOT)*100))+'#')
			        try:
				    ser.write(packet_exhalation)
			        except:
				    print('BT error send Inhalation')
			        send_last_time= time()
			    sending_time=time() - send_last_time
			peep_val_send = indiff
			GPIO.output(33, GPIO.HIGH)
		    EHold = '0'
		    if(time_elapsed_exhale<((cycle_time-inhale_time)-0.4)):
		        print("TE condition :    " + str(time_elapsed_exhale))
		    if(diff <= 0 or indiff >(((peak_insp_pressure - peep_val)/3 ) + peep_val)):
			print("SDP condition :" +  str(diff) + "::" + str(indiff))
#		    if(indiff >(((peak_insp_pressure - peep_val)/3 ) + peep_val)):
#			print("ABP condition : " + str(indiff))
#		    flow_for_peep = 0.25 * (min(exp_flow_array))
		   # print(flow_for_peep)
#		    value1 = find_nearest(exp_flow_array , flow_for_peep)
#		    peep_array_index = exp_flow_array.index(value1)
#		    peep_val_send = exp_press_array[peep_array_index] 			#print(exp_press_array[index1])		#print(min(exp_flow_array))
#		    print(exp_press_array)
		    exhale_time_last = time()
                    flag=0 
		    inhale_loop = "1"
		    loop = 1
		    if(toggle_switch == '1'):
		        time_elapsed_exhale = time_elapsed_exhale + 0.4
			time_elapsed_exhale_flow = time_elapsed_exhale_flow + 0.4
		    else:
			time_elapsed_exhale = time_elapsed_exhale + 0.1
			time_elapsed_exhale_flow = time_elapsed_exhale_flow + 0.1
		    print("Peeeeeeeeeeeeeeeeeeeeeeeeeeeeeep factor is")
		    print(peep_factor)
		    TITOT=(time_elapsed_inhale/(time_elapsed_inhale + time_elapsed_exhale + TOT_last))
#		    if(len(exp_press_array) > 10):
#		        peep_val_send =  exp_press_array[len(exp_press_array) - 3]
		    if((peep_val_send) <= peep_val and patient_status == 1 and toggle_switch == '1' and lock == 0):
			peep_factor = peep_factor - (peep_val - peep_val_send)*0.1
		    if((peep_val_send) > peep_val and patient_status == 1 and toggle_switch == '1' and lock == 0):
			peep_factor = peep_factor + (peep_val_send - peep_val)*0.1
		    if(int(peep_val_send) > peep_val+2):
			peep_count = peep_count + 1
			if(leak_percentage > 20 and trigger == '0'):
			    ser.write("ACK39")
			if(peep_count >=3 and peep_set == 0):
			    buzzer(4,1)
			    peep_set = 1
			    try:
			        ser.write('ACK08')
			    except:
			        print('unable to send')
		    else:
			if(peep_set == 1):
			    buzzer(4,0)
			    peep_set = 0
			peep_count = 0
			ser.write("ACK49")
			try:
			    ser.write('ACK58')
			except:
			    print('unable to send')
		    FiO2 = self.FiO2()
                    self.power()
#		    print('FiO2 is')
#		    print(FiO2)
		    MVe_array.append(volume)
		    if(len(MVe_array)>=5):
		        MVe= (MVe_array[1] + MVe_array[2] + MVe_array[3] + MVe_array[4])/4
			MVe = MVe * BPM
		        MVe_array.pop(0)
		        try:
			    leak_percentage = ((MVi - MVe)/MVi)*100
			    if(leak_percentage <= -1):
			        leak_percentage = 0
		        except:
			    leak_percentage = 0
#		        print('MVe is')
#		        print(MVe)
#		   	print(" bhai apna leak flow hai ye :::::::::::::::::::::::::::::::::::::::::::::: " + str(trigflow_comp - trigflow))
		        packet_end_exhalation = ('D@' + str(round(peep_val_send,2)) + ',' + str(int(BPM)) + ',' + str(FiO2) + ',' + str(round(volume_peak_exhale,2)*-1) + "," + str(round((MVe/1000),2)) + "," + str(leak_percentage) + "," + str(trigflow_comp - trigflow) + "," + str(volume) + "," + str(time_elapsed_exhale_flow)  + '#')
#			print('string issssssssssssssssssssssssssssssssssssss')
#			print(packet_end_exhalation)
		        try:
	                    ser.write(packet_end_exhalation)
		        except:
			    print('BT send error in end of exhalation')
#		    try:
#		        with open('/home/pi/AgVa_5.0/data.csv', mode='a') as data:
 #  		            data = csv.writer(data, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
#		            now = datetime.datetime.now()
#		            data.writerow([str(now),str(self.measure_temp()),str(Pmean),str(volume),str(volume_peak_inhale),str(trigger),str(BPM),str(MVi),str(ratio), str(volume_peak_exhale),str(MVe)])
#		    except:
#			print('data file open error')
        except KeyboardInterrupt:
            return
    #--------------------------------------------------

    #--------------------------------------------------
    # Volume Control Intermitent Mode of Ventilation
    def VC_IMV(self):
        global volume_flag,exp_flow_array,exp_press_array,peep,peep_open,time_elapsed_exhale_flow,patient_trigger_flow,rise_time, rise_flag,inhale_array,motor_factor,pressure_low_count,leak_comp_flag,running_avg,trigflow_comp, comp_flag,volume_comp,change_setting,back_backup,inhale_loop,pump_pressure_array,P_plat_array,compliance,compliance_array,toggle_switch,peep_open,loop,peep,peep_pwm,P_plat,P_plat_high,exhale_time_last, TOT_last,TITOT,P_plat_low,ABP_flag,pressure_count,peep_count,volume_count, SDP_flag,patient_set,flow_plat,ratio_set,patient_status,pump_pressure,pump_pressure_low, pump_pressure_high,time_error_set,thread_mode_status,time_error, time_error_last,prev_mode,P_plat_value,peep_hole, peep_val,data,ratio,RR_time,indiff, RR, BPM, MVi_array, MVi,volume, volume_peak_inhale, trigger, max_value, volume_peak_exhale, MVe_array, MVe,clock_t2
        thread_mode_status = True
	rise_time = 0
	volume_flag = 0
	time_elapsed_exhale_flow = 0
	patient_trigger_flow = 0
	rise_flag = '0'
	pressure_low_count = 0
	volume_count = 0
	running_avg = 0.0
	inhale_array = []
	del MVi_array[:]
	del MVe_array[:]
	del exp_press_array[:]
	del exp_flow_array[:]
	motor_factor = 0.5
	trigflow_comp = 0
	comp_flag = '0'
	exhale_time_last = time()
	TOT_last = time()
	volume_comp = 0
	peep_count = 0
	ratio_set = 0
	flag = 0
	inhale_loop = "0"
	backup_counter = 0
	VTi_volume = 0
	leak_percentage = 0
	loop = 1
	pressure_count = 0
	del compliance_array[:]
	volume_set = 0
	compliance_flag = 0
	patient_set = 0
	peep_set = 0
	sol.ChangeDutyCycle(100)
	GPIO.output(35, GPIO.LOW)
	temp_peep = 0
	peep_val_send = 0
	time_error_set = 0
        start_time=time() - 5000
	time_error_last = time()
	sending_time = 0.2
        try:
            while True:
		prev_mode = 22
#		print('we are in VC_IMV mode')
		data= self.read_data()
#		print(data)
		if(data == 1):
		    print("breaking the cuircuit in hereeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee")
		    break;
#		print(inhale_time)
#		print(BPM)
#		print(PIP)
#		print(VTi_max)
#		print(peep)
#		print(P_plat)
#		print(trigflow)
#		print(pump_pressure)
	#	GPIO.output(22, GPIO.LOW)
		clock_t2 = time()
		volFlow_rate = self.rate()
		indiff= self.ABP_pressure()
		exp_press_array.apppend(indiff)
		packet_exhalation = ('C@' + str(round(indiff,2)) + ',' + str(round(volFlow_rate,2)) + ',' + str(int(volume)) + ',' + str(int((TITOT)*100))+'#')
		if(sending_time > 0.02):
		    try:
			ser.write(packet_exhalation)
		    except:
			print('BT Error sending in Exhalation conditional')
		    send_last_time = time()
		sending_time =time() - send_last_time 
#		print("THE TIME TAKEN IS")
#		print(cuurent_time - start_time)
                flow=self.rate()
	        if(indiff <= peep_val+1 and comp_flag == '1' and leak_comp_flag == '1'):
		    trigflow_comp = trigflow + running_avg + 3
		    comp_flag = '0'
		    running_avg = 0
#                flow=self.rate()
#		flow = flow - peep_hole[int(peep_val)]
                current_time=time()
		if(leak_comp_flag == '0'):
		    trigflow_comp = trigflow
           #    volume=0
	        if(comp_flag == '0' and flow > 1):
		    running_avg = (running_avg + flow)/2
		TOT_last = current_time - exhale_time_last
#		peep_val_send = indiff
		time_error = time() - time_error_last
		SDP_flag = 0
		loop = 1
#		print("THE TIME TAKEN IS")
#		print(current_time - start_time)
                if(flow>trigflow_comp or (current_time-start_time)>cycle_time):
		    RR_time=current_time-start_time
		    pump_pressure_now = pump_pressure
		    inhale_loop = "1"
		    leak_comp_flag = leak_comp()
		    patient_trigger_flow = flow
		    volume = 0
		    comp_flag = '1'
		    GPIO.output(35,GPIO.LOW)
 		    peep_val_send = temp_peep
 		    if(len(exp_press_array) > 10):
 			peep_val_send = exp_press_array[len(exp_press_array) - 4]
 		    del exp_press_array[:]
		   # GPIO.output(19,GPIO.HIGH)
		    sol.ChangeDutyCycle(100)
                    motor_1.ChangeDutyCycle(100)
		    if(P_plat <= 22):
			motor_1.ChangeDutyCycle(80)
#		    if(P_plat <= 17):
#			motor_1.ChangeDutyCycle(65)
#			pump_pressure_now = pump_pressure /2
 #                   RR_time=current_time-start_time
		    if(time_error < 0.7):
			time_error_set = 1
#			try:
#			    ser.write('ACK24')
#			except:
#			    print('unable to send data')
		    if(time_error > 0.7):
			time_error_set = 0
#			try:
#			    ser.write('ACK24')
#			except:
#			    print('unable to send data')
		    if(float(ratio) <= 0.25):
			ratio_set = 1
#			try:
#			    ser.write('ACK12')
#			except:
#			    print('unable to send data')
		    if(float(ratio) >= 0.75):
			ratio_set = 1
#			try:
#			    ser.write('ACK13')
#			except:
#			    print('unable to send data')
		    if(float(ratio) >0.25 and float(ratio) < 0.75):
			ratio_set = 0
#			try:
#			    ser.write('ACK22')
#			except:
#			    print('unable to send data')
		    if(RR_time > cycle_time):
			trigger='0'
			backup_counter = 0
		    else:
# 			backup_counter = backup_counter +1 
# 			if(backup_counter >= 4 and back_backup == 1):
# 			    sol.ChangeDutyCycle(0)
# 			    motor_1.ChangeDutyCycle(peep_open)
# 			    try:
# 				f = open("/home/pi/AgVa_5.0/backup.txt","r")
# 				backup_setting = f.readline()
# 				f.close()
# 				f = open("/home/pi/AgVa_5.0/mode.txt","w")
# 				f.write(str(backup_setting))
# 				f.close()
# 				back_backup = 0
# 				ser.write("ACK48")
# 				change_setting = '1'
# 				break;
# 			    except:
# 				print("unable to go back to the original mode")
			trigger='1'
                    #motor_2.ChangeDutyCycle(pump_pressure)
                    indiff= self.ABP_pressure()
                  #  print(indiff)
                    time_elapsed_inhale=0
                    indiff = 0
		    volume=self.Flow()
#		    print('the value of volume is')
#		    print(volume)
		    if(volume == -9999):
			SDP_flag = 1
			print('well hrre you gooooooooooooooooooooooooooooooooo')
		    Pmean=0
		    del Pmean_array[:]
		    del inhale_array[:]
		    clock_t2= time()
#		    print('now it is')
#		    print(clock_t2)
		    volume=0
		    volume_peak_inhale=0
		    sending_time = 0.2
		    send_last_time=0
		    VTi_volume=0
		    flow_flag = 0
		    volFlow_rate = self.rate()
		    pump_flag = 0
		    volume_comp = 0
		    pump_pressure_now = pump_pressure/2
		    peak_insp_pressure = 0
		    peak_flow = volFlow_rate
		    rise_flag = '0'
                    while(( volume_comp <= VTi_max * 1.05)  and indiff < PIP and time_elapsed_inhale <= inhale_time):
                    #    motor_1.ChangeDutyCycle(pump_pressure)
                        #motor_2.ChangeDutyCycle(pump_pressure)
			GPIO.output(37, GPIO.HIGH)
                        t3 = time()
#			print("inhale loop")
                        indiff= self.ABP_pressure()
                        time_elapsed_inhale= t3-current_time
 #                       print(time_elapsed_inhale)
#			print(volume)
#			print(VTi_max)
#			print(indiff)
#			print(PIP)
			Pmean_array.append(indiff)
			if(indiff >= P_plat * 0.8 and rise_flag == '0'):
				rise_time = time_elapsed_inhale
				rise_flag = '1'
			if(SDP_flag == 0):
			    volume=self.Flow()
			    volume_comp = volume
			    volume_comp = volume_comp - ((volume_comp*leak_percentage)/100)
#			    print('hello its me')
			    volFlow_rate=self.rate()
			if(volFlow_rate > peak_flow):
			    peak_flow = volFlow_rate
#			if(volFlow_rate <  peak_flow * 0.40 and time_elapsed_inhale >= 0.5 and trigger == '1'):
#			    break 
			if(sending_time > 0.02):
			    packet_inhalation =('A@' + str(round(indiff,2)) + ','+str(round(volFlow_rate,2)) + ',' + str(int(volume)) + ',' + str(trigger) + '#')
			    try:
				ser.write(packet_inhalation)
			    except:
				print('BT error send Inhalation')
#			    print('packet size isssssssssss')
#			    print(len(packet_inhalation))
#			    print('packet conatins ')
#			    print(packet_inhalation)
			    send_last_time= time()
			sending_time=t3 - send_last_time
#			print('inhale_time is')
			if((indiff >= P_plat*motor_factor or time_elapsed_inhale >= 0.1) and flow_flag == 0): #(time_elapsed_inhale > 0.3 and flow_flag == 0) or indiff >= 10): #or indiff >= P_plat - 4): #or volFlow_rate >= flow_plat - 10):
			    flow_flag = 1
			    motor_1.ChangeDutyCycle(pump_pressure)
#			    print('CMHO reacged')
#			    print(indiff)
# 			if(P_plat <= 17 and (indiff >= P_plat/2 or time_elapsed_inhale >= 0.2) and flow_flag == 0): #(time_elapsed_inhale > 0.3 and flow_flag == 0) or indiff >= 10): #or indiff >= P_plat - 4): #or volFlow_rate >= flow_plat - 10):
# 			    flow_flag = 1
# 			    motor_1.ChangeDutyCycle(pump_pressure)
# 			if(flow_flag == 1 and volFlow_rate <= flow_plat):
# 			    motor_1.ChangeDutyCycle(pump_pressure)
			if(volFlow_rate >= flow_plat and flow_flag ==1):
			    flow_flag = 2
			if(flow_flag == 3):
			    if(volFlow_rate <= flow_plat and pump_pressure_now <= 98):
				pump_pressure_now = pump_pressure_now + 1
			        motor_1.ChangeDutyCycle(pump_pressure_now)
			    if(volFlow_rate >= flow_plat and pump_pressure_now >= 2): #and pump_pressure_now >( pump_pressure /2)):
			        pump_pressure_now = pump_pressure_now - 1
			        motor_1.ChangeDutyCycle(pump_pressure_now)
# 			if(pump_pressure_now <= pump_pressure and volFlow_rate <= flow_plat):
# 			    if(pump_flag == 0):
# 			        pump_pressure_now  = pump_pressure_now + 1
# 			 print(pump_pressure)
# 				pump_pressure_now = pump_pressure_now + 0.5
# 	#		    print('the pump_pressure now is')
# 	#		    print(pump_pressure_now)
# 	#		    print('pump pressure is')
# 	#		    print(pump_pressure)
# 			    motor_1.ChangeDutyCycle(pump_pressure_now)
# 			elif(volFlow_rate > flow_plat and pump_pressure_now >( pump_pressure /2)):
# 			    pump_pressure_now = pump_pressure_now - 0.5
# 			    motor_1.ChangeDutyCycle(pump_pressure_now)
# 			    pump_flag = 1
#			print(inhale_time)
			inhale_array.append(indiff)
			VTi_volume = max(volume,VTi_volume)
			peak_insp_pressure = max(peak_insp_pressure, indiff)
			volume_peak_inhale=max(volume_peak_inhale,volFlow_rate)
		    if(indiff >= (PIP+2)):
			pressure_count = pressure_count +1
			if(pressure_count >= 3):
		            try:
		  	        ser.write('ACK07')
			    except:
		      	        print('unable to send')
		    else:
			pressure_count = 0
			try:
			    ser.write('ACK57')
			except:
			    print('unable to send')
		    if(indiff <= (P_plat)*0.5):
			pressure_low_count = pressure_low_count +1
			if(pressure_low_count >= 3):
		            try:
		  	        ser.write('ACK62')
			    except:
		      	        print('unable to send')
		    else:
			pressure_low_count = 0
			try:
			    ser.write('ACK72')
			except:
			    print('unable to send')
		    GPIO.output(37, GPIO.LOW)
		    MVi_array.append(VTi_volume)
		    VTi_volume = VTi_volume - ((VTi_volume*leak_percentage)/100)
                    flag=1
		    loop = 0
		    if(VTi_volume < (VTi_max*0.8) and trigger == '0' and indiff >= PIP and patient_status == 1):
		        ser.write("ACK61")
		    else:
			ser.write("ACK71")
		    if(VTi_volume < (VTi_max*0.8) and trigger == '0' and pump_pressure >= 95 and patient_status == 1):
			ser.write("ACK60")
		    else:
			ser.write("ACK70")
		    toggle_switch = str(toggle())
	#	    print("the toffle switch in here issssssssssssssssssssssssssss " + toggle_switch)
		#    motor_1.ChangeDutyCycle(peep)
		    sleep(0.01)
		   # GPIO.output(35, GPIO.HIGH) # low level triggered relay
#
		    if(toggle_switch == "0"):
#			print("got in the first statement")
		        motor_1.ChangeDutyCycle(peep_open)
			sol.ChangeDutyCycle(0)
		    if(toggle_switch == "1"):
			GPIO.output(33, GPIO.HIGH)
		        motor_1.ChangeDutyCycle(0)
			sol.ChangeDutyCycle(100)
	#		sleep(0.01)
			GPIO.output(35, GPIO.HIGH)
		    if(VTi_volume < (VTi_max * 0.8)):
			volume_count = volume_count + 1
			if(volume_count >= 5):
			    buzzer(7,1)
			    try:
			        ser.write('ACK36')
			    except:
			        print('unable to send')
		    else:
			buzzer(7,0)
			volume_count = 0
			try:
			    ser.write('ACK46')
			except:
			    print('unable to send')
		    inhale_array_length = len(inhale_array)
		    if(inhale_array_length % 2 != 0):
			inhale_array.append(0)
			inhale_array_length = inhale_array_length + 1
		    inhale_first_half = inhale_array[0:inhale_array_length/2]
		    inhale_second_half = inhale_array[inhale_array_length/2:]
		    first_max = max(inhale_first_half)
		    second_max = max(inhale_second_half)
		    if(first_max - P_plat > 0.5 and motor_factor > 0.2):
		        motor_factor = motor_factor - 0.01
			if(first_max - P_plat > 2):
			    motor_factor = motor_factor - 0.01
		    if(P_plat - first_max > 0.5 and motor_factor <= 1.0):
			motor_factor = motor_factor + 0.01
			if(P_plat - first_max > 2):
			    motor_factor = motor_factor + 0.01
		    try:
		        Pmean= sum(Pmean_array)/len(Pmean_array)
		    except:
			print("zero exception error")
		    try:
		        if(compliance_flag == 0):
		            compliance_array.append(VTi_volume/(peak_insp_pressure - peep_val))
			    if(len(compliance_array) == 2):
			        compliance = (sum(compliance_array)/len(compliance_array)) * 0.7
			        pressure = (VTi_max/compliance) + peep_val
			        if(pressure > PIP):
				    pressure  = PIP - 2
	        	        P_plat_value=find_nearest(P_plat_array,pressure)
	        	        index= P_plat_array.index(P_plat_value)
				inst_pump_pressure = pump_pressure_array[index]
				if(inst_pump_pressure > pump_pressure):
	        	            pump_pressure = pump_pressure_array[index]  
			        compliance_flag = 1
		    except:
			print("error")
#		    print('Pmean is')
#		    print('Here it comessssssssss')
#		    print(int(VTi_volume))
#		    print(VTi_max)
#		    print(peak_insp_pressure)
#		    print(patient_status)
                    RR.append(RR_time)
#		    print(pump_pressure_high)
		    indiff = int(indiff)
		    if(VTi_volume >= VTi_max  and time_elapsed_inhale < (inhale_time * 0.8) and trigger == '0' and compliance_flag == 1 and ABP_flag == 0 and SDP_flag == 0 and patient_status == 1 and pump_pressure >= 10 and pump_pressure <= 99):
			pump_pressure = pump_pressure - 1
			P_plat = indiff
#			print('-1')
		    elif(VTi_volume < VTi_max and ABP_flag == 0 and compliance_flag == 1 and SDP_flag == 0 and indiff <= PIP and patient_status ==1 and pump_pressure >= 5 and pump_pressure <= 94):
			if((VTi_max - VTi_volume) > 100 and leak_percentage < 70 and VTi_max > 100):
			    if(((VTi_max - VTi_volume)/30) > 5):
				pump_pressure = pump_pressure + 5
			    else:
			        pump_pressure = pump_pressure + ((VTi_max - VTi_volume)/30)
			elif(VTi_max > 100 ):
			    pump_pressure = pump_pressure + 1
			if(VTi_max <= 100):
			    pump_pressure = pump_pressure + 5
			P_plat = indiff
		    if(pump_pressure > 95):
			pump_pressure = 95
		    if(patient_status == 1 and peep_val_send < peep_val and peep < 90):
			peep = peep + 2
		    elif(patient_status == 1 and peep_val_send > peep_val and peep > 5 and peep < 90):
			peep = peep - 1
#			print("reducing the peeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeep")
#		    print("Diagnostics")
#		    print(VTi_max)
#		    print(VTi_volume)
#		    print(peak_insp_pressure)
#		    print(patient_status)
#		    print(P_plat)
#		    print(pump_pressure)
		if(flag == 1):
	   	    patient_set = 0
		    if((peak_insp_pressure <= P_plat * 0.5)  and patient_set == 0 ):
			buzzer(5,1)
			patient_set = 1
			patient_status = 0
		        print('disconnection')
#			trigger = '0'
			try:
			    ser.write('ACK05')
			except:
			    print('not able to send')
		       # GPIO.output(22, GPIO.HIGH)
		    if((peak_insp_pressure> P_plat * 0.5) and patient_set == 0):
			buzzer(5,0)
			patient_set = 1
			patient_status = 1
#			trigger = '1'
			try:
			    ser.write('ACK06')
			except:
			    print('not able to send')
                    if(len(RR) >= 5):
                        BPM = ( RR[1] + RR[2] + RR[3] + RR[4]) /4
                        BPM = ceil(60/BPM)
			if(BPM > 70):
			    try:
				ser.write('ACK09')
			    except:
				print('unable to send')
			else:
			    try:
				ser.write('ACK59')
			    except:
				print('unable to send')
 #                       print('BPM is')
  #                      print(BPM)
                        RR.pop(0)
		    if(len(MVi_array) >= 5):
		        MVi= (MVi_array[1] + MVi_array[2] + MVi_array[3] + MVi_array[4])/4
			MVi = MVi * BPM
#		        print('MVi value is')
#		        print(MVi)
		        MVi_array.pop(0)
		        packet_end_inhalation = ('B@' + str(round(peak_insp_pressure,2)) + "," + str(int(VTi_volume)) + "," + str(round(volume_peak_inhale,2)) + "," + str(round(Pmean,2)) + "," + str(round((MVi/1000),2))  + "," + str(round(patient_trigger_flow,2)) + "," + str(round(time_elapsed_inhale,2)) + "," + str(round(indiff,2)) + "," + str('0') + "," + str(round(rise_time,2)) + '#')
#		        print('packet length is')
#		        print(packet_end_inhalation)
		        try:
		            ser.write(packet_end_inhalation)
		        except:
			    print('BT Errror in end of inhalation loop')
                if(flag==1):
		    time_error_last = time()
#		    del exp_press_array[:]
#		    del exp_flow_array[:]
		    time_elap = 0
		    inhale_loop = "0"
		    q= time()
		    volume = 0
		    sending_time = 200
		    send_last_time = 0
		    volume_peak_exhale = 0
		    exhale_break = '0'
		    while(time_elap < 0.1 or (toggle_switch == "1" and time_elap < 0.4)):
			w=time()
			time_elap=w-q
#			exp_flow = self.rate()
#			exp_flow_array.append(exp_flow)
                        indiff = self.ABP_pressure()
			exp_press_array.append(indiff)
			peak_insp_pressure = int(peak_insp_pressure)
			if(peak_insp_pressure <= 0):
			    peak_insp_pressure = 1
			ratio = 1.0
                        if(ratio > 0.8 and time_elap >= 0.2 and toggle_switch == "1" and exhale_break == '0'):
                            GPIO.output(35, GPIO.LOW)
                            sleep(0.05)
                            motor_1.ChangeDutyCycle(peep)
			    exhale_break = '1'
                        elif(ratio >= 0.15 and ratio <= 0.4 and time_elap >= 0.4 and P_plat < 28 and toggle_switch == "1" and exhale_break == '0'):
                            GPIO.output(35, GPIO.LOW)
                            sleep(0.05)
                            motor_1.ChangeDutyCycle(peep)
			    exhale_break = '1'
                        elif(ratio >= 0.15 and ratio <= 0.4 and time_elap >= 0.4 and P_plat >=28 and toggle_switch == "1" and exhale_break == '0'):
                            GPIO.output(35, GPIO.LOW)
                            sleep(0.05)
                            motor_1.ChangeDutyCycle(peep)
			    exhale_break = '1'
                        elif(ratio > 0.4 and ratio <= 0.8 and time_elap >= 0.3 and toggle_switch == "1" and exhale_break == '0'):
                            GPIO.output(35, GPIO.LOW)
                            sleep(0.05)
                            motor_1.ChangeDutyCycle(peep)
			    exhale_break = '1'
                        elif(ratio <0.15 and ratio >=0.00 and time_elap >= 0.4 and toggle_switch == "1" and exhale_break == '0'):
                            GPIO.output(35, GPIO.LOW)
                            sleep(0.05)
                            motor_1.ChangeDutyCycle(peep)
			    exhale_break = '1'
			diff = self.SDP_pressure()
			if(SDP_flag == 0):
			    volFlow_rate = self.rate()
			    volume=self.Flow()
			if(diff == 700 or diff == -700 or SDP_flag == 1):
			    try:
				f = open("/home/pi/AgVa_5.0/mode.txt","w")
				f.write(str('11'))
				f.close()
				try:
				    ser.write('ACK30')
				except:
				    print('unable to send data')
			    except:
				print('unable to write mode')
			packet_exhalation= ('C@' + str(round(indiff,2)) + ',' + str(round(volFlow_rate,2)) +',' + str(int(volume)) + ',' + str(int((TITOT)*100)) + '#')
			if(sending_time > 0.02):
			    try:
				ser.write(packet_exhalation)
			    except:
				print('BT exhalation error')
			    send_last_time = time()
			sending_time = w- send_last_time
#			print('packet exhalation size  is')
#			print((packet_exhalation))
			volume_peak_exhale= min(volume_peak_exhale, volFlow_rate)
#			print('packet_exhal;ation conatins is')
		    temp_peep = self.ABP_pressure()
		    if(toggle_switch == "1" and volFlow_rate <= (volume_peak_exhale * 0.05) and indiff <= peep_val):
                        sol.ChangeDutyCycle(100)
#			sleep(0.3)
		#	motor_1.ChangeDutyCycle(peep)
                    new_time=time()
                    time_elapsed_exhale=0.0
                    diff=self.SDP_pressure()
		    start_time=current_time
		    time_elapsed_exhale_flow = 0
		    sending_time=200
		    send_last_time = 0
		    sol_flag  = 0
                    while((time_elapsed_exhale<(cycle_time-inhale_time-0.5) and (diff <= 0 or indiff > peep_val*1.5)) or (SDP_flag ==1 and time_elapsed_exhale<(cycle_time-inhale_time-0.5))):
                        time_elapsed_exhale=time()-new_time
			exp_flow = self.rate()
			exp_flow_array.append(exp_flow)
                        indiff = self.ABP_pressure()
			exp_press_array.append(indiff)	
			#indiff = self.ABP_pressure()
			if(toggle_switch == "1" and sol_flag == 0 and  volFlow_rate <= (volume_peak_exhale*0.05) and indiff <= peep_val):
			    sol.ChangeDutyCycle(100)
	#		    sleep(0.3)
			    sol_flag  = 1
			if(SDP_flag == 0):
           		    volume=self.Flow()
			    diff = self.SDP_pressure()
			    volFlow_rate=self.rate()
 			if(volFlow_rate < -5):
 			    time_elapsed_exhale_flow = time_elapsed_exhale
			volume_peak_exhale=min(volume_peak_exhale,volFlow_rate)
			packet_exhalation = ('C@' + str(round(indiff,2)) + ',' + str(round(volFlow_rate,2)) + ',' + str(int(volume)) + ',' + str(int((TITOT)*100))+'#')
			if(sending_time > 0.02):
			    try:
				ser.write(packet_exhalation)
			    except:
				print('BT Error sending in Exhalation conditional')
			    send_last_time = time()
			sending_time =time() - send_last_time 
                        start_time=current_time
                    #    print(start_time)
#		    flow_for_peep = 0.25 * (min(exp_flow_array))
#                    print(flow_for_peep)
 #                   value1 = find_nearest(exp_flow_array , flow_for_peep)
  #                  peep_array_index = exp_flow_array.index(value1)
#                    peep_val_send = exp_press_array[peep_array_index]
#		    peep_val_send = temp_peep
		    flag=0
		    loop = 1
		    inhale_loop = "1"
		    exhale_time_last = time()
		    time_elapsed_exhale = time_elapsed_exhale + 0.5
#		    if(len(exp_press_array) > 10):
 #  		        peep_val_send = exp_press_array[len(exp_press_array) - 3]
		    TITOT=(time_elapsed_inhale/(time_elapsed_inhale + time_elapsed_exhale + TOT_last))
#		    print('peak volume flow rate exhale')
#		    print(volume_peak_exhale)
		    if(int(peep_val_send) <= peep_val and patient_status == 1 and peep < 100 and peep_open < 100):
			if(toggle_switch == "1"):
			    peep = peep + 2
			else:
			    peep_open = peep_open + 2
		    if(int(peep_val_send) > peep_val and patient_status == 1 and peep > 0 and peep_open > 0):
			if(toggle_switch == "1"):
			    peep = peep - 1
			else:
			    peep_open = peep_open - 1
		    if(int(peep_val_send) > peep_val+2):
			peep_count = peep_count + 1
			if(leak_percentage > 20 and trigger == '0'):
			    ser.write("ACK39")
			if(peep_count >=3 and peep_set == 0):
			    buzzer(4,1)
			    peep_set = 1
			    try:
			        ser.write('ACK08')
			    except:
			        print('unable to send')
		    else:
			if(peep_set == 1):
			    buzzer(4,0)
			    peep_set = 0
			peep_count = 0
			ser.write("ACK49")
			try:
			    ser.write('ACK58')
			except:
			    print('unable to send')
		    MVe_array.append(volume)
		    FiO2 = self.FiO2()
                    self.power()
#		    print('FiO2 is')
#		    print(FiO2)
		    if(toggle_switch == '1'):
		        time_elapsed_exhale = time_elapsed_exhale + 0.4
			time_elapsed_exhale_flow = time_elapsed_exhale_flow + 0.4
		    else:
			time_elapsed_exhale = time_elapsed_exhale + 0.1
			time_elapsed_exhale_flow = time_elapsed_exhale_flow + 0.1
		    if(len(MVe_array)>=5):
		        MVe= (MVe_array[1] + MVe_array[2] + MVe_array[3] + MVe_array[4])/4
			MVe = MVe * BPM
		        MVe_array.pop(0)
#		        print('MVe is')
		        try:
			    leak_percentage = ((MVi - MVe)/MVi)*100
			    if(leak_percentage <= -1):
			        leak_percentage = 0
		        except:
			    leak_percentage = 0
#		        print(MVe)
		   # packet_end_exhalation = 'ABC'
		        packet_end_exhalation = ('D@' + str(round(peep_val_send,2)) + ',' + str(int(BPM)) + ',' + str(FiO2) + ',' + str(round(volume_peak_exhale,2)*-1)  + "," + str(round((MVe/1000),2)) + "," + str(leak_percentage)+ "," + str(trigflow_comp - trigflow) + "," + str(volume)  + "," + str(time_elapsed_exhale_flow) + '#')
#			print('string issssssssssssssssssssssssssssssssssssss')
#			print(packet_end_exhalation)
		        try:
	                    ser.write(packet_end_exhalation)
		        except:
			    print('BT send error in end of exhalation')
	#	    try:
	#	        with open('/home/pi/AgVa_5.0/data.csv', mode='a') as data:
   	#	            data = csv.writer(data, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
	#	            now = datetime.datetime.now()
	#	            data.writerow([str(now),str(self.measure_temp()),str(Pmean),str(volume),str(volume_peak_inhale),str(trigger),str(BPM),str(MVi),str(ratio), str(volume_peak_exhale),str(MVe)])
	#	    except:
	#		print('data file open error')
        except KeyboardInterrupt:
            return

    def PC_CMV(self):
	global peep_first,volume_flag,lock,peep_factor,IH_time, EH_time, IHold, EHold,exp_flow_array,exp_press_array,peep,peep_open,time_elapsed_exhale_flow ,rise_time, rise_flag,inhale_array,motor_factor,pressure_low_count,inhale_loop,toggle_switch,loop,peep,patient_set,exhale_time_last, TOT_last,flow_plat,TITOT,ABP_flag,peep_count,pressure_count, SDP_flag,pump_pressure_low,patient_status,pump_pressure, pump_pressure_high,ratio_set,time_error_set,thread_mode_status,time_error, time_error_last,prev_mode,data,P_plat_value,ratio,RR_time,indiff, RR, BPM, MVi_array, MVi,volume, volume_peak_inhale, trigger, max_value, volume_peak_exhale, MVe_array, MVe,clock_t2
        volume=0
	volume_flag = 0
	lock = 0
	peep_factor = 2.0
	time_elapsed_exhale_flow = 0
	rise_time = 0
	rise_flag = '0'
	pressure_low_count = 0
	patient_set = 0
	exhale_time_last = time()
	TOT_last = time()
	inhale_array = []
	motor_factor = 0.5
	del exp_press_array[:]
	del exp_flow_array[:]
	del MVi_array[:]
	del MVe_array[:]
	peep_count = 0
	loop = 1
	peep_set = 0
	pressure_count = 0
	ratio_set = 0
	GPIO.output(35, GPIO.LOW)
	GPIO.output(7, GPIO.LOW)
        GPIO.output(40, GPIO.HIGH)
	VTi_volume = 0
	leak_percentage = 0
	inhale_loop = "0"
	flag = 0
	time_error_set = 0
	sol.ChangeDutyCycle(100)
	thread_mode_status = True
	start_time = time() - 5000
	time_error_last = time()
	sending_time = 0.2
	flag = 0
	peep_val_send = 0
	temp_peep = 0
        try:
            while True:
		prev_mode = 11
	#	print('we are in PC_CMV mode')
		data= self.read_data()
		hold_check()
	#	print(data)
		if(data == 1):
		    break;
#		print(inhale_time)
#		print(BPM)
#		print(PIP)
#		print(VTi_max)
#		print(peep)
#		print(P_plat)
		indiff= self.ABP_pressure()
		exp_press_array.append(indiff)
		trigger = 0
	#	GPIO.output(22, GPIO.LOW)
		clock_t2 = time()
		volFlow_rate = self.rate()
#		indiff= self.ABP_pressure()
		packet_exhalation = ('C@' + str(round(indiff,2)) + ',' + str(round(volFlow_rate,2)) + ',' + str(int(volume)) + ',' + str(int((TITOT)*100))+'#')
		if(sending_time > 0.02):
		    try:
			ser.write(packet_exhalation)
		    except:
			print('BT Error sending in Exhalation conditional')
		    send_last_time = time()
		sending_time =time() - send_last_time 
#		print('pump_pressure is')
#		print(pump_pressure)
                flow=self.rate()
                current_time=time()
		TOT_last = current_time - exhale_time_last
               # if(indiff <= peep_val+2 and flow >= -10):
                #    sol.ChangeDutyCycle(100)
          #      volume=0
	  #	peep_val_send = indiff
		SDP_flag = 0
		loop = 1
		time_error = time() - time_error_last
                if((current_time-start_time)>cycle_time):
		    pump_pressure_now = pump_pressure
		    inhale_loop = "1"
		    del inhale_array[:]
		    GPIO.output(33, GPIO.HIGH)
		    GPIO.output(7, GPIO. LOW)
                    GPIO.output(40, GPIO.HIGH)
                    volume  = 0
		    peep_val_send =  temp_peep
 		    if(len(exp_press_array) > 10):
 			peep_val_send = exp_press_array[len(exp_press_array) - 4]
 		    del exp_press_array[:]
		    GPIO.output(35, GPIO.LOW)
		    #sol.ChangeDutyCycle(100)
                    motor_1.ChangeDutyCycle(100)
		    if(P_plat <= 22):
			motor_1.ChangeDutyCycle(80)
#		    if(P_plat <= 17):
#			motor_1.ChangeDutyCycle(65)
	#		pump_pressure_now = pump_pressure /2
                    RR_time=current_time-start_time
		    if(time_error < 0.7):
			time_error_set = 1
#			try:
#			    ser.write('ACK24')
#			except:
#			    print('unable to send data')
		    if(time_error > 0.7):
			time_error_set = 0
#			try:
#			    ser.write('ACK24')
##			    print('unable to send data')
		    if(float(ratio) <= 0.25):
			ratio_set = 1
#			try:
#			    ser.write('ACK12')
#			except:
#			    print('unable to send data')
		    if(float(ratio) >= 0.75):
			ratio_set = 1
#			try:
#			    ser.write('ACK13')
#			except:
#			    print('unable to send data')
		    if(float(ratio) >0.25 and float(ratio) < 0.75):
			ratio_set = 0
#			try:
#			    ser.write('ACK22')
#			except:
#			    print('unable to send data')
#		    if(RR_time > cycle_time):
#			trigger='0'
#		    elif(flow>trigflow):
#			trigger='1'
                    #motor_2.ChangeDutyCycle(pump_pressure)
                    indiff= self.ABP_pressure()
                  #  print(indiff)
                    time_elapsed_inhale=0
                    indiff = 0
		    volume=self.Flow()
		    if(volume == -9999):
			SDP_flag = 1
		    Pmean=0
		    del Pmean_array[:]
		    clock_t2= time()
#		    print('now it is')
#		    print(clock_t2)
		    volume=0
		    volume_peak_inhale=0
		    sending_time = 0.2
		    send_last_time=0
		    VTi_volume=0  
		    flow_flag = 0
		    volFlow_rate = self.rate()
		    pump_pressure_now = pump_pressure /2
		    peak_insp_pressure = 0
		    pump_flag = 0
		    rise_flag = '0'
                    while(indiff <= PIP and time_elapsed_inhale <= inhale_time):
                        #motor_1.ChangeDutyCycle(pump_pressure)
                        #motor_2.ChangeDutyCycle(pump_pressure)
			GPIO.output(37, GPIO.LOW)
                        #GPIO.output(7, GPIO.LOW)
			#GPIO.output(40, GPIO.LOW)
			t3 = time()
                        indiff= self.ABP_pressure()
                        time_elapsed_inhale= t3-current_time
                   #     print(indiff)
			Pmean_array.append(indiff)
			if(indiff >= P_plat * 0.8 and rise_flag == '0'):
				rise_time = time_elapsed_inhale
				rise_flag = '1'
			if(SDP_flag == 0):
			    volume=self.Flow()
			    volFlow_rate=self.rate()
			if(sending_time > 0.02):
			    packet_inhalation =('A@' + str(round(indiff,2)) + ','+str(round(volFlow_rate,2)) + ',' + str(int(volume)) + ',' + str(trigger) + '#')
			    try:
				ser.write(packet_inhalation)
			    except:
				print('BT error send Inhalation')
#			    print('packet size isssssssssss')
#			    print(len(packet_inhalation))
#			    print('packet conatins ')
	#		    print(packet_inhalation)
			    send_last_time= time()
			sending_time=t3 - send_last_time
			if((indiff >= P_plat*motor_factor or time_elapsed_inhale >= 0.5) and flow_flag == 0): #(time_elapsed_inhale > 0.3 and flow_flag == 0) or indiff >= 10): #or indiff >= P_plat - 4): #or volFlow_rate >= flow_plat - 10):
			    flow_flag = 1
			    motor_1.ChangeDutyCycle(pump_pressure)
# 			    print('CMHO reacged')
# 			    print(indiff)
# 			if(P_plat <= 17 and (indiff >= P_plat/2 or time_elapsed_inhale >= 0.2) and flow_flag == 0): #(time_elapsed_inhale > 0.3 and flow_flag == 0) or indiff >= 10): #or indiff >= P_plat - 4): #or volFlow_rate >= flow_plat - 10):
# 			    flow_flag = 1
# 			    motor_1.ChangeDutyCycle(pump_pressure)
# 			if(flow_flag == 1 and volFlow_rate <= flow_plat):
# 			    motor_1.ChangeDutyCycle(pump_pressure)
			if(volFlow_rate >= flow_plat and flow_flag ==1):
			    flow_flag = 2
			if(flow_flag == 3):
			    if(volFlow_rate <= flow_plat and pump_pressure_now <= 98):
				pump_pressure_now = pump_pressure_now + 1
			        motor_1.ChangeDutyCycle(pump_pressure_now)
			    if(volFlow_rate >= flow_plat and pump_pressure_now >= 2): #and pump_pressure_now >( pump_pressure /2)):
			        pump_pressure_now = pump_pressure_now - 1
			        motor_1.ChangeDutyCycle(pump_pressure_now)
# 			if(pump_pressure_now <= pump_pressure and volFlow_rate <= flow_plat):
# 			    if(pump_flag == 0):
# 			        pump_pressure_now  = pump_pressure_now + 1
# 			    else:
# 				pump_pressure_now = pump_pressure_now + 0.5
# 	#		    print('the pump_pressure now is')
# 	#		    print(pump_pressure_now)
# 	#		    print('pump pressure is')
# 	#		    print(pump_pressure)
# 			    motor_1.ChangeDutyCycle(pump_pressure_now)
# 			elif(volFlow_rate > flow_plat and pump_pressure_now >( pump_pressure /2)):
# 			    pump_pressure_now = pump_pressure_now - 0.5
# 			    motor_1.ChangeDutyCycle(pump_pressure_now)
# 			    pump_flag = 1
#			print(time_elapsed_inhale)
#			print(inhale_time)
			inhale_array.append(indiff)
			VTi_volume = max(volume,VTi_volume)
			peak_insp_pressure = max(peak_insp_pressure, indiff)
			volume_peak_inhale=max(volume_peak_inhale,volFlow_rate)
		    if(IHold == '1'):
			motor_1.ChangeDutyCycle(int(pump_pressure*0.9))
			GPIO.output(33, GPIO.HIGH)
			print("entering inspiratory hold")
			print(IH_time)
			ser.write("ACK66")
			instant_time = time()
		        sending_time = 0.2
		        send_last_time=0
			while(time() - instant_time < IH_time):
			    print(sending_time)
			    indiff= self.ABP_pressure()
			    volFlow_rate=self.rate()
			    if(sending_time > 0.02):
			        packet_inhalation =('A@' + str(round(indiff,2)) + ','+str(round(volFlow_rate,2)) + ',' + str(int(volume)) + ',' + str(trigger) + '#')
			        try:
				    ser.write(packet_inhalation)
			        except:
				    print('BT error send Inhalation')
			        send_last_time= time()
			    sending_time=time() - send_last_time
		    IHold = '0'
		    GPIO.output(33, GPIO.LOW)
		    if(indiff >= (P_plat+3)):
			pressure_count = pressure_count +1
			if(pressure_count >= 3):
		            try:
		  	        ser.write('ACK07')
			    except:
		      	        print('unable to send')
		    else:
			pressure_count = 0
			try:
			    ser.write('ACK57')
			except:
			    print('unable to send')
		    if(indiff <= (P_plat)*0.5):
			pressure_low_count = pressure_low_count +1
			if(pressure_low_count >= 3):
		            try:
		  	        ser.write('ACK62')
			    except:
		      	        print('unable to send')
		    else:
			pressure_low_count = 0
			try:
			    ser.write('ACK72')
			except:
			    print('unable to send')
		    GPIO.output(37,GPIO.LOW)
                    flag=1
		    loop = 0
		#    motor_1.ChangeDutyCycle(peep)
		    toggle_switch = str(toggle())
		    if(EHold == '1'):
			toggle_switch ='1'
		    sleep(0.01)
		    GPIO.output(35, GPIO.HIGH) # low level triggered relay
		    if(toggle_switch == "0"):
			print("got in the first statement")
			print(peep)
		        motor_1.ChangeDutyCycle(peep_open)
	#		sol.ChangeDutyCycle(0)
		    if(toggle_switch == "1"):
			print("got in the 2nd statement")
			GPIO.output(33, GPIO.HIGH)
		        motor_1.ChangeDutyCycle(peep)
			#sol.ChangeDutyCycle(0)
			GPIO.output(40, GPIO.LOW)
			GPIO.output(7, GPIO.HIGH)

		    try:
		        Pmean= sum(Pmean_array)/len(Pmean_array)
		    except:
			print("zero exception error")
#		    print('Pmean is')
		    inhale_array_length = len(inhale_array)
		    if(inhale_array_length % 2 != 0):
			inhale_array.append(0)
			inhale_array_length = inhale_array_length + 1
		    inhale_first_half = inhale_array[0:inhale_array_length/2]
		    inhale_second_half = inhale_array[inhale_array_length/2:]
		    first_max = max(inhale_first_half)
		    second_max = max(inhale_second_half)
		    if(first_max - P_plat > 0.5 and motor_factor > 0.2):
		        motor_factor = motor_factor - 0.01
			if(first_max - P_plat > 2):
			    motor_factor = motor_factor - 0.01
		    if(P_plat - first_max > 0.5 and motor_factor <= 1.0):
			motor_factor = motor_factor + 0.8
			if(P_plat - first_max > 2):
			    motor_factor = motor_factor + 0.05
	#	    print('P_plat is')
	#	    print(int(peak_insp_pressure))
	#	    print(P_plat)
#		    print(Pmean)
                    RR.append(RR_time)
#		    print('volume is')
#		    print(volume*0.4)
		    indiff = int(indiff)
		    if(indiff > P_plat and patient_status == 1 and ABP_flag == 0 and indiff < PIP and  pump_pressure >= 6 and pump_pressure <= 97):
			pump_pressure = pump_pressure - 1
			print('-1')
		    elif(indiff < P_plat and patient_status == 1 and ABP_flag == 0 and indiff < PIP and  pump_pressure >= 5 and pump_pressure <= 95):
			pump_pressure = pump_pressure + 1
			print('+1')
		    if(patient_status == 1 and peep_val_send < peep_val and peep <= peep_first + 10  and peep >= 4 and  peep < 90 and toggle_switch == '1'):
			peep = peep + 1
		    elif(patient_status == 1 and peep_val_send > peep_val and peep >= peep_first - 10  and peep > 5 and peep < 92 and toggle_switch == '1'):
			peep = peep - 1
#		    print('Vpeak Inspiratory')
#		    print(volume_peak_inhale)
		    MVi_array.append(VTi_volume)
#		    print('trigger is')
	#	    print("PEEEEEEEEEP FACTOR IS")
	#	    print(peep_factor)
#		    print(Pmean)
		if(flag == 1):
		    patient_set = 0
		    if((leak_percentage > 90 or indiff <= 5) and patient_set == 0 ):
			buzzer(5,1)
			patient_set = 1
			patient_status = 0
		        print('disconnection')
#			trigger = '0'
			try:
			    ser.write('ACK05')
			except:
			    print('not able to send')
		      #  GPIO.output(22, GPIO.HIGH)
		    if((leak_percentage <= 90 or indiff > 5) and patient_set == 0):
			buzzer(5,0)
			patient_set = 1
			patient_status = 1
#			trigger = '1'
			try:
			    ser.write('ACK06')
			except:
			    print('not able to send')
                    if(len(RR) >= 5):
                        BPM = ( RR[1] + RR[2] + RR[3] + RR[4]) /4
                        BPM = ceil(60/BPM)
			if(BPM > 70):
			    try:
				ser.write('ACK09')
			    except:
				print('unable to send')
			else:
			    try:
				ser.write('ACK59')
			    except:
				print('unable to send')
 #                       print('BPM is')
  #                      print(BPM)
                        RR.pop(0)
		    if(len(MVi_array) >= 5):
		        MVi= (MVi_array[1] + MVi_array[2] + MVi_array[3] + MVi_array[4])/4
			MVi = MVi * BPM
#		        print('MVi value is')
#		        print(MVi)
		        MVi_array.pop(0)
		        packet_end_inhalation = ('B@' + str(round(peak_insp_pressure,2)) + "," + str(int(VTi_volume - (VTi_volume*leak_percentage)/200)) + "," + str(round(volume_peak_inhale,2)) + "," + str(round(Pmean,2)) + "," + str(round((MVi/1000),2)) + "," + str('0') + "," + str(round(time_elapsed_inhale,2)) + "," + str(round(indiff,2)) + "," + str('0') + "," + str(round(rise_time,2)) + '#')
#		        print('packet length is')
		        print(packet_end_inhalation)
		        try:
		            ser.write(packet_end_inhalation)
		        except:
			    print('BT Errror in end of inhalation loop')
                if(flag==1):
		    time_error_last = time()
		    time_elap = 0
		    q= time()
		    del exp_press_array[:]
		    del exp_flow_array[:]
		    inhale_loop = "0"
		    volume = 0
		    buzzer(7,0)
		    sending_time = 200
		    send_last_time = 0
		    volume_peak_exhale = 0
		    toggle_switch = '1'
		    exhale_break = '0'
		    while(time_elap < 0.1 or (toggle_switch == "1" and time_elap < 0.3)):
			w=time()
			time_elap=w-q
			exp_flow = self.rate()
                        exp_flow_array.append(exp_flow)
                        indiff = self.ABP_pressure()
                        exp_press_array.append(indiff)			
                        #indiff = self.ABP_pressure()
			peak_insp_pressure = int(peak_insp_pressure)
			if(peak_insp_pressure <= 0):
			    peak_insp_pressure = 1
			ratio = 0.11 #peep_val/P_plat
                        if(ratio > 0.8 and time_elap >= 0.2 and toggle_switch == '1' and exhale_break == '0'):
                            GPIO.output(35, GPIO.LOW)
                            sleep(0.05)
        #                    motor_1.ChangeDutyCycle(peep)
			    exhale_break = '1'
                        elif(ratio >= 0.15 and ratio <= 0.4 and time_elap >= 0.4 and P_plat < 28 and toggle_switch == '1' and exhale_break == '0'):
                            GPIO.output(35, GPIO.LOW)
                            sleep(0.05)
         #                   motor_1.ChangeDutyCycle(peep)
			    exhale_break = '1'
                        elif(ratio >= 0.15 and ratio <= 0.4 and time_elap >= 0.4 and P_plat >=28 and toggle_switch == '1' and exhale_break == '0'):
                            GPIO.output(35, GPIO.LOW)
                            sleep(0.05)
          #                  motor_1.ChangeDutyCycle(peep)
			    exhale_break = '1'
                        elif(ratio > 0.4 and ratio <= 0.8 and time_elap >= 0.3 and toggle_switch == '1' and exhale_break == '0'):
                            GPIO.output(35, GPIO.LOW)
                            sleep(0.05)
           #                 motor_1.ChangeDutyCycle(peep)
			    exhale_break = '1'
                        elif(ratio <0.15 and ratio >=0.00 and time_elap >= 0.4 and toggle_switch == '1' and exhale_break == '0'):
                            GPIO.output(35, GPIO.LOW)
                            sleep(0.05)
            #                motor_1.ChangeDutyCycle(peep)
			    exhale_break = '1'
			if(SDP_flag == 0):
			    volFlow_rate = self.rate()
			    volume=self.Flow()
			packet_exhalation= ('C@' + str(round(indiff,2)) + ',' + str(round(volFlow_rate,2)) +',' + str(int(volume)) + ',' + str(int((TITOT)*100)) + '#')
			if(sending_time > 0.02):
			    try:
				ser.write(packet_exhalation)
			    except:
				print('BT exhalation error')
			    send_last_time = time()
			sending_time = w- send_last_time
#			print('packet exhalation size  is')
#			print((packet_exhalation))
			volume_peak_exhale= min(volume_peak_exhale, volFlow_rate)
# 		        if(toggle_switch == "1" and indiff <= peep_val):
#                             sol.ChangeDutyCycle(100)
# 			    GPIO.output(33, GPIO.LOW)
# 	#		    print("breaking point")
# 	#		    print(indiff)
# 			    holding_now = time()
# 			    holding_time = 0.001
# 			    while( time() - holding_now <= holding_time):
# 				indiff = self.ABP_pressure()
# 			        volFlow_rate = self.rate()
# 			        volume=self.Flow()
# 				packet_exhalation= ('C@' + str(round(indiff,2)) + ',' + str(round(0,2)) +',' + str(int(volume)) + ',' + str(int((TITOT)*100)) + '#')
# 				if(1 == 1):
# 			    	    try:
# 					ser.write(packet_exhalation)
# 			    	    except:
# 					print('BT exhalation error')
# 			    	    send_last_time = time()
# 				sending_time = w- send_last_time
# 				sleep(0.02)
#			print('packet_exhal;ation conatins is')
#			print(packet_exhalation)
                    temp_peep = self.ABP_pressure()
                    new_time=time()
                    time_elapsed_exhale=0.0
		    time_elapsed_exhale_flow  = 0
                    diff=self.SDP_pressure()
		    start_time=current_time
# 		    if(toggle_switch == "1" and indiff <= peep_val-peep_factor):
#                         sol.ChangeDutyCycle(100)
# 			holding_now = time()
# 			holding_time = 0.001
# 			sending_time = 2.0
# 			send_last_time = 0
# 			while( time() - holding_now <= holding_time):
# 			    indiff = self.ABP_pressure()
# 			    volFlow_rate = self.rate()
# 			    volume=self.Flow()
# 			    packet_exhalation= ('C@' + str(round(indiff,2)) + ',' + str(round(0,2)) +',' + str(int(volume)) + ',' + str(int((TITOT)*100)) + '#')
# 			    if(1 ==1 ):
# 			        try:
# 				    ser.write(packet_exhalation)
# 			    	except:
# 				    print('BT exhalation error')
# 			    	send_last_time = time()
# 			    sending_time = time()- send_last_time
# 			    sleep(0.02)
		    sending_time=200
		    send_last_time = 0
		    sol_flag  = 0
                    while((time_elapsed_exhale<(cycle_time-inhale_time-0.3) and diff <= 0) or (SDP_flag == 1 and time_elapsed_exhale<(cycle_time-inhale_time-0.5))):
                        time_elapsed_exhale=time()-new_time
		#	exp_flow = self.rate()
                 #       exp_flow_array.append(exp_flow)
                        indiff = self.ABP_pressure()
 			exp_press_array.append(indiff)			
			#indiff = self.ABP_pressure()

			#GPIO.output(40, GPIO.LOW)
                        #GPIO.output(7, GPIO.HIGH)
			if(toggle_switch == "1" and sol_flag  == 0 and ((indiff <= peep_val+2 and volFlow_rate >= -10) or (indiff <= peep_val-2))):
			    #sol.ChangeDutyCycle(100)
			    GPIO.output(33, GPIO.LOW)			
			    sol_flag = 1
			    holding_now = time()
			    holding_time = 0.001
			    while( time() - holding_now <= holding_time):
				indiff = self.ABP_pressure()
			        volFlow_rate = self.rate()
			        volume=self.Flow()
				packet_exhalation= ('C@' + str(round(indiff,2)) + ',' + str(round(0,2)) +',' + str(int(volume)) + ',' + str(int((TITOT)*100)) + '#')
				if(1 == 1):
			    	    try:
					ser.write(packet_exhalation)
			    	    except:
					print('BT exhalation error')
			    	    send_last_time = time()
				sending_time = time()- send_last_time
				sleep(0.02)
			if(SDP_flag == 0):
           		    volume=self.Flow()
			    volFlow_rate=self.rate()
			    diff = self.SDP_pressure()
			volume_peak_exhale=min(volume_peak_exhale,volFlow_rate)
 			if(volFlow_rate < -5):
 			    time_elapsed_exhale_flow = time_elapsed_exhale
			packet_exhalation = ('C@' + str(round(indiff,2)) + ',' + str(int(volFlow_rate)) + ',' + str(int(volume)) + ',' + str(int((TITOT)*100))+'#')
			if(sending_time > 0.02):
			    try:
				ser.write(packet_exhalation)
			    except:
				print('BT Error sending in Exhalation conditional')
			    send_last_time = time()
			sending_time =time() - send_last_time 
                        start_time=current_time
                    print(peep_val)
                    if(int(peep_val_send) >=  peep_val - 1 and int(peep_val_send) <= peep_val+1):
                        lock = 1
                    elif(int(peep_val_send) > peep_val+1 or int(peep_val_send) < peep_val -1):
                        lock = 0
                    print("LOCK STATUS")			
		    print(lock)
		    print(int(peep_val_send))
		    if(EHold == '1'):
			GPIO.output(33, GPIO.HIGH)
			print("Entering expiratory hold")
			instant_time = time()
			ser.write("ACK67")
		        sending_time = 0.2
		        send_last_time=0
			while(time() - instant_time < EH_time):
			    indiff= self.ABP_pressure()
	#		    print(time() - instant_time)
			    volFlow_rate=self.rate()
			    if(sending_time > 0.02):
			        packet_exhalation = ('C@' + str(round(indiff,2)) + ',' + str(round(volFlow_rate,2)) + ',' + str(int(volume)) + ',' + str(int((TITOT)*100))+'#')
			        try:
				    ser.write(packet_exhalation)
			        except:
				    print('BT error send Inhalation')
			        send_last_time= time()
			    sending_time=time() - send_last_time
			peep_val_send = indiff
			GPIO.output(33, GPIO.LOW)
		    EHold = '0'
		    flag=0
		    loop = 1
		    inhale_loop = "1"
		    exhale_time_last = time()
		    time_elapsed_exhale = time_elapsed_exhale + 0.5
		 #   if(len(exp_press_array) > 10):
		  #      peep_val_send =  exp_press_array[len(exp_press_array) - 3]
		    TITOT=(time_elapsed_inhale/(time_elapsed_inhale + time_elapsed_exhale + TOT_last))
	#	    print("PEEEEEEEEEEEEP VALUEEEEEEEEEEEEEEEEE")
	#	    print(peep_val_send)
	#	    print(peep_val)
		    if((peep_val_send) < peep_val and patient_status == 1 and toggle_switch == '1' and lock == 0):
		        peep_factor = peep_factor - (peep_val - peep_val_send)*0.1
			print("Subrarcting factor")
			print(peep_factor)
		    if((peep_val_send) > peep_val+1 and patient_status == 1 and toggle_switch == '1' and lock == 0):
			peep_factor = peep_factor + (peep_val_send - peep_val)*0.1
			print("Adding factor")
			print(peep_factor)
		    if(int(peep_val_send) > peep_val+2):
			peep_count = peep_count + 1
			if(leak_percentage > 20 and trigger == '0'):
			    ser.write("ACK39")
			if(peep_count >=3 and peep_set == 0):
			    buzzer(4,1)
			    peep_set = 1
			    try:
			        ser.write('ACK08')
			    except:
			        print('unable to send')
		    else:
			if(peep_set == 1):
			    buzzer(4,0)
			    peep_set = 0
			peep_count = 0
			ser.write("ACK49")
			try:
			    ser.write('ACK58')
			except:
			    print('unable to send')
		    MVe_array.append(volume)
	   	    FiO2 = self.FiO2()
		    self.power()
	#	    print('FiO2 is')
	#	    print(FiO2)
		    if(toggle_switch == '1'):
		        time_elapsed_exhale = time_elapsed_exhale + 0.4
			time_elapsed_exhale_flow = time_elapsed_exhale_flow + 0.4
		    else:
			time_elapsed_exhale = time_elapsed_exhale + 0.1
			time_elapsed_exhale_flow = time_elapsed_exhale_flow + 0.1
		    if(len(MVe_array)>=5):
		        MVe= (MVe_array[1] + MVe_array[2] + MVe_array[3] + MVe_array[4])/4
			MVe = MVe * BPM
		        MVe_array.pop(0)
		        try:
			    leak_percentage = ((MVi - MVe)/MVi)*100
			    if(leak_percentage <= -1):
			        leak_percentage = 0
		        except:
			    leak_percentage = 0
#		        print('MVe is')
#		        print(MVe)
		   # packet_end_exhalation = 'ABC'
		        packet_end_exhalation = ('D@' + str(round(peep_val_send,2)) + ',' + str(int(BPM)) + ',' + str(FiO2) + ',' + str(round(volume_peak_exhale,2)*-1) + "," + str(round((MVe/1000),2)) + "," + str(leak_percentage) + "," + str(0) + "," + str(volume) + "," + str(time_elapsed_exhale_flow) + '#')
#			print('string issssssssssssssssssssssssssssssssssssss')
			print(packet_end_exhalation)
		        try:
	                    ser.write(packet_end_exhalation)
		        except:
			    print('BT send error in end of exhalation')
	#	    try:
	#	        with open('/home/pi/AgVa_5.0/data.csv', mode='a') as data:
   	#	            data = csv.writer(data, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
	#	            now = datetime.datetime.now()
	#	            data.writerow([str(now),str(self.measure_temp()),str(Pmean),str(volume),str(volume_peak_inhale),str(trigger),str(BPM),str(MVi),str(ratio), str(volume_peak_exhale),str(MVe)])
	#	    except:
	#		print('data file open error')
	except KeyboardInterrupt:
	    return

    def VC_CMV(self):
	global peep_first,compliance_flag,volume_flag,lock,peep_factor,IH_time, EH_time, IHold, EHold,exp_press_array,exp_flow_array,peep,peep_open,time_elapsed_exhale_flow,rise_time, rise_flag,inhale_array,motor_factor,pressure_low_count,volume_comp,inhale_loop,pump_pressure_array,P_plat_array,compliance,compliance_array,toggle_switch,loop,peep,P_plat,exhale_time_last, TOT_last,patient_set,TITOT,P_plat_high,pressure_count,peep_count,volume_count, ABP_flag, SDP_flag, P_plat_low,flow_plat,ratio_set,pump_pressure,pump_pressure_high,pump_pressure_low,patient_status,time_error_set,thread_mode_status,time_error, time_error_last,prev_mode,data,P_plat_value,ratio,RR_time,indiff, RR, BPM, MVi_array, MVi,volume, volume_peak_inhale, trigger, max_value, volume_peak_exhale, MVe_array, MVe,clock_t2
        volume=0
	volume_flag = 0
	lock = 0
	peep_factor = 2.0
	time_elapsed_exhale_flow = 0
	rise_time = 0
	rise_flag = '0'
	pressure_low_count =0
	volume_comp = 0
	loop = 1
	peep_count = 0
	del MVi_array[:]
	del MVe_array[:]
	del exp_flow_array[:]
	del exp_press_array[:]
	inhale_array = []
	motor_factor = 0.5
	volume_count = 0
	exhale_time_last = time()
	TOT_last = time()
	pressure_count = 0
	patient_set = 0
	inhale_loop = "1"
	VTi_volume = 0
	leak_percentage = 0
	ratio_set = 0
	time_error_set = 0
	compliance_flag = 0
	GPIO.output(35, GPIO.LOW)
	del compliance_array[:]
	peep_set = 0
	volume_set = 0
	sol.ChangeDutyCycle(100)
	thread_mode_status = True
	start_time = time() - 5000
	time_error_last = time()
	flag = 0
	sending_time = 0.2
	peep_val_send = 0
	temp_peep = 0
        try:
            while True:
		prev_mode = 21
#		print('we are in VC_CMV mode')
		data= self.read_data()
		hold_check()
#		print(data)
		if(data == 1):
		    break;
#		print(inhale_time)
#		print(BPM)
#		print(PIP)
#		print(VTi_max)
#		print(peep)
#		print(P_plat)
		indiff= self.ABP_pressure()
		exp_press_array.append(indiff)
		trigger = 0
	#	GPIO.output(22, GPIO.LOW)
		clock_t2 = time()
		volFlow_rate = self.rate()
#		indiff= self.ABP_pressure()
		packet_exhalation = ('C@' + str(round(indiff,2)) + ',' + str(round(volFlow_rate,2)) + ',' + str(int(volume)) + ',' + str(int((TITOT)*100))+'#')
		if(sending_time > 0.02):
		    try:
			ser.write(packet_exhalation)
		    except:
			print('BT Error sending in Exhalation conditional')
		    send_last_time = time()
		sending_time =time() - send_last_time 
#		print('pump_pressure is')
#		print(pump_pressure)
                flow=self.rate()
                current_time=time()
                if(indiff <= peep_val+2 and flow >= -10):
                    sol.ChangeDutyCycle(100)
		TOT_last = current_time - exhale_time_last
       #         volume=0
	#	peep_val_send = indiff
		SDP_flag = 0
		time_error = time() - time_error_last
		loop = 1
                if((current_time-start_time)>cycle_time):
		    pump_pressure_now = pump_pressure
		    inhale_loop = "1"
		    GPIO.output(33, GPIO.LOW)
		    volume = 0
		    del inhale_array[:]
		    GPIO.output(35, GPIO.LOW)
		    sol.ChangeDutyCycle(100)
# 		    if((pump_pressure /2) < peep):
 		    peep_val_send = temp_peep
 		    if(len(exp_press_array) > 10):
			peep_val_send = exp_press_array[len(exp_press_array) - 4]
		    del exp_press_array[:]
                    motor_1.ChangeDutyCycle(100)
		    if(VTi_max <= 200):
			motor_1.ChangeDutyCycle(pump_pressure)
#	#	    if(P_plat <= 17):
	#		motor_1.ChangeDutyCycle(65)
# 			pump_pressure_now = pump_pressure /2
                    RR_time=current_time-start_time
		    if(time_error < 0.7):
			time_error_set = 1
#			try:
#			    ser.write('ACK24')
#			except:
#			    print('unable to send data')
		    if(time_error > 0.7):
			time_error_set = 0
#			try:
#			    ser.write('ACK24')
#			except:
#			    print('unable to send data')
		    if(float(ratio) <= 0.25):
			ratio_set = 1
#			try:
#			    ser.write('ACK12')
#			except:
#			    print('unable to send data')
		    if(float(ratio) >= 0.75):
			ratio_set = 1 
#			try:
#			    ser.write('ACK13')
#			except:
#			    print('unable to send data')
		    if(float(ratio) >0.25 and float(ratio) < 0.75):
			ratio_set = 0
#			try:
#			    ser.write('ACK22')
#			except:
#			    print('unable to send data')
	#	    if(RR_time > cycle_time):
	#		trigger='0'
	#	    elif(flow>trigflow):
	#		trigger='1'
                    #motor_2.ChangeDutyCycle(pump_pressure)
                    indiff= self.ABP_pressure()
                  #  print(indiff)
                    time_elapsed_inhale=0
                    indiff = 0
		    volume=self.Flow()
		    if(volume == -9999):
			SDP_flag = 1
		    Pmean=0
		    del Pmean_array[:]
		    clock_t2= time()
		    volume_flag = 0
		    volume_reach_time = 0
		    volume=0
		    volume_peak_inhale=0
		    sending_time = 0.2
		    send_last_time=0
		    VTi_volume=0
		    flow_flag = 0
		    volFlow_rate = self.rate()
		    pump_pressure_now = pump_pressure /2
		    peak_insp_pressure = 0
		    volume_comp = 0
		    pump_flag = 0
		    rise_flag = '0'
                    while(indiff <= PIP and  time_elapsed_inhale <= inhale_time):
                    #    motor_1.ChangeDutyCycle(pump_pressure)
                        #motor_2.ChangeDutyCycle(pump_pressure)
			GPIO.output(37, GPIO.HIGH)
                        t3 = time()
                        indiff= self.ABP_pressure()
                        time_elapsed_inhale= t3-current_time
   #                     volume_reach_time = time_elapsed_inhale
			Pmean_array.append(indiff)
			if(indiff >= P_plat * 0.8 and rise_flag == '0'):
				rise_time = time_elapsed_inhale
				rise_flag = '1'
			if(SDP_flag == 0):
			    volume=self.Flow()
			    volume_comp = volume
			    volume_comp = volume_comp - ((volume_comp*leak_percentage)/100)
			    volFlow_rate=self.rate()
			if(sending_time > 0.02):
			    packet_inhalation =('A@' + str(round(indiff,2)) + ','+str(round(volFlow_rate,2)) + ',' + str(int(volume)) + ',' + str(trigger) + '#')
			    try:
				ser.write(packet_inhalation)
			    except:
				print('BT error send Inhalation')
#			    print('packet size isssssssssss')
#			    print(len(packet_inhalation))
#			    print('packet conatins ')
#			    print(packet_inhalation)
			    send_last_time= time()
			sending_time=t3 - send_last_time
			if((indiff >= P_plat*motor_factor or time_elapsed_inhale >= 0.5) and flow_flag == 0): #(time_elapsed_inhale > 0.3 and flow_flag == 0) or indiff >= 10): #or indiff >= P_plat - 4): #or volFlow_rate >= flow_plat - 10):
			    flow_flag = 1
			    motor_1.ChangeDutyCycle(pump_pressure)
# 			    print('CMHO reacged')
# 			    print(indiff)
# 			if(P_plat <= 17 and (indiff >= P_plat/2 or time_elapsed_inhale >= 0.2) and flow_flag == 0): #(time_elapsed_inhale > 0.3 and flow_flag == 0) or indiff >= 10): #or indiff >= P_plat - 4): #or volFlow_rate >= flow_plat - 10):
# 			    flow_flag = 1
# 			    motor_1.ChangeDutyCycle(pump_pressure)
# 			if(flow_flag == 1 and volFlow_rate <= flow_plat):
# 			    motor_1.ChangeDutyCycle(pump_pressure)
			if(volFlow_rate >= flow_plat and flow_flag ==1):
			    flow_flag = 2
			if(flow_flag == 3):
			    if(volFlow_rate <= flow_plat and pump_pressure_now <= 98):
				pump_pressure_now = pump_pressure_now + 1
			        motor_1.ChangeDutyCycle(pump_pressure_now)
			    if(volFlow_rate >= flow_plat and pump_pressure_now >= 2): #and pump_pressure_now >( pump_pressure /2)):
			        pump_pressure_now = pump_pressure_now - 1
			        motor_1.ChangeDutyCycle(pump_pressure_now)
# 			if(pump_pressure_now <= pump_pressure and volFlow_rate <= flow_plat):
# 			    if(pump_flag == 0):
# 			        pump_pressure_now  = pump_pressure_now + 1
# 			    else:
# 				pump_pressure_now = pump_pressure_now + 0.5
		        if(volume_comp >= VTi_max and volume_flag == 0):
		    	    volume_reach_time = time_elapsed_inhale
		    	    GPIO.output(33, GPIO.HIGH)
		    	    volume_flag = 1
# 			    motor_1.ChangeDutyCycle(pump_pressure_now)
# 			elif(volFlow_rate > flow_plat and pump_pressure_now >( pump_pressure /2)):
# 			    pump_pressure_now = pump_pressure_now - 0.5
# 			    motor_1.ChangeDutyCycle(pump_pressure_now)
# 			    pump_flag = 1
#			print(time_elapsed_inhale)
#			print(inhale_time)
			inhale_array.append(indiff)
			VTi_volume = max(volume,VTi_volume)
			peak_insp_pressure = max(peak_insp_pressure, indiff)
			volume_peak_inhale=max(volume_peak_inhale,volFlow_rate)
		    if(IHold == '1'):
			motor_1.ChangeDutyCycle(int(pump_pressure*0.9))
			GPIO.output(33, GPIO.HIGH)
			print("entering inspiratory hold")
			print(IH_time)
			ser.write("ACK66")
			instant_time = time()
		        sending_time = 0.2
		        send_last_time=0
			while(time() - instant_time < IH_time):
			    print(sending_time)
			    indiff= self.ABP_pressure()
			    volFlow_rate=self.rate()
			    if(sending_time > 0.02):
			        packet_inhalation =('A@' + str(round(indiff,2)) + ','+str(round(volFlow_rate,2)) + ',' + str(int(volume)) + ',' + str(trigger) + '#')
			        try:
				    ser.write(packet_inhalation)
			        except:
				    print('BT error send Inhalation')
			        send_last_time= time()
			    sending_time=time() - send_last_time
		    IHold = '0'
		    GPIO.output(33, GPIO.LOW)
		    if(indiff >= (PIP+2)):
			pressure_count = pressure_count +1
			if(pressure_count >= 3):
		            try:
		  	        ser.write('ACK07')
			    except:
		      	        print('unable to send')
		    else:
			pressure_count = 0
			try:
			    ser.write('ACK57')
			except:
			    print('unable to send')
		    if(indiff <= (P_plat)*0.5):
			pressure_low_count = pressure_low_count +1
			if(pressure_low_count >= 3):
		            try:
		  	        ser.write('ACK62')
			    except:
		      	        print('unable to send')
		    else:
			pressure_low_count = 0
			try:
			    ser.write('ACK72')
			except:
			    print('unable to send')
		    GPIO.output(37, GPIO.LOW)
		    MVi_array.append(VTi_volume)
		    VTi_volume = VTi_volume - ((VTi_volume*leak_percentage)/200)
		    loop = 0
                    flag=1
		#    motor_1.ChangeDutyCycle(peep)
		    if(VTi_volume < (VTi_max*0.8) and indiff >= PIP and patient_status == 1):
		        ser.write("ACK61")
		    else:
			ser.write("ACK71")
		    if(VTi_volume < (VTi_max*0.8) and pump_pressure >= 95 and patient_status == 1):
			ser.write("ACK60")
		    else:
			ser.write("ACK70")
		    toggle_switch = str(toggle())
		    if(EHold == '1'):
			toggle_switch ='1'
		    sleep(0.01)
	#	    GPIO.output(35, GPIO.HIGH) # low level triggered relay
		    if(toggle_switch == "0"):
#			print("got in the first statement")
		        motor_1.ChangeDutyCycle(peep_open)
			sol.ChangeDutyCycle(0)
		    if(toggle_switch == "1"):
	#		GPIO.output(33, GPIO.HIGH)
			print("got in the 2nd statement")
		        motor_1.ChangeDutyCycle(peep)
			sol.ChangeDutyCycle(0)
		    inhale_array_length = len(inhale_array)
		    if(inhale_array_length % 2 != 0):
			inhale_array.append(0)
			inhale_array_length = inhale_array_length + 1
		    inhale_first_half = inhale_array[0:inhale_array_length/2]
		    inhale_second_half = inhale_array[inhale_array_length/2:]
		    first_max = max(inhale_first_half)
		    second_max = max(inhale_second_half)
		    if(first_max - P_plat > 0.5 and motor_factor > 0.2):
		        motor_factor = motor_factor - 0.01
			if(first_max - P_plat > 2):
			    motor_factor = motor_factor - 0.01
		    if(P_plat - first_max > 0.5 and motor_factor <= 1.0):
			motor_factor = motor_factor + 0.01
			if(P_plat - first_max > 2):
			    motor_factor = motor_factor + 0.01
		    if(VTi_volume < (VTi_max * 0.8)):
			volume_count = volume_count + 1
			if(volume_count >= 5):
			    buzzer(7,1)
			    try:
			        ser.write('ACK36')
			    except:
			        print('unable to send')
		    else:
			buzzer(7,0)
			volume_count = 0
			try:
			    ser.write('ACK46')
			except:
			    print('unable to send')
		    try:
		        Pmean= sum(Pmean_array)/len(Pmean_array)
		    except:
			print("zero exception error")
		    try:
		        if(compliance_flag == 0):
		            compliance_array.append(VTi_volume/(peak_insp_pressure - peep_val))
			    if(len(compliance_array) == 2):
			        compliance = (sum(compliance_array)/len(compliance_array)) * 0.75
			        pressure = (VTi_max/compliance) + peep_val
			        if(pressure > PIP):
				    pressure  = PIP - 2
	        	        P_plat_value=find_nearest(P_plat_array,pressure)
	        	        index= P_plat_array.index(P_plat_value)
				inst_pump_pressure = pump_pressure_array[index]
				if(inst_pump_pressure > pump_pressure):
	        	            pump_pressure = pump_pressure_array[index]  
			        compliance_flag = 1
		    except:
			print("error")
#		    print('Pmean is')
#		    print(Pmean)
#		    print('P_plat is')
#		    print(int(peak_insp_pressure))
#		    print(P_plat)
                    RR.append(RR_time)
#		    print('volume is')
		    indiff = int(indiff)
		    if((VTi_volume > VTi_max*1.1 or (volume_reach_time < inhale_time*0.8  and VTi_volume >= VTi_max))  and ABP_flag == 0 and peak_insp_pressure >= peep_val+5 and compliance_flag == 1  and SDP_flag == 0 and patient_status == 1 and pump_pressure >= 6 and pump_pressure <= 99):
			pump_pressure = pump_pressure - 1
			P_plat = indiff
#			print("--------------------------------")
		    elif((VTi_volume < VTi_max or volume_reach_time >= inhale_time*0.8) and compliance_flag == 1  and ABP_flag == 0 and SDP_flag == 0 and indiff < PIP and patient_status == 1 and pump_pressure >= 5 and pump_pressure <= 95):
			pump_pressure = pump_pressure + 1
# 			if((VTi_max - VTi_volume) > 100 and leak_percentage < 70 and VTi_max > 100 ):
# 			    if(((VTi_max - VTi_volume)/3 ) > 5):
# 				pump_pressure = pump_pressure + 5
# 			    else:
# 			        pump_pressure = pump_pressure + ((VTi_max - VTi_volume)/30)
# 			elif(VTi_max > 100):
# 			    pump_pressure = pump_pressure + 1
# 			if(VTi_max <= 100):
# 			    pump_pressure = pump_pressure + 5
			P_plat = indiff
		    if(pump_pressure > 95):
		        pump_pressure = 95
		    if(patient_status == 1 and peep_val_send < peep_val and peep <= peep_first + 10  and peep >= 4 and  peep < 90 and toggle_switch == '1'):
			peep = peep + 1
		    elif(patient_status == 1 and peep_val_send > peep_val and peep >= peep_first - 10  and peep > 5 and peep < 92 and toggle_switch == '1'):
			peep = peep - 1
#		    print('Vpeak Inspiratory')
#		    print(volume_peak_inhale)
	#	    MVi_array.append(VTi_volume)
#		    print('trigger is')
#		    print(trigger)
#		    print('Pmean is')
#		    print(Pmean)
		if(flag == 1):
		    patient_set = 0
		    if((leak_percentage > 90 or indiff <= 5) and patient_set == 0 ):
			buzzer(5,1)
			patient_set = 1
			patient_status = 0
		        print('disconnection')
#			trigger = '0'
			try:
			    ser.write('ACK05')
			except:
			    print('not able to send')
		       # GPIO.output(22, GPIO.HIGH)
		    if((leak_percentage <= 90 or indiff > 5) and patient_set == 0):
			buzzer(5,0)
			patient_set = 1
			patient_status = 1
#			trigger = '1'
			try:
			    ser.write('ACK06')
			except:
			    print('not able to send')
                    if(len(RR) >= 5):
                        BPM = ( RR[1] + RR[2] + RR[3] + RR[4]) /4
                        BPM = ceil(60/BPM)
			if(BPM > 70):
			    try:
				ser.write('ACK09')
			    except:
				print('unable to send')
			else:
			    try:
				ser.write('ACK59')
			    except:
				print('unable to send')
 #                       print('BPM is')
  #                      print(BPM)
                        RR.pop(0)
		    if(len(MVi_array) >= 5):
		        MVi= (MVi_array[1] + MVi_array[2] + MVi_array[3] + MVi_array[4])/4
			MVi = MVi * BPM
#		        print('MVi value is')
#		        print(MVi)
		        MVi_array.pop(0)
		        packet_end_inhalation = ('B@' + str(round(peak_insp_pressure,2)) + "," + str(int(VTi_volume)) + "," + str(round(volume_peak_inhale,2)) + "," + str(round(Pmean,2)) + "," + str(round((MVi/1000),2))  + "," + str('0') + "," + str(round(time_elapsed_inhale,2)) + "," + str(round(indiff,2)) + "," + str('0') + "," + str(round(rise_time,2)) + '#')
#		        print('packet length is')
#		        print(packet_end_inhalation)
		        try:
		            ser.write(packet_end_inhalation)
		        except:
			    print('BT Errror in end of inhalation loop')
                if(flag==1):
		    time_error_last = time()
		    time_elap = 0
		    q= time()
		    del exp_press_array[:]
		    del exp_flow_array[:]
		    inhale_loop = "0"
		    volume = 0
		    sending_time = 200
		    send_last_time = 0
		    volume_peak_exhale = 0
		    exhale_break = '0'
		    while(time_elap < 0.1 or (toggle_switch == "1" and time_elap < 0.3)):
			w=time()
			time_elap=w-q
#			exp_flow = self.rate()
 #                       exp_flow_array.append(exp_flow)
                        indiff = self.ABP_pressure()
			exp_press_array.append(indiff)
			peak_insp_pressure = int(peak_insp_pressure)
			if(peak_insp_pressure <= 0):
			    peak_insp_pressure = 1
			ratio = peep/P_plat
                        if(ratio > 0.8 and time_elap >= 0.2 and toggle_switch == '1' and exhale_break == '0'):
                            GPIO.output(35, GPIO.LOW)
                            sleep(0.05)
        #                    motor_1.ChangeDutyCycle(peep)
			    exhale_break = '1'
                        elif(ratio >= 0.15 and ratio <= 0.4 and time_elap >= 0.4 and P_plat < 28 and toggle_switch == '1' and exhale_break == '0'):
                            GPIO.output(35, GPIO.LOW)
                            sleep(0.05)
         #                   motor_1.ChangeDutyCycle(peep)
			    exhale_break = '1'
                        elif(ratio >= 0.15 and ratio <= 0.4 and time_elap >= 0.4 and P_plat >=28 and toggle_switch == '1' and exhale_break == '0'):
                            GPIO.output(35, GPIO.LOW)
                            sleep(0.05)
          #                  motor_1.ChangeDutyCycle(peep)
			    exhale_break = '1'
                        elif(ratio > 0.4 and ratio <= 0.8 and time_elap >= 0.3 and toggle_switch == '1' and exhale_break == '0'):
                            GPIO.output(35, GPIO.LOW)
                            sleep(0.05)
           #                 motor_1.ChangeDutyCycle(peep)
			    exhale_break = '1'
                        elif(ratio <0.15 and ratio >=0.00 and time_elap >= 0.4 and toggle_switch == '1' and exhale_break == '0'):
                            GPIO.output(35, GPIO.LOW)
                            sleep(0.05)
            #                motor_1.ChangeDutyCycle(peep)
			    exhale_break = '1'
			diff = self.SDP_pressure()
			if(SDP_flag == 0):
			    volFlow_rate = self.rate()
			    volume=self.Flow()
			if(diff == 700 or diff == -700 or SDP_flag == 1):
			    try:
				f = open("/home/pi/AgVa_5.0/mode.txt","w")
				f.write(str('11'))
				f.close()
				try:
				    ser.write('ACK30')
				except:
				    print('unable to send data')
			    except:
				print('unable to write mode')
			packet_exhalation= ('C@' + str(round(indiff,2)) + ',' + str(round(volFlow_rate,2)) +',' + str(int(volume)) + ',' + str(int((TITOT)*100)) + '#')
			if(sending_time > 0.02):
			    try:
				ser.write(packet_exhalation)
			    except:
				print('BT exhalation error')
			    send_last_time = time()
			sending_time = w- send_last_time
# 		        if(toggle_switch == "1" and indiff <= peep_val):
#                             sol.ChangeDutyCycle(100)
# 			    GPIO.output(33, GPIO.LOW)
# 			    holding_now = time()
# 			    holding_time = 0.0001
# 			    while( time() - holding_now <= holding_time):
# 				indiff = self.ABP_pressure()
# 			        volFlow_rate = self.rate()
# 			        volume=self.Flow()
# 				packet_exhalation= ('C@' + str(round(indiff,2)) + ',' + str(round(0,2)) +',' + str(int(volume)) + ',' + str(int((TITOT)*100)) + '#')
# 				if(1 == 1):
# 			    	    try:
# 					ser.write(packet_exhalation)
# 			    	    except:
# 					print('BT exhalation error')
# 			    	    send_last_time = time()
# 				sending_time = w- send_last_time
			#	sleep(0.02)
#			print('packet exhalation size  is')
#			print((packet_exhalation))
			volume_peak_exhale= min(volume_peak_exhale, volFlow_rate)
#			print('packet_exhal;ation conatins is')
#			print(packet_exhalation)
                    temp_peep = self.ABP_pressure()
                    new_time=time()
                    time_elapsed_exhale=0.0
# 		    if(toggle_switch == "1" and indiff <= peep_val-peep_factor):
#                         sol.ChangeDutyCycle(100)
# 			holding_now = time()
# 			holding_time = 0.001
# 			sending_time = 2.0
# 			send_last_time = 0
# 			while( time() - holding_now <= holding_time):
# 			    indiff = self.ABP_pressure()
# 			    volFlow_rate = self.rate()
# 			    volume=self.Flow()
# 			    packet_exhalation= ('C@' + str(round(indiff,2)) + ',' + str(round(0,2)) +',' + str(int(volume)) + ',' + str(int((TITOT)*100)) + '#')
# 			    if( 1== 1):
# 			        try:
# 				    ser.write(packet_exhalation)
# 			    	except:
# 				    print('BT exhalation error')
# 			    	send_last_time = time()
# 			    sending_time = time()- send_last_time
			#    sleep(0.02)
                    diff=self.SDP_pressure()
		    start_time=current_time
		    time_elapsed_exhale_flow = 0
		    sending_time=200
		    send_last_time = 0
		    sol_flag  = 0
                    while((time_elapsed_exhale<(cycle_time-inhale_time - 0.3) and (diff <= 0 or indiff > peep_val* 1.5 ) ) or (SDP_flag ==1 and time_elapsed_exhale<(cycle_time-inhale_time-0.5))):
                        time_elapsed_exhale=time()-new_time
#			exp_flow = self.rate()
 #                       exp_flow_array.append(exp_flow)
			indiff = self.ABP_pressure()
			exp_press_array.append(indiff)
		        if(toggle_switch == "1" and sol_flag == 0 and ((indiff <= peep_val+2 and volFlow_rate >= -10) or (indiff <= peep_val-2))):
                            sol.ChangeDutyCycle(100)
			    GPIO.output(33, GPIO.LOW)
			    sol_flag = 1
			    holding_now = time()
			    holding_time = 0.001
			    while( time() - holding_now <= holding_time):
				indiff = self.ABP_pressure()
			        volFlow_rate = self.rate()
			        volume=self.Flow()
				packet_exhalation= ('C@' + str(round(indiff,2)) + ',' + str(round(0,2)) +',' + str(int(volume)) + ',' + str(int((TITOT)*100)) + '#')
				if(1 ==1 ):
			    	    try:
					ser.write(packet_exhalation)
			    	    except:
					print('BT exhalation error')
			    	    send_last_time = time()
				sending_time = time()- send_last_time
		#		sleep(0.02)
			if(SDP_flag == 0):
           		    volume=self.Flow()
			    volFlow_rate=self.rate()
			    diff = self.SDP_pressure()
			volume_peak_exhale=min(volume_peak_exhale,volFlow_rate)
 			if(volFlow_rate < -5):
 			    time_elapsed_exhale_flow = time_elapsed_exhale
			packet_exhalation = ('C@' + str(round(indiff,2)) + ',' + str(round(volFlow_rate,2)) + ',' + str(int(volume)) + ',' + str(int((TITOT)*100))+'#')
			if(sending_time > 0.02):
			    try:
				ser.write(packet_exhalation)
			    except:
				print('BT Error sending in Exhalation conditional')
			    send_last_time = time()
			sending_time =time() - send_last_time 
                        start_time=current_time
                    #    print(start_time)
#		    flow_for_peep = 0.25 * (min(exp_flow_array))
                    if(int(peep_val_send) >=  peep_val - 1 and int(peep_val_send) <= peep_val+1):
                        lock = 1
                    elif(int(peep_val_send) > peep_val+1 or int(peep_val_send) < peep_val -1):
                        lock = 0
		    if(EHold == '1'):
			GPIO.output(33, GPIO.HIGH)
			print("Entering expiratory hold")
			instant_time = time()
			ser.write("ACK67")
		        sending_time = 0.2
		        send_last_time=0
			while(time() - instant_time < EH_time):
			    indiff= self.ABP_pressure()
			    print(time() - instant_time)
			    volFlow_rate=self.rate()
			    if(sending_time > 0.02):
			        packet_exhalation = ('C@' + str(round(indiff,2)) + ',' + str(round(volFlow_rate,2)) + ',' + str(int(volume)) + ',' + str(int((TITOT)*100))+'#')
			        try:
				    ser.write(packet_exhalation)
			        except:
				    print('BT error send Inhalation')
			        send_last_time= time()
			    sending_time=time() - send_last_time
			peep_val_send = indiff
			GPIO.output(33, GPIO.LOW)
		    EHold = '0'
 #                   print(exp_press_array)
  #                  value1 = find_nearest(exp_flow_array , flow_for_peep)
   #                 peep_array_index = exp_flow_array.index(value1)
#                    peep_val_send = min(temp_peep, exp_press_array[peep_array_index])
#		    peep_val_send = temp_peep
                    flag=0
		    exhale_time_last = time()
		    inhale_loop = "1"
		    loop = 1
		    time_elapsed_exhale = time_elapsed_exhale + 0.5
#		    if(len(exp_press_array) > 10):
#		        peep_val_send =  exp_press_array[len(exp_press_array) - 3]
		    TITOT=(time_elapsed_inhale/(time_elapsed_inhale + time_elapsed_exhale + TOT_last))
#		    print('peak volume flow rate exhale')
#		    print(volume_peak_exhale)
		    if(int(peep_val_send) <= peep_val and patient_status == 1 and toggle_switch == '1' and lock == 0):
			peep_factor = peep_factor - (peep_val - peep_val_send)*0.1
		    if(int(peep_val_send) > peep_val and patient_status == 1 and toggle_switch == '1' and lock == 0):
			peep_factor = peep_factor + (peep_val_send - peep_val)*0.1
		    if(int(peep_val_send) > peep_val+2):
			peep_count = peep_count + 1
			if(leak_percentage > 20 and trigger == '0'):
			    ser.write("ACK39")
			if(peep_count >=3 and peep_set == 0):
			    buzzer(4,1)
			    peep_set = 1
			    try:
			        ser.write('ACK08')
			    except:
			        print('unable to send')
		    else:
			if(peep_set == 1):
			    buzzer(4,0)
			    peep_set = 0
			peep_count = 0
			ser.write("ACK49")
			try:
			    ser.write('ACK58')
			except:
			    print('unable to send')
		    MVe_array.append(volume)
		    FiO2 = self.FiO2()
                    self.power()
#		    print('FiO2 is')
#		    print(FiO2)
		    if(toggle_switch == '1'):
		        time_elapsed_exhale = time_elapsed_exhale + 0.4
			time_elapsed_exhale_flow = time_elapsed_exhale_flow + 0.4
		    else:
			time_elapsed_exhale = time_elapsed_exhale + 0.1
			time_elapsed_exhale_flow = time_elapsed_exhale_flow + 0.1
		    if(len(MVe_array)>=5):
		        MVe= (MVe_array[1] + MVe_array[2] + MVe_array[3] + MVe_array[4])/4
			MVe = MVe * BPM
		        MVe_array.pop(0)
		        try:
			    leak_percentage = ((MVi - MVe)/MVi)*100
			    if(leak_percentage <= -1):
			        leak_percentage = 0
		        except:
			    leak_percentage = 0
#		        print('MVe is')
#		        print(MVe)
		   # packet_end_exhalation = 'ABC'
		        packet_end_exhalation = ('D@' + str(round(peep_val_send,2)) + ',' + str(int(BPM)) + ',' + str(FiO2) + ',' + str(round(volume_peak_exhale,2)*-1)  + "," + str(round((MVe/1000),2)) + "," + str(leak_percentage) + "," + str(0) + "," + str(volume) + "," + str(time_elapsed_exhale_flow) + '#')
#			print('string issssssssssssssssssssssssssssssssssssss')
#			print(packet_end_exhalation)
		        try:
	                    ser.write(packet_end_exhalation)
		        except:
			    print('BT send error in end of exhalation')
#		    try:
#		        with open('/home/pi/AgVa_5.0/data.csv', mode='a') as data:
 #  		            data = csv.writer(data, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
#		            now = datetime.datetime.now()
#		            data.writerow([str(now),str(self.measure_temp()),str(Pmean),str(volume),str(volume_peak_inhale),str(trigger),str(BPM),str(MVi),str(ratio), str(volume_peak_exhale),str(MVe)])
#		    except:
#			print('data file open error')
	except KeyboardInterrupt:
	    return


    #------------------------------------------------
    def PSV(self):
        global peep_first,volume_flag,lock,peep_factor,IH_time, EH_time, IHold, EHold,exp_press_array,exp_flow_array,peep,peep_open,time_elapsed_exhale_flow,patient_trigger_flow,rise_time, rise_flag,inhale_array,motor_factor,pressure_low_count,leak_comp_flag,running_avg, trigflow_comp, comp_flag,inhale_loop,toggle_switch,loop,patient_set,TITOT,exhale_time_last, TOT_last,patient_status,pressure_count,peep_count, ABP_flag, SDP_flag, pump_pressure_high, pump_pressure_low, pump_pressure,flow_plat,ratio_set,time_error_set,thread_mode_status,ABP_flag,time_error, time_error_last,P_plat_value,prev_mode,data,ratio,RR_time,indiff, RR, BPM, MVi_array, MVi,volume, volume_peak_inhale, trigger, max_value, volume_peak_exhale, MVe_array, MVe,clock_t2
        thread_mode_status = True
	peep_factor = 2.0
	volume_flag = 0
	lock = 0
	time_elapsed_exhale_flow = 0
	rise_time = 0
	rise_flag = '0'
	patient_trigger_flow =0
	pressure_count = 0
	del exp_press_array[:]
	del exp_flow_array[:]
	del MVi_array[:]
	del MVe_array[:]
	pressure_low_count =0
	running_avg = 0.0
	inhale_array = []
	motor_factor = 0.5
	comp_flag = '0'
	trigflow_comp = 0
	peep_count = 0
	exhale_time_last = time()
	inhale_loop = "0"
	TOT_last = time()
	ratio_set = 0
	VTi_volume = 0
	leak_percentage = 0
	GPIO.output(35, GPIO.LOW)
	loop = 1
	patient_set = 0
	flag = 0
	peep_set = 0
	time_error_set = 0
	flow_plat = 50
	sol.ChangeDutyCycle(100)
        start_time=time() - 5000
	time_error_last = time()
	sending_time = 0.2
	peep_val_send = 0
	temp_peep = 0
        try:
            while True:
		prev_mode = 13
		ABP_flag = 0
		print('we are in PSV mode')
		hold_check()
		data= self.read_data()
		print('mode is')
		print(data)
		if(data == 1):
		    break;
#		print(inhale_time)
#		print(BPM)
#		print(PIP)
#		print(VTi_max)
#		print(peep)
		indiff= self.ABP_pressure()
		exp_press_array.append(indiff)
#		print(pump_pressure)
	#	GPIO.output(22, GPIO.LOW)
		volFlow_rate = self.rate()
#		indiff= self.ABP_pressure()
		if(indiff == -8888):
			ABP_flag = 1
			indiff = 0
		packet_exhalation = ('C@' + str(round(indiff,2)) + ',' + str(round(volFlow_rate,2)) + ',' + str(int(volume)) + ',' + str(int((TITOT)*100))+'#')
		if(sending_time > 0.02):
		    try:
			ser.write(packet_exhalation)
		    except:
			print('BT Error sending in Exhalation conditional')
		    send_last_time = time()
		sending_time =time() - send_last_time 
#		print('pump_pressure is')
#		print(pump_pressure)
                flow=self.rate()
	        if(indiff <= peep_val +1 and comp_flag == '1' and leak_comp_flag == '1'):
		    trigflow_comp = trigflow + running_avg + 3
		    comp_flag = '0'
		    running_avg = 0
 #               flow=self.rate()
#		flow = flow - peep_hole[int(peep_val)]
                current_time=time()
		if(leak_comp_flag == '0'):
		    trigflow_comp = trigflow
         #       volume=0
		TOT_last = current_time - exhale_time_last
	#	peep_val_send = indiff
	        if(comp_flag == '0' and flow > 1):
		    running_avg = (running_avg + flow)/2
                if(indiff <= peep_val+2 and flow >= -10):
                    sol.ChangeDutyCycle(100)
		SDP_flag = 0
		flow_flag_unset = 0
		loop = 1
		time_error = time() - time_error_last
                if(flow >= trigflow  or (current_time-start_time)>cycle_time):
                    motor_1.ChangeDutyCycle(100)
		    if(P_plat <= 22):
			motor_1.ChangeDutyCycle(80)
		    sol.ChangeDutyCycle(100)
		    RR_time=current_time-start_time
		    pump_pressure_now = pump_pressure
		    GPIO.output(33, GPIO.LOW)
		    GPIO.output(35, GPIO.LOW)
		    patient_trigger_flow = flow
		    leak_comp_flag = leak_comp()
		    inhale_loop = "1"
		    del inhale_array[:]
		    volume = 0
		    comp_flag = '1'
# 		    sol.ChangeDutyCycle(100)
		    flow_flag = 0
 		    peep_val_send = temp_peep
 		    if(len(exp_press_array) > 10):
 			peep_val_send = exp_press_array[len(exp_press_array) - 4]
 		    del exp_press_array[:]
#                    motor_1.ChangeDutyCycle(pump_pressure)
#                     motor_1.ChangeDutyCycle(100)
# 		    if(P_plat <= 22):
# 			motor_1.ChangeDutyCycle(80)
#		    if(P_plat <= 17):
#			motor_1.ChangeDutyCycle(65)
# 			pump_pressure_now = pump_pressure /2
                    #RR_time=current_time-start_time
		    if(time_error < 0.7):
			time_error_set = 1
#			try:
#			    ser.write('ACK24')
#			except:
#			    print('unable to send data')
		    if(time_error > 0.7):
			time_error_set = 0
#			try:
#			    ser.write('ACK24')
#			except:
#			    print('unable to send data')
		    if(float(ratio) <= 0.25):
			ratio_set = 1
#			try:
#			    ser.write('ACK12')
#			except:
#			    print('unable to send data')
		    if(float(ratio) >= 0.75):
			ratio_set = 1
#			try:
#			    ser.write('ACK13')
#			except:
#			    print('unable to send data')
		    if(float(ratio) >0.25 and float(ratio) < 0.75):
			ratio_set = 0
#			try:
#			    ser.write('ACK22')
#			except:
#			    print('unable to send data')
		    if(RR_time > cycle_time):
			trigger='0'
		    else:
			trigger='1'
                 #   indiff= self.ABP_pressure()
                  #  print(indiff)
                    time_elapsed_inhale=0
		    now = time()
		    GPIO.output(37, GPIO.HIGH)
		    time_elapsed = 0
		    volume_peak_inhale=0
		    flow_flag = 0
		    volFlow_rate = self.rate()
		    pump_flag = 0
		    rise_flag = '0'
#		    pump_pressure_now = pump_pressure /2
#		    while(time_elapsed < 0.3):
#			time_elapsed = time() - now
#			indiff = self.ABP_pressure()
#			if(indiff >= (P_plat*1.15)):
#			    ser.write('ACK07')
#			if(SDP_flag == 0):
#			    volFlow_rate = self.rate()
#			    volume_peak_inhale=max(volume_peak_inhale,volFlow_rate)
#			if(indiff >= P_plat/2 and flow_flag == 0 and time_elapsed_inhale >= 0.3): #(time_elapsed_inhale > 0.3 and flow_flag == 0) or indiff >= 10): #or indiff >= P_plat - 4): #or volFlow_rate >= flow_plat - 10):
#			    flow_flag = 1
#			    print('CMHO reacged')
#			    print(indiff)
#			if(flow_flag == 1 and volFlow_rate <= flow_plat):
#			    motor_1.ChangeDutyCycle(pump_pressure)
#			if(volFlow_rate >= flow_plat and flow_flag ==1):
#			    flow_flag = 2
#			if(flow_flag == 2):
#			    if(volFlow_rate <= flow_plat and pump_pressure_now <= 98):
#				pump_pressure_now = pump_pressure_now + 1
#			        motor_1.ChangeDutyCycle(pump_pressure_now)
#			    if(volFlow_rate >= flow_plat and pump_pressure_now >= 2): #and pump_pressure_now >( pump_pressure /2)):
#			        pump_pressure_now = pump_pressure_now - 1
#			        motor_1.ChangeDutyCycle(pump_pressure_now)
#			packet_inhalation =('A@' + str(int(indiff)) + ','+str(int(volFlow_rate)) + ',' + str(int(volume)) + ',' + str(trigger) + '#')
#			if(sending_time >0.7):
#			    try:
#			        ser.write(packet_inhalation)
#				sending_time = 0.4
#			    except:
#			        print('BT error send Inhalation')
		    time_elapsed  = 0
		    Pmean = 0
		    del Pmean_array[:]
		    Pmean_array.append(1)
		    volFlow_rate = self.rate()
		    clock_t2= time()
#		    print('now it is')
#		    print(clock_t2)
		    volume=0
		    sending_time = 0.2
		    send_last_time=0
		    VTi_volume=0
		    peak_insp_pressure = 0
		    flow_flag_100 = 0
		    pump_pressure_now = pump_pressure /2
		    volFlow_rate_previous = volFlow_rate
		    peak_flow = volFlow_rate
                    while((peak_flow*0.40 <= volFlow_rate and indiff <= PIP and time_elapsed_inhale <= inhale_time) or time_elapsed_inhale <= 0.5):
                    #    motor_1.ChangeDutyCycle(pump_pressure)
                        #motor_2.ChangeDutyCycle(pump_pressure)
                        t3 = time()
			GPIO.output(37, GPIO.HIGH)
			if(ABP_flag == 0):
                            indiff= self.ABP_pressure()
                        time_elapsed_inhale= t3-current_time
                   #     print(indiff)
			if(indiff >= P_plat * 0.8 and rise_flag == '0'):
				rise_time = time_elapsed_inhale
				rise_flag = '1'
			Pmean_array.append(indiff)
			if(SDP_flag == 0):
			    volume=self.Flow()
			    volFlow_rate=self.rate()
			    if(volFlow_rate >= peak_flow):
				peak_flow = volFlow_rate
			if(volume == -9999):
			    SDP_flag = 1;
			if(sending_time > 0.02):
			    packet_inhalation =('A@' + str(round(indiff,2)) + ','+str(round(volFlow_rate,2)) + ',' + str(int(volume)) + ',' + str(trigger) + '#')
			    try:
				ser.write(packet_inhalation)
			    except:
				print('BT error send Inhalation')
#			    print('packet size isssssssssss')
#			    print(len(packet_inhalation))
#			    print('packet conatins ')
			    print(packet_inhalation)
			    send_last_time= time()
			sending_time=t3 - send_last_time
 			if((indiff >= P_plat*motor_factor or time_elapsed_inhale >= 0.5) and flow_flag == 0): #(time_elapsed_inhale > 0.3 and flow_flag == 0) or indiff >= 10): #or indiff >= P_plat - 4): #or volFlow_rate >= flow_plat - 10):
			    flow_flag = 1
			    motor_1.ChangeDutyCycle(pump_pressure)
			    print('CMHO reacged')
			    print(indiff)
# 			if(P_plat <= 17 and (indiff >= P_plat/2 or time_elapsed_inhale >= 0.2) and flow_flag == 0): #(time_elapsed_inhale > 0.3 and flow_flag == 0) or indiff >= 10): #or indiff >= P_plat - 4): #or volFlow_rate >= flow_plat - 10):
# 			    flow_flag = 1
# 			    motor_1.ChangeDutyCycle(pump_pressure)
# 			if(flow_flag == 1 and volFlow_rate <= flow_plat):
# 			    motor_1.ChangeDutyCycle(pump_pressure)
			if(volFlow_rate >= flow_plat and flow_flag ==1):
			    flow_flag = 2
			if(flow_flag == 3):
			    if(volFlow_rate <= flow_plat and pump_pressure_now <= 98):
				pump_pressure_now = pump_pressure_now + 1
			        motor_1.ChangeDutyCycle(pump_pressure_now)
			    if(volFlow_rate >= flow_plat and pump_pressure_now >=  P_plat/2): #and pump_pressure_now >( pump_pressure /2)):
			        pump_pressure_now = pump_pressure_now - 1
			        motor_1.ChangeDutyCycle(pump_pressure_now)
# 			if(pump_pressure_now <= pump_pressure and volFlow_rate <= flow_plat):
# 			    if(pump_flag == 0):
# 			        pump_pressure_now  = pump_pressure_now + 1
# 			    else:
# 				pump_pressure_now = pump_pressure_now + 0.5
# 	#		    print('the pump_pressure now is')
# 	#		    print(pump_pressure_now)
# 	#		    print('pump pressure is')
# 	#		    print(pump_pressure)
# 			    motor_1.ChangeDutyCycle(pump_pressure_now)
# 			elif(volFlow_rate > flow_plat and pump_pressure_now >( pump_pressure /2)):
# 			    pump_pressure_now = pump_pressure_now - 0.5
# 			    motor_1.ChangeDutyCycle(pump_pressure_now)
# 			    pump_flag = 1
#			print(time_elapsed_inhale)
#			print(inhale_time)
                        #    GPIO.output(22, GPIO.LOW)
			inhale_array.append(indiff)
			VTi_volume = max(volume,VTi_volume)
			peak_insp_pressure = max(peak_insp_pressure, indiff)
			volume_peak_inhale=max(volume_peak_inhale,volFlow_rate)
		    if(IHold == '1'):
			motor_1.ChangeDutyCycle(int(pump_pressure*0.9))
			GPIO.output(33, GPIO.HIGH)
			print("entering inspiratory hold")
			print(IH_time)
			instant_time = time()
			ser.write("ACK66")
		        sending_time = 0.2
		        send_last_time=0
			while(time() - instant_time < IH_time):
			    print(sending_time)
			    indiff= self.ABP_pressure()
			    volFlow_rate=self.rate()
			    if(sending_time > 0.02):
			        packet_inhalation =('A@' + str(round(indiff,2)) + ','+str(round(volFlow_rate,2)) + ',' + str(int(volume)) + ',' + str(trigger) + '#')
			        try:
				    ser.write(packet_inhalation)
			        except:
				    print('BT error send Inhalation')
			        send_last_time= time()
			    sending_time=time() - send_last_time
		    IHold = '0'
		    GPIO.output(33, GPIO.LOW)
		    if(indiff >= (P_plat+3)):
			pressure_count = pressure_count +1
			if(pressure_count >= 3):
		            try:
		  	        ser.write('ACK07')
			    except:
		      	        print('unable to send')
		    else:
			pressure_count = 0
			try:
			    ser.write('ACK57')
			except:
			    print('unable to send')
		    if(indiff <= (P_plat)*0.5):
			pressure_low_count = pressure_low_count +1
			if(pressure_low_count >= 3):
		            try:
		  	        ser.write('ACK62')
			    except:
		      	        print('unable to send')
		    else:
			pressure_low_count = 0
			try:
			    ser.write('ACK72')
			except:
			    print('unable to send')
                    flag=1
		    loop = 0
		#    motor_1.ChangeDutyCycle(peep)
		    toggle_switch = str(toggle())
		    if(EHold == '1'):
			toggle_switch ='1'
		    sleep(0.01)
		#    GPIO.output(35, GPIO.HIGH) # low level triggered relay
		    try:
		        Pmean= sum(Pmean_array)/len(Pmean_array)
		    except:
			print("zero exception error")
		    if(toggle_switch == "0"):
			print("got in the first statement")
		        motor_1.ChangeDutyCycle(peep_open)
			sol.ChangeDutyCycle(0)
		    if(toggle_switch == "1"):
			print("got in the 2nd statement")
	#		GPIO.output(33, GPIO.HIGH)
		        motor_1.ChangeDutyCycle(peep)
			sol.ChangeDutyCycle(0)
		    inhale_array_length = len(inhale_array)
		    if(inhale_array_length % 2 != 0):
			inhale_array.append(0)
			inhale_array_length = inhale_array_length + 1
		    inhale_first_half = inhale_array[0:inhale_array_length/2]
		    inhale_second_half = inhale_array[inhale_array_length/2:]
		    first_max = max(inhale_first_half)
		    second_max = max(inhale_second_half)
		    if(first_max - P_plat > 0.5 and motor_factor > 0.2):
		        motor_factor = motor_factor - 0.01
			if(first_max - P_plat > 2):
			    motor_factor = motor_factor - 0.01
		    if(P_plat - first_max > 0.5 and motor_factor <= 1.0):
			motor_factor = motor_factor + 0.01
			if(P_plat - first_max > 2):
			    motor_factor = motor_factor + 0.01
#		    print('Pmean is')
#		    print(Pmean)
		    print('P_plat is')
		    print(int(peak_insp_pressure))
		    print(P_plat)
                    RR.append(RR_time)
		    indiff = int(indiff)
		    if(indiff > P_plat and patient_status == 1 and ABP_flag == 0 and indiff < PIP and pump_pressure >= 6 and pump_pressure <= 97):
			pump_pressure = pump_pressure - 1
			print('-1')
		    elif(indiff < P_plat and patient_status == 1 and ABP_flag == 0 and indiff < PIP and  pump_pressure >= 5 and pump_pressure <= 95):
			pump_pressure = pump_pressure + 1
			print('+1')
		    if(patient_status == 1 and peep_val_send < peep_val and peep <= peep_first + 10  and peep >= 4 and  peep < 90 and toggle_switch == '1'):
			peep = peep + 1
		    elif(patient_status == 1 and peep_val_send > peep_val and peep >= peep_first - 10  and peep > 5 and peep < 92 and toggle_switch == '1'):
			peep = peep - 1
#		    print('volume is')
#		    print(volume*0.4)
#		    print('Vpeak Inspiratory')
#		    print(volume_peak_inhale)
		    MVi_array.append(VTi_volume)
#		    print('trigger is')
#		    print(trigger)
#		    print('Pmean is')
#		    print(Pmean)
		    GPIO.output(37, GPIO.LOW)
		if(flag == 1):
		    patient_set = 0
		    if((leak_percentage > 90 or indiff <= 5) and patient_set == 0 ):
			buzzer(5,1)
			patient_set = 1
			patient_status = 0
		        print('disconnection')
#			trigger = '0'
			try:
			    ser.write('ACK05')
			except:
			    print('not able to send')
		       # GPIO.output(22, GPIO.HIGH)
		    if((leak_percentage <= 90 or indiff > 5) and patient_set == 0):
			patient_set = 1
			patient_status = 1
			buzzer(5,0)
			try:
			    ser.write('ACK06')
			except:
			    print('not able to send')
                    if(len(RR) >= 5):
                        BPM = ( RR[1] + RR[2] + RR[3] + RR[4]) /4
                        BPM = ceil(60/BPM)
			if(BPM > 70):
			    try:
				ser.write('ACK09')
			    except:
				print('unable to send')
			else:
			    try:
				ser.write('ACK59')
			    except:
				print('unable to send')
 #                      print('BPM is')
  #                     print(BPM)
                        RR.pop(0)
		    if(len(MVi_array) >= 5):
		        MVi= (MVi_array[1] + MVi_array[2] + MVi_array[3] + MVi_array[4])/4
			MVi = MVi * BPM
#		        print('MVi value is')
#		        print(MVi)
		        MVi_array.pop(0)
		        packet_end_inhalation = ('B@' + str(round(peak_insp_pressure,2)) + "," + str(int(VTi_volume - (VTi_volume*leak_percentage)/200)) + "," + str(round(volume_peak_inhale,2)) + "," + str(round(Pmean,2)) + "," + str(round((MVi/1000),2))  + "," + str(round(patient_trigger_flow,2)) + "," + str(round(time_elapsed_inhale,2)) + "," + str(round(indiff,2)) + "," + str('0') + "," + str(round(rise_time,2)) + '#')
#		        print('packet length is')
		        print(packet_end_inhalation)
		        try:
		            ser.write(packet_end_inhalation)
		        except:
			    print('BT Errror in end of inhalation loop')
                if(flag==1):
		    time_error_last = time()
		    time_elap = 0
		    inhale_loop = "0"
		    q= time()
		    del exp_flow_array[:]
		    del exp_press_array[:]
		    volume = 0
		    buzzer(7,0)
		    sending_time = 200
		    send_last_time = 0
		    volume_peak_exhale = 0
		    exhale_break = '0'
		    sol_flag = 0
		    while(time_elap < 0.1 or (toggle_switch == "1" and time_elap < 0.3)):
			w=time()
			time_elap=w-q
#			exp_flow = self.rate()
 #                       exp_flow_array.append(exp_flow)
			peak_insp_pressure = int(peak_insp_pressure)
#			print()
			if(peak_insp_pressure <= 0):
			    peak_insp_pressure = 1
			ratio = 1.0
                        indiff = self.ABP_pressure()
			exp_press_array.append(indiff)
                        if(ratio > 0.8 and time_elap >= 0.2 and toggle_switch == '1' and exhale_break == '0'):
                            GPIO.output(35, GPIO.LOW)
                            sleep(0.05)
         #                   motor_1.ChangeDutyCycle(peep)
			    exhale_break = '1'
                        elif(ratio >= 0.15 and ratio <= 0.4 and time_elap >= 0.4 and P_plat < 28 and toggle_switch == '1' and exhale_break == '0'):
                            GPIO.output(35, GPIO.LOW)
                            sleep(0.05)
          #                  motor_1.ChangeDutyCycle(peep)
			    exhale_break = '1'
                        elif(ratio >= 0.15 and ratio <= 0.4 and time_elap >= 0.4 and P_plat >=28 and toggle_switch == '1' and exhale_break == '0'):
                            GPIO.output(35, GPIO.LOW)
                            sleep(0.05)
           #                 motor_1.ChangeDutyCycle(peep)
			    exhale_break = '1'
                        elif(ratio > 0.4 and ratio <= 0.8 and time_elap >= 0.3 and toggle_switch == '1' and exhale_break == '0'):
                            GPIO.output(35, GPIO.LOW)
                            sleep(0.05)
            #                motor_1.ChangeDutyCycle(peep)
			    exhale_break = '1'
                        elif(ratio <0.15 and ratio >=0.00 and time_elap >= 0.4 and toggle_switch == '1' and exhale_break == '0'):
                            GPIO.output(35, GPIO.LOW)
                            sleep(0.05)
             #               motor_1.ChangeDutyCycle(peep)
			    exhale_break = '1'
			diff = self.SDP_pressure()
			if(ABP_flag == 0):
			    indiff = self.ABP_pressure()
			if(SDP_flag == 0):
			    volFlow_rate = self.rate()
			    volume=self.Flow()
			if(diff == 700 or diff == -700 or SDP_flag == 1):
			    try:
				f = open("/home/pi/AgVa_5.0/mode.txt","w")
				f.write(str('11'))
				f.close()
				try:
				    ser.write('ACK30')
				except:
				    print('unable to send data')
			    except:
				print('unable to write mode')
			packet_exhalation= ('C@' + str(int(indiff)) + ',' + str(int(volFlow_rate)) +',' + str(int(volume)) + ',' + str(int((TITOT)*100)) + '#')
			if(sending_time > 0.02):
			    try:
				ser.write(packet_exhalation)
			    except:
				print('BT exhalation error')
			    send_last_time = time()
			sending_time = w- send_last_time
# 		        if(toggle_switch == "1" and indiff <= peep_val and sol_flag == 0):
#                             sol.ChangeDutyCycle(100)
# 			    holding_now = time()
# 			    holding_time = 0.3
# 			    peep_check = peep_val - trigflow - 1 
# 			    if(peep_check < 0):
# 			        peep_check = 0
# 			    while( time() - holding_now <= holding_time and time_elap <0.3):
# 				time_elap = time() - q
# 				GPIO.output(33, GPIO.LOW)
# 				indiff = self.ABP_pressure()
# 				if(indiff <= peep_check):
# 				    break;
# 			        volFlow_rate = self.rate()
# 			        volume=self.Flow()
# 				packet_exhalation= ('C@' + str(round(indiff,2)) + ',' + str(round(0,2)) +',' + str(int(volume)) + ',' + str(int((TITOT)*100)) + '#')
# 				if( 1 == 1):
# 			    	    try:
# 					ser.write(packet_exhalation)
# 			    	    except:
# 					print('BT exhalation error')
# 			    	    send_last_time = time()
# 				sending_time = w- send_last_time
# 				sleep(0.02)
# 				sol_flag = 1
#			print('packet exhalation size  is')
#			print((packet_exhalation))
			volume_peak_exhale= min(volume_peak_exhale, volFlow_rate)
			print('packet_exhal;ation conatins is')
			print(packet_exhalation)
                    temp_peep = self.ABP_pressure()
#
                    new_time=time()
                    time_elapsed_exhale=0.0
# 		    if(toggle_switch == "1" and indiff <= peep_val-peep_factor and sol_flag == 0):
#                         sol.ChangeDutyCycle(100)
# 			holding_now = time()
# 			holding_time = 0.3
# 			sending_time = 2.0
# 			send_last_time = 0
# 			while( time() - holding_now <= holding_time):
# 			    indiff = self.ABP_pressure()
# 			    volFlow_rate = self.rate()
# 			    volume=self.Flow()
# 			    packet_exhalation= ('C@' + str(round(indiff,2)) + ',' + str(round(0,2)) +',' + str(int(volume)) + ',' + str(int((TITOT)*100)) + '#')
# 			    if(1 == 1):
# 			        try:
# 				    ser.write(packet_exhalation)
# 			    	except:
# 				    print('BT exhalation error')
# 			    	send_last_time = time()
# 			    sending_time = time()- send_last_time
# 			    sleep(0.02)
# 			sol_flag = 1
                    diff=self.SDP_pressure()
		    start_time=current_time
		    sending_time=200
		    time_elapsed_exhale_flow = 0
	#	    sol_flag  = 0
		    send_last_time = 0
                    while((time_elapsed_exhale<(cycle_time-inhale_time-0.3) and diff <= 0) or (SDP_flag ==1 and time_elapsed_exhale<(cycle_time-inhale_time-0.5))):
                        time_elapsed_exhale=time()-new_time
			exp_flow = self.rate()
                        exp_flow_array.append(exp_flow)
			if(ABP_flag == 0):
			    indiff = self.ABP_pressure()
			exp_press_array.append(indiff)
			if(SDP_flag == 0):
           		    volume=self.Flow()
			    volFlow_rate=self.rate()
			    diff = self.SDP_pressure()
		        if((toggle_switch == "1" and sol_flag  == 0 and ((indiff <= peep_val+2 and volFlow_rate >= -10) or (indiff <= peep_val-2))) or (time_elapsed_exhale>(cycle_time-inhale_time-0.3))  ):
			    peep_check = peep_val - trigflow - 1
			    if(peep_check < 0):
			        peep_check = 0
			    if(volFlow_rate >= trigflow):
			        break;
                            sol.ChangeDutyCycle(100)
			    sol_flag  = 1
			    holding_now = time()
			    holding_time = 0.3
# 			    peep_check = trigflow - peep_val - 2.0
# 			    if(peep_check < 0):
# 			        peep_check = 0
			    while( time() - holding_now <= holding_time and (time_elapsed_exhale<(cycle_time-inhale_time-0.3)) ):
				indiff = self.ABP_pressure()
				GPIO.output(33, GPIO.LOW)
				time_elapsed_exhale = time() - new_time
				if(volFlow_rate >= trigflow+peep_val):
				    break;
			        volFlow_rate = self.rate()
			        volume=self.Flow()
				packet_exhalation= ('C@' + str(round(indiff,2)) + ',' + str(round(0,2)) +',' + str(int(volume)) + ',' + str(int((TITOT)*100)) + '#')
				if(1 == 1):
			    	    try:
					ser.write(packet_exhalation)
			    	    except:
					print('BT exhalation error')
			    	    send_last_time = time()
				sending_time = time()- send_last_time
				sleep(0.02)
				sol_flag = 1
 			if(volFlow_rate < -5):
 			    time_elapsed_exhale_flow = time_elapsed_exhale
			volume_peak_exhale=min(volume_peak_exhale,volFlow_rate)
			packet_exhalation = ('C@' + str(round(indiff,2)) + ',' + str(round(volFlow_rate,2)) + ',' + str(int(volume)) + ',' + str(int((TITOT)*100))+'#')
			if(sending_time > 0.02):
			    try:
				ser.write(packet_exhalation)
			    except:
				print('BT Error sending in Exhalation conditional')
			    send_last_time = time()
			sending_time =time() - send_last_time 
                        start_time=current_time
                    #    print(start_time)
                    if(int(peep_val_send) >=  peep_val - 1 and int(peep_val_send) <= peep_val+1):
                        lock = 1
                    elif(int(peep_val_send) > peep_val+1 or int(peep_val_send) < peep_val -1):
                        lock = 0
		    if(EHold == '1'):
			GPIO.output(33, GPIO.HIGH)
			print("Entering expiratory hold")
			instant_time = time()
			ser.write("ACK67")
		        sending_time = 0.2
		        send_last_time=0
			while(time() - instant_time < EH_time):
			    indiff= self.ABP_pressure()
			    print(time() - instant_time)
			    volFlow_rate=self.rate()
			    if(sending_time > 0.02):
			        packet_exhalation = ('C@' + str(round(indiff,2)) + ',' + str(round(volFlow_rate,2)) + ',' + str(int(volume)) + ',' + str(int((TITOT)*100))+'#')
			        try:
				    ser.write(packet_exhalation)
			        except:
				    print('BT error send Inhalation')
			        send_last_time= time()
			    sending_time=time() - send_last_time
			peep_val_send = indiff
			GPIO.output(33, GPIO.LOW)
		    EHold = '0'
                    flag=0
#		    flow_for_peep = 0.25 * (min(exp_flow_array))
#                    print(flow_for_peep)
 #                   value1 = find_nearest(exp_flow_array , flow_for_peep)
  #                  peep_array_index = exp_flow_array.index(value1)
#                    peep_val_send = exp_press_array[peep_array_index]
		    inhale_loop = "1"
		    exhale_time_last = time()
		    loop = 1
		    time_elapsed_exhale = time_elapsed_exhale + 0.5
#		    peep_val_send = temp_peep
#		    print(time_elapsed_inhale /( time_elapsed_inhale + time_elapsed_exhale))
		    TITOT=(time_elapsed_inhale/(time_elapsed_inhale + time_elapsed_exhale + TOT_last))
#		    if(len(exp_press_array) > 10):
#		        peep_val_send =  exp_press_array[len(exp_press_array) - 3]
		    if(int(peep_val_send) <= peep_val and patient_status == 1 and toggle_switch == '1' and lock == 0):
			peep_factor = peep_factor - (peep_val - peep_val_send)*0.1
		    if(int(peep_val_send) > peep_val and patient_status == 1 and toggle_switch == '1' and lock == 0):
			peep_factor = peep_factor + (peep_val_send - peep_val)*0.1
		    if(int(peep_val_send) > peep_val+2):
			peep_count = peep_count + 1
			if(leak_percentage > 20 and trigger == '0'):
			    ser.write("ACK39")
			if(peep_count >=3 and peep_set == 0):
			    buzzer(4,1)
			    peep_set = 1
			    try:
			        ser.write('ACK08')
			    except:
			        print('unable to send')
		    else:
			if(peep_set == 1):
			    buzzer(4,0)
			    peep_set = 0
			peep_count = 0
			ser.write("ACK49")
			try:
			    ser.write('ACK58')
			except:
			    print('unable to send')
		    MVe_array.append(volume)
		    FiO2 = self.FiO2()
                    self.power()
		    print('FiO2 is')
		    print(FiO2)
		    if(toggle_switch == '1'):
		        time_elapsed_exhale = time_elapsed_exhale + 0.4
			time_elapsed_exhale_flow = time_elapsed_exhale_flow + 0.4
		    else:
			time_elapsed_exhale = time_elapsed_exhale + 0.1
			time_elapsed_exhale_flow = time_elapsed_exhale_flow + 0.1
		    if(len(MVe_array)>=5):
		        MVe= (MVe_array[1] + MVe_array[2] + MVe_array[3] + MVe_array[4])/4
			MVe = MVe * BPM
		        MVe_array.pop(0)
		        try:
			    leak_percentage = ((MVi - MVe)/MVi)*100
			    if(leak_percentage <= -1):
			        leak_percentage = 0
		        except:
			    leak_percentage = 0
#		        print('MVe is')
#		        print(MVe)
		   # packet_end_exhalation = 'ABC'
		        packet_end_exhalation = ('D@' + str(round(peep_val_send,2)) + ',' + str(int(BPM)) + ',' + str(FiO2) + ',' + str(round(volume_peak_exhale,2)*-1)  + "," + str(round((MVe/1000),2)) + "," + str(leak_percentage) + "," + str(trigflow_comp - trigflow) + "," + str(volume)  + "," + str(time_elapsed_exhale_flow) + '#')
#			print('string issssssssssssssssssssssssssssssssssssss')
			print(packet_end_exhalation)
		        try:
	                    ser.write(packet_end_exhalation)
		        except:
			    print('BT send error in end of exhalation')
	#	    try:
	#	        with open('/home/pi/AgVa_5.0/data.csv', mode='a') as data:
   	#	            data = csv.writer(data, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
	#	            now = datetime.datetime.now()
	#	            data.writerow([str(now),str(self.measure_temp()),str(Pmean),str(volume),str(volume_peak_inhale),str(trigger),str(BPM),str(MVi),str(ratio), str(volume_peak_exhale),str(MVe)])
	#	    except:
	#		print('data file open error')
        except KeyboardInterrupt:
            return


#-----------------------------------------------------------
    def ACV(self):
        global peep_first,compliance_flag,volume_flag,lock,peep_factor,IH_time, EH_time, IHold, EHold,exp_press_array,exp_flow_array,peep,peep_open,time_elapsed_exhale_flow,patient_trigger_flow,rise_time, rise_flag,inhale_array,motor_factor,pressure_low_count,leak_comp_flag,running_avg,trigflow_comp, comp_flag,volume_comp,inhale_loop,pump_pressure_array,P_plat_array,compliance,compliance_array,toggle_switch,peep_open,loop,peep,peep_pwm,P_plat,P_plat_high,exhale_time_last, TOT_last,TITOT,P_plat_low,ABP_flag,pressure_count,peep_count,volume_count, SDP_flag,patient_set,flow_plat,ratio_set,patient_status,pump_pressure,pump_pressure_low, pump_pressure_high,time_error_set,thread_mode_status,time_error, time_error_last,prev_mode,P_plat_value,peep_hole, peep_val,data,ratio,RR_time,indiff, RR, BPM, MVi_array, MVi,volume, volume_peak_inhale, trigger, max_value, volume_peak_exhale, MVe_array, MVe,clock_t2
        thread_mode_status = True
	peep_factor = 2.0
	lock = 0
	volume_flag = 0
	time_elapsed_exhale_flow = 0
	pressure_low_count = 0
	patient_trigger_flow = 0
	rise_time = 0
	rise_flag = '0'
	trigflow_comp = 0
	del MVi_array[:]
	del MVe_array[:]
	del exp_press_array[:]
	del exp_flow_array[:]
	running_avg = 0.0
	inhale_array = []
	motor_factor = 0.5
	comp_flag = '0'
	volume_count = 0
	volume_comp = 0
	exhale_time_last = time()
	TOT_last = time()
	peep_count = 0
	ratio_set = 0
	loop = 1
	del compliance_array[:]
	inhale_loop = "0"
	pressure_count = 0
	GPIO.output(35, GPIO.LOW)
	VTi_volume = 0
	leak_percentage = 0
	volume_set = 0
	flag = 0
	compliance_flag = 0
	patient_set = 0
	peep_set = 0
	sol.ChangeDutyCycle(100)
	peep_val_send = 0
	temp_peep = 0
	time_error_set = 0
        start_time=time() - 5000
	time_error_last = time()
	sending_time = 0.2
        try:
            while True:
		prev_mode = 25
		print('we are in ACV mode')
		data= self.read_data()
		print(data)
		hold_check()
		if(data == 1):
		    print("breaking the cuircuit in hereeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee")
		    break;
#		print(inhale_time)
#		print(BPM)
#		print(PIP)
#		print(VTi_max)
#		print(peep)
#		print(P_plat)
#		print(trigflow)
		indiff= self.ABP_pressure()
		exp_press_array.append(indiff)
		clock_t2 = time()
		volFlow_rate = self.rate()
	#	indiff= self.ABP_pressure()
		packet_exhalation = ('C@' + str(round(indiff,2)) + ',' + str(round(volFlow_rate,2)) + ',' + str(int(volume)) + ',' + str(int((TITOT)*100))+'#')
		if(sending_time > 0.02):
		    try:
			ser.write(packet_exhalation)
		    except:
			print('BT Error sending in Exhalation conditional')
		    send_last_time = time()
		sending_time =time() - send_last_time 
#		print("THE TIME TAKEN IS")
#		print(cuurent_time - start_time)
                flow=self.rate()
	        if(indiff <= peep_val+1 and comp_flag == '1' and leak_comp_flag == '1'):
		    trigflow_comp = trigflow + running_avg + 3
		    comp_flag = '0'
		    running_avg = 0
#                flow=self.rate()
#		flow = flow - peep_hole[int(peep_val)]
                current_time=time()
		if(leak_comp_flag == '0'):
		    trigflow_comp = trigflow
            #    volume=0
		TOT_last = current_time - exhale_time_last
	#	peep_val_send = indiff
	        if(comp_flag == '0' and flow > 1):
		    running_avg = (running_avg + flow)/2
		time_error = time() - time_error_last
                if(indiff <= peep_val+2 and flow >= -10):
                    sol.ChangeDutyCycle(100)
		SDP_flag = 0
		loop = 1
		print("THE TIME TAKEN IS")
		print(current_time - start_time)
                if(flow >= trigflow  or (current_time-start_time)>cycle_time):
		    sol.ChangeDutyCycle(100)
                    motor_1.ChangeDutyCycle(100)
		    if(VTi_max <= 200):
			motor_1.ChangeDutyCycle(pump_pressure)
		    RR_time=current_time-start_time
		    pump_pressure_now = pump_pressure
		    patient_trigger_flow = flow
		    inhale_loop = "1"
		    GPIO.output(33, GPIO.LOW)
		    del inhale_array[:]
		    volume = 0
		    leak_comp_flag = leak_comp()
		    comp_flag = '1'
		    GPIO.output(35, GPIO.LOW)
		    peep_val_send = temp_peep
 		    if(len(exp_press_array) > 10):
 			peep_val_send = exp_press_array[len(exp_press_array) - 4]
 		    del exp_press_array[:]
# 		    else:
		   # GPIO.output(19,GPIO.HIGH)
# 		    sol.ChangeDutyCycle(100)
#                     motor_1.ChangeDutyCycle(100)
# 		    if(P_plat <= 22):
# 			motor_1.ChangeDutyCycle(80)
#		    if(P_plat <= 17):
#			motor_1.ChangeDutyCycle(65)
#			pump_pressure_now = pump_pressure /2
 #                   RR_time=current_time-start_time
		    if(time_error < 0.7):
			time_error_set = 1
#			try:
#			    ser.write('ACK24')
#			except:
#			    print('unable to send data')
		    if(time_error > 0.7):
			time_error_set = 0
#			try:
#			    ser.write('ACK24')
#			except:
#			    print('unable to send data')
		    if(float(ratio) <= 0.25):
			ratio_set = 1
#			try:
#			    ser.write('ACK12')
#			except:
#			    print('unable to send data')
		    if(float(ratio) >= 0.75):
			ratio_set = 1
#			try:
#			    ser.write('ACK13')
#			except:
#			    print('unable to send data')
		    if(float(ratio) >0.25 and float(ratio) < 0.75):
			ratio_set = 0
#			try:
#			    ser.write('ACK22')
#			except:
#			    print('unable to send data')
		    if(RR_time > cycle_time):
			trigger='0'
		    else:
			trigger='1'
                    #motor_2.ChangeDutyCycle(pump_pressure)
                    indiff= self.ABP_pressure()
                  #  print(indiff)
                    time_elapsed_inhale=0
                    indiff = 0
		    volume=self.Flow()
#		    print('the value of volume is')
#		    print(volume)
		    if(volume == -9999):
			SDP_flag = 1
			print('well hrre you gooooooooooooooooooooooooooooooooo')
		    Pmean=0
		    del Pmean_array[:]
		    clock_t2= time()
		    volume_flag = 0
		    volume_reach_time = 0
		    volume=0
		    volume_peak_inhale=0
		    sending_time = 0.2
		    send_last_time=0
		    VTi_volume=0
		    flow_flag = 0
		    volFlow_rate = self.rate()
		    pump_flag = 0
		    pump_pressure_now = pump_pressure/2
		    peak_insp_pressure = 0
		    volume_comp = 0
		    peak_flow = volFlow_rate
		    rise_flag = '0'
                    while(indiff < PIP and time_elapsed_inhale <= inhale_time):
                    #    motor_1.ChangeDutyCycle(pump_pressure)
                        #motor_2.ChangeDutyCycle(pump_pressure)
			GPIO.output(37, GPIO.HIGH)
                        t3 = time()
			print("inhale loop")
                        indiff= self.ABP_pressure()
                        time_elapsed_inhale= t3-current_time
      #                  volume_reach_time = time_elapsed_inhale
                        print(time_elapsed_inhale)
			print(volume)
			print(VTi_max)
			print(indiff)
			print(PIP)
			Pmean_array.append(indiff)
			if(indiff >= P_plat * 0.8 and rise_flag == '0'):
				rise_time = time_elapsed_inhale
				rise_flag = '1'
			if(SDP_flag == 0):
			    volume=self.Flow()
			    volume_comp = volume
			    volume_comp = volume_comp - ((volume_comp*leak_percentage)/100)
			    print('hello its me')
			    volFlow_rate=self.rate()
			if(volFlow_rate > peak_flow):
			    peak_flow = volFlow_rate
			if(volFlow_rate <  peak_flow * 0.25 and time_elapsed_inhale >= 0.5 and trigger == '1'):
			    break 
			if(sending_time > 0.02):
			    packet_inhalation =('A@' + str(round(indiff,2)) + ','+str(round(volFlow_rate,2)) + ',' + str(int(volume)) + ',' + str(trigger) + '#')
			    try:
				ser.write(packet_inhalation)
			    except:
				print('BT error send Inhalation')
#			    print('packet size isssssssssss')
#			    print(len(packet_inhalation))
#			    print('packet conatins ')
#			    print(packet_inhalation)
			    send_last_time= time()
			sending_time=t3 - send_last_time
			print('inhale_time is')
			if((indiff >= P_plat*motor_factor or time_elapsed_inhale >= 0.5) and flow_flag == 0): #(time_elapsed_inhale > 0.3 and flow_flag == 0) or indiff >= 10): #or indiff >= P_plat - 4): #or volFlow_rate >= flow_plat - 10):
			    flow_flag = 1
			    motor_1.ChangeDutyCycle(pump_pressure)
#			    print('CMHO reacged')
#			    print(indiff)
# 			if(P_plat <= 17 and (indiff >= P_plat/2 or time_elapsed_inhale >= 0.2) and flow_flag == 0): #(time_elapsed_inhale > 0.3 and flow_flag == 0) or indiff >= 10): #or indiff >= P_plat - 4): #or volFlow_rate >= flow_plat - 10):
# 			    flow_flag = 1
# 			    motor_1.ChangeDutyCycle(pump_pressure)
# 			if(flow_flag == 1 and volFlow_rate <= flow_plat):
# 			    motor_1.ChangeDutyCycle(pump_pressure)
			if(volFlow_rate >= flow_plat and flow_flag ==1):
			    flow_flag = 2
			if(flow_flag == 3):
			    if(volFlow_rate <= flow_plat and pump_pressure_now <= 98):
				pump_pressure_now = pump_pressure_now + 1
			        motor_1.ChangeDutyCycle(pump_pressure_now)
			    if(volFlow_rate >= flow_plat and pump_pressure_now >= 2): #and pump_pressure_now >( pump_pressure /2)):
			        pump_pressure_now = pump_pressure_now - 1
			        motor_1.ChangeDutyCycle(pump_pressure_now)
# 			if(pump_pressure_now <= pump_pressure and volFlow_rate <= flow_plat):
# 			    if(pump_flag == 0):
# 			        pump_pressure_now  = pump_pressure_now + 1
# 			    else:
# 				pump_pressure_now = pump_pressure_now + 0.5
# 	#		    print('the pump_pressure now is')
# 	#		    print(pump_pressure_now)
# 	#		    print('pump pressure is')
# 	#		    print(pump_pressure)
		        if(volume_comp >= VTi_max and volume_flag == 0 and trigger == '0'):
		    	    volume_reach_time = time_elapsed_inhale
		    	    GPIO.output(33, GPIO.HIGH)
		    	    volume_flag = 1
			print(inhale_time)
			inhale_array.append(indiff)
			VTi_volume = max(volume,VTi_volume)
			peak_insp_pressure = max(peak_insp_pressure, indiff)
			volume_peak_inhale=max(volume_peak_inhale,volFlow_rate)
		    if(IHold == '1'):
			motor_1.ChangeDutyCycle(int(pump_pressure*0.9))
			GPIO.output(33, GPIO.HIGH)
			print("entering inspiratory hold")
			print(IH_time)
			ser.write("ACK66")
			instant_time = time()
		        sending_time = 0.2
		        send_last_time=0
			while(time() - instant_time < IH_time):
			    print(sending_time)
			    indiff= self.ABP_pressure()
			    volFlow_rate=self.rate()
			    if(sending_time > 0.02):
			        packet_inhalation =('A@' + str(round(indiff,2)) + ','+str(round(volFlow_rate,2)) + ',' + str(int(volume)) + ',' + str(trigger) + '#')
			        try:
				    ser.write(packet_inhalation)
			        except:
				    print('BT error send Inhalation')
			        send_last_time= time()
			    sending_time=time() - send_last_time
		    IHold = '0'
		    GPIO.output(33, GPIO.LOW)
		    if(indiff >= (PIP+2)):
			pressure_count = pressure_count +1
			if(pressure_count >= 3):
		            try:
		  	        ser.write('ACK07')
			    except:
		      	        print('unable to send')
		    else:
			pressure_count = 0
			try:
			    ser.write('ACK57')
			except:
			    print('unable to send')
		    if(indiff <= (P_plat)*0.5):
			pressure_low_count = pressure_low_count +1
			if(pressure_low_count >= 3):
		            try:
		  	        ser.write('ACK62')
			    except:
		      	        print('unable to send')
		    else:
			pressure_low_count = 0
			try:
			    ser.write('ACK72')
			except:
			    print('unable to send')
		    GPIO.output(37, GPIO.LOW)
		    MVi_array.append(VTi_volume)
		    VTi_volume = VTi_volume - ((VTi_volume*leak_percentage)/200)
                    flag=1
		    loop = 0
		    if(VTi_volume < (VTi_max*0.8) and indiff >= PIP and patient_status == 1):
		        ser.write("ACK61")
		    else:
			ser.write("ACK71")
		    if(VTi_volume < (VTi_max*0.8) and pump_pressure >= 95 and patient_status == 1):
			ser.write("ACK60")
		    else:
			ser.write("ACK70")
		    toggle_switch = str(toggle())
		    if(EHold == '1'):
			toggle_switch ='1'
	#	    print("the toffle switch in here issssssssssssssssssssssssssss " + toggle_switch)
		#    motor_1.ChangeDutyCycle(peep)
		    sleep(0.01)
		   # GPIO.output(35, GPIO.HIGH) # low level triggered relay
		    if(toggle_switch == "0"):
			print("got in the first statement")
		        motor_1.ChangeDutyCycle(peep_open)
			sol.ChangeDutyCycle(0)
		    if(toggle_switch == "1"):
			print("got in the 2nd statement")
	#		GPIO.output(33, GPIO.HIGH)
		        motor_1.ChangeDutyCycle(peep)
			sol.ChangeDutyCycle(0)
	#		sleep(0.01)
	#		GPIO.output(21, GPIO.HIGH)
		    inhale_array_length = len(inhale_array)
		    if(inhale_array_length % 2 != 0):
			inhale_array.append(0)
			inhale_array_length = inhale_array_length + 1
		    inhale_first_half = inhale_array[0:inhale_array_length/2]
		    inhale_second_half = inhale_array[inhale_array_length/2:]
		    first_max = max(inhale_first_half)
		    second_max = max(inhale_second_half)
		    if(first_max - P_plat > 0.5 and motor_factor > 0.2):
		        motor_factor = motor_factor - 0.01
			if(first_max - P_plat > 2):
			    motor_factor = motor_factor - 0.01
		    if(P_plat - first_max > 0.5 and motor_factor <= 1.0):
			motor_factor = motor_factor + 0.01
			if(P_plat - first_max > 2):
			    motor_factor = motor_factor + 0.01
		    if(VTi_volume < (VTi_max * 0.8)):
			volume_count = volume_count + 1
			if(volume_count >= 3):
			    buzzer(7,1)
			    try:
			        ser.write('ACK36')
			    except:
			        print('unable to send')
		    else:
			buzzer(7,0)
			volume_count = 0
			try:
			    ser.write('ACK46')
			except:
			    print('unable to send')
		    try:
		        Pmean= sum(Pmean_array)/len(Pmean_array)
		    except:
			print("zero exception error")
		    try:
		        if(compliance_flag == 0):
		            compliance_array.append(VTi_volume/(peak_insp_pressure - peep_val))
			    if(len(compliance_array) == 2):
			        compliance = (sum(compliance_array)/len(compliance_array)) * 0.75
			        pressure = (VTi_max/compliance) + peep_val
			        if(pressure > PIP):
				    pressure  = PIP - 2
	        	        P_plat_value=find_nearest(P_plat_array,pressure)
	        	        index= P_plat_array.index(P_plat_value)
				inst_pump_pressure = pump_pressure_array[index]
				if(inst_pump_pressure > pump_pressure):
	        	            pump_pressure = pump_pressure_array[index]  
			        compliance_flag = 1
		    except:
			print("error")
#		    print('Pmean is')
#		    print('Here it comessssssssss')
#		    print(int(VTi_volume))
#		    print(VTi_max)
#		    print(peak_insp_pressure)
#		    print(patient_status)
                    RR.append(RR_time)
#		    print(pump_pressure_high)
		    indiff = int(indiff)
		    if((VTi_volume > VTi_max*1.1 or (volume_reach_time < inhale_time*0.8 and trigger == '0' and VTi_volume >= VTi_max)) and ABP_flag == 0 and SDP_flag == 0 and peak_insp_pressure >= peep_val+5 and patient_status == 1 and pump_pressure >= 6 and pump_pressure <= 99):
			pump_pressure = pump_pressure - 1
			P_plat = indiff
			print('-1')
		    elif((VTi_volume < VTi_max or volume_reach_time >= inhale_time*0.8) and ABP_flag == 0 and SDP_flag == 0 and indiff < PIP and patient_status ==1 and pump_pressure >= 5 and pump_pressure <= 95):
			pump_pressure = pump_pressure + 1
# 			if((VTi_max - VTi_volume) > 100 and leak_percentage < 70 and VTi_max > 100):
# 			    if(((VTi_max - VTi_volume)/3 ) > 5):
# 				pump_pressure = pump_pressure + 5
# 			    else:
# 			        pump_pressure = pump_pressure + ((VTi_max - VTi_volume)/30)
# 			elif(VTi_max > 100):
# 			    pump_pressure = pump_pressure + 1
# 			if(VTi_max <= 100):
# 			    pump_pressure = pump_pressure + 5
			P_plat = indiff
		    if(pump_pressure > 95):
			pump_pressure = 95
		    if(patient_status == 1 and peep_val_send < peep_val and peep <= peep_first + 10  and peep >= 4 and  peep < 90 and toggle_switch == '1'):
			peep = peep + 1
		    elif(patient_status == 1 and peep_val_send > peep_val and peep >= peep_first - 10  and peep > 5 and peep < 92 and toggle_switch == '1'):
			peep = peep - 1
			print("reducing the peeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeep")
		    print("Diagnostics")
		    print(VTi_max)
		    print(VTi_volume)
	#	    MVi_array.append(VTi_volume)
		    print(peak_insp_pressure)
		    print(patient_status)
		    print(P_plat)
		    print(pump_pressure)
		if(flag == 1):
	   	    patient_set = 0
		    if((leak_percentage > 90 or indiff <= 5)  and patient_set == 0 ):
			buzzer(5,1)
			patient_set = 1
			patient_status = 0
		        print('disconnection')
#			trigger = '0'
			try:
			    ser.write('ACK05')
			except:
			    print('not able to send')
		       # GPIO.output(22, GPIO.HIGH)
		    if((leak_percentage <= 90 or indiff > 5) and patient_set == 0):
			buzzer(5,0)
			patient_set = 1
			patient_status = 1
#			trigger = '1'
			try:
			    ser.write('ACK06')
			except:
			    print('not able to send')
                    if(len(RR) >= 5):
                        BPM = ( RR[1] + RR[2] + RR[3] + RR[4]) /4
                        BPM = ceil(60/BPM)
			if(BPM > 70):
			    try:
				ser.write('ACK09')
			    except:
				print('unable to send')
			else:
			    try:
				ser.write('ACK59')
			    except:
				print('unable to send')
 #                       print('BPM is')
  #                      print(BPM)
                        RR.pop(0)
		    if(len(MVi_array) >= 5):
		        MVi= (MVi_array[1] + MVi_array[2] + MVi_array[3] + MVi_array[4])/4
			MVi = MVi * BPM
#		        print('MVi value is')
#		        print(MVi)
		        MVi_array.pop(0)
		        packet_end_inhalation = ('B@' + str(round(peak_insp_pressure,2)) + "," + str(int(VTi_volume)) + "," + str(round(volume_peak_inhale,2)) + "," + str(round(Pmean,2)) + "," + str(round((MVi/1000),2)) + "," + str(round(patient_trigger_flow,2)) + "," + str(round(time_elapsed_inhale,2)) + "," + str(round(indiff,2)) + "," + str('0') + "," + str(round(rise_time,2)) + '#')
#		        print('packet length is')
		        print(packet_end_inhalation)
		        try:
		            ser.write(packet_end_inhalation)
		        except:
			    print('BT Errror in end of inhalation loop')
                if(flag==1):
		    time_error_last = time()
		    time_elap = 0
		    q= time()
		    del exp_press_array[:]
		    del exp_flow_array[:]
		    inhale_loop = "0"
		    volume = 0
		    sol_flag = 0
		    sending_time = 200
		    send_last_time = 0
		    volume_peak_exhale = 0
		    exhale_break = '0'
		    while(time_elap < 0.1 or (toggle_switch == "1" and time_elap < 0.3)):
			w=time()
			time_elap=w-q
#			exp_flow = self.rate()
 #                       exp_flow_array.append(exp_flow)
                        indiff = self.ABP_pressure()
			exp_press_array.append(indiff)
			peak_insp_pressure = int(peak_insp_pressure)
			if(peak_insp_pressure <= 0):
			    peak_insp_pressure = 1
			ratio = 1.0
                        if(ratio > 0.8 and time_elap >= 0.2 and toggle_switch == "1" and exhale_break == '0'):
                            GPIO.output(35, GPIO.LOW)
                            sleep(0.05)
 #                           motor_1.ChangeDutyCycle(peep)
			    exhale_break = '1'
                        elif(ratio >= 0.15 and ratio <= 0.4 and time_elap >= 0.4 and P_plat < 28 and toggle_switch == "1" and exhale_break == '0'):
                            GPIO.output(35, GPIO.LOW)
                            sleep(0.05)
  #                          motor_1.ChangeDutyCycle(peep)
			    exhale_break = '1'
                        elif(ratio >= 0.15 and ratio <= 0.4 and time_elap >= 0.4 and P_plat >=28 and toggle_switch == "1" and exhale_break == '0'):
                            GPIO.output(35, GPIO.LOW)
                            sleep(0.05)
   #                         motor_1.ChangeDutyCycle(peep)
			    exhale_break = '1'
                        elif(ratio > 0.4 and ratio <= 0.8 and time_elap >= 0.3 and toggle_switch == "1" and exhale_break == '0'):
                            GPIO.output(35, GPIO.LOW)
                            sleep(0.05)
    #                        motor_1.ChangeDutyCycle(peep)
			    exhale_break = '1'
                        elif(ratio <0.15 and ratio >=0.00 and time_elap >= 0.4 and toggle_switch == "1" and exhale_break == '0'):
                            GPIO.output(35, GPIO.LOW)
                            sleep(0.05)
     #                       motor_1.ChangeDutyCycle(peep)
			    exhale_break = '1'
			diff = self.SDP_pressure()
			if(SDP_flag == 0):
			    volFlow_rate = self.rate()
			    volume=self.Flow()
			if(diff == 700 or diff == -700 or SDP_flag == 1):
			    try:
				f = open("/home/pi/AgVa_5.0/mode.txt","w")
				f.write(str('11'))
				f.close()
				try:
				    ser.write('ACK30')
				except:
				    print('unable to send data')
			    except:
				print('unable to write mode')
			packet_exhalation= ('C@' + str(round(indiff,2)) + ',' + str(round(volFlow_rate,2)) +',' + str(int(volume)) + ',' + str(int((TITOT)*100)) + '#')
			if(sending_time > 0.02):
			    try:
				ser.write(packet_exhalation)
			    except:
				print('BT exhalation error')
			    send_last_time = time()
			sending_time = w- send_last_time
# 		        if(toggle_switch == "1" and indiff <= peep_val and sol_flag == 0 ):
#                             sol.ChangeDutyCycle(100)
# 			    holding_now = time()
# 			    holding_time = 0.3
# 			    peep_check = peep_val - trigflow - 1
# 			    if(peep_check < 0):
# 			        peep_check = 0
# 			    while( time() - holding_now <= holding_time and time_elap <= 0.3):
# 				time_elap = time() - q
# 				GPIO.output(33, GPIO.LOW)
# 				indiff = self.ABP_pressure()
# 				if(indiff <= peep_check):
# 				    break;
# 			        volFlow_rate = self.rate()
# 			        volume=self.Flow()
# 				packet_exhalation= ('C@' + str(round(indiff,2)) + ',' + str(round(0,2)) +',' + str(int(volume)) + ',' + str(int((TITOT)*100)) + '#')
# 				if(1 == 1):
# 			    	    try:
# 					ser.write(packet_exhalation)
# 			    	    except:
# 					print('BT exhalation error')
# 			    	    send_last_time = time()
# 				sleep(0.02)
# 			    sending_time = w- send_last_time
# 	#		    sleep(0.02)
# 			    sol_flag == 1
#			print('packet exhalation size  is')
#			print((packet_exhalation))
			volume_peak_exhale= min(volume_peak_exhale, volFlow_rate)
			print('packet_exhal;ation conatins is')
			print(packet_exhalation)
# 		    if(toggle_switch == "1" and indiff <= peep_val-peep_factor and sol_flag == 0):
#                         sol.ChangeDutyCycle(100)
# 			holding_now = time()
# 			holding_time = 0.3
# 			sending_time = 2.0
# 			send_last_time = 0
# 			while( time() - holding_now <= holding_time):
# 			    indiff = self.ABP_pressure()
# 			    volFlow_rate = self.rate()
# 			    volume=self.Flow()
# 			    packet_exhalation= ('C@' + str(round(indiff,2)) + ',' + str(round(0,2)) +',' + str(int(volume)) + ',' + str(int((TITOT)*100)) + '#')
# 			    if(1 == 1):
# 			        try:
# 				    ser.write(packet_exhalation)
# 			    	except:
# 				    print('BT exhalation error')
# 			    	send_last_time = time()
# 			    sending_time = time()- send_last_time
# 			    sleep(0.02)
# 			    sol_flag = 1
		    temp_peep = self.ABP_pressure()
#
                    new_time=time()
                    time_elapsed_exhale=0.0
                    diff=self.SDP_pressure()
		    start_time=current_time
		    sending_time=200
		    time_elapsed_exhale_flow = 0
#		    sol_flag  = 0
		    send_last_time = 0
                    while((time_elapsed_exhale<(cycle_time-inhale_time-0.3) and (diff <= 0 or indiff > peep_val* 1.5)) or (SDP_flag ==1 and time_elapsed_exhale<(cycle_time-inhale_time-0.5))):
                        time_elapsed_exhale=time()-new_time
#			exp_flow = self.rate()
  #                      exp_flow_array.append(exp_flow)
			indiff = self.ABP_pressure()
			exp_press_array.append(indiff)
			if((toggle_switch == "1" and sol_flag == 0 and ((indiff <= peep_val+2 and volFlow_rate >= -10) or (indiff <= peep_val-2))) or (time_elapsed_exhale>(cycle_time-inhale_time-0.3)) ):
			    peep_check = peep_val - trigflow - 1
			    if(peep_check < 0):
			        peep_check = 0
			    if(volFlow_rate >= trigflow):
			        break;   
			    sol.ChangeDutyCycle(100)
			    sol_flag  = 1
			    holding_now = time()
			    holding_time = 0.3
# 			    peep_check = trigflow - peep_val - 2.0
# 			    if(peep_check < 0):
# 			        peep_check = 0
			    while( time() - holding_now <= holding_time and (time_elapsed_exhale<(cycle_time-inhale_time-0.3))):
				time_elapsed_exhale = time() - new_time
				GPIO.output(33, GPIO.LOW)
				indiff = self.ABP_pressure()
				if(volFlow_rate >= trigflow+peep_val):
				    break;
			        volFlow_rate = self.rate()
			        volume=self.Flow()
				packet_exhalation= ('C@' + str(round(indiff,2)) + ',' + str(round(0,2)) +',' + str(int(volume)) + ',' + str(int((TITOT)*100)) + '#')
				if(sending_time > 0.02):
			    	    try:
					ser.write(packet_exhalation)
			    	    except:
					print('BT exhalation error')
			    	    send_last_time = time()
				sending_time = time()- send_last_time
				sleep(0.02)
				sol_flag = 1
			if(SDP_flag == 0):
           		    volume=self.Flow()
			    diff = self.SDP_pressure()
			    volFlow_rate=self.rate()
			volume_peak_exhale=min(volume_peak_exhale,volFlow_rate)
 			if(volFlow_rate < -5):
 			    time_elapsed_exhale_flow = time_elapsed_exhale
			packet_exhalation = ('C@' + str(round(indiff,2)) + ',' + str(round(volFlow_rate,2)) + ',' + str(int(volume)) + ',' + str(int((TITOT)*100))+'#')
			if(sending_time > 0.02):
			    try:
				ser.write(packet_exhalation)
			    except:
				print('BT Error sending in Exhalation conditional')
			    send_last_time = time()
			sending_time =time() - send_last_time 
                        start_time=current_time
                    #    print(start_time)
                    if(int(peep_val_send) >=  peep_val - 1 and int(peep_val_send) <= peep_val+1):
                        lock = 1
                    elif(int(peep_val_send) > peep_val+1 or int(peep_val_send) < peep_val -1):
                        lock = 0
		    if(EHold == '1'):
			GPIO.output(33, GPIO.HIGH)
			print("Entering expiratory hold")
			instant_time = time()
			ser.write("ACK67")
		        sending_time = 0.2
		        send_last_time=0
			while(time() - instant_time < EH_time):
			    indiff= self.ABP_pressure()
			    print(time() - instant_time)
			    volFlow_rate=self.rate()
			    if(sending_time > 0.02):
			        packet_exhalation = ('C@' + str(round(indiff,2)) + ',' + str(round(volFlow_rate,2)) + ',' + str(int(volume)) + ',' + str(int((TITOT)*100))+'#')
			        try:
				    ser.write(packet_exhalation)
			        except:
				    print('BT error send Inhalation')
			        send_last_time= time()
			    sending_time=time() - send_last_time
			peep_val_send = indiff
			GPIO.output(33, GPIO.LOW)
		    EHold = '0'
#		    flow_for_peep = 0.25 * (min(exp_flow_array))
 #                   print(flow_for_peep)
  #                  value1 = find_nearest(exp_flow_array , flow_for_peep)
   #                 peep_array_index = exp_flow_array.index(value1)
 #                   peep_val_send = exp_press_array[peep_array_index]
                    flag=0
		    loop = 1
		    inhale_loop = "1"
		    exhale_time_last = time()
		    time_elapsed_exhale = time_elapsed_exhale + 0.5
#		    peep_val_send = temp_peep
#		    if(len(exp_press_array) > 10):
#		        peep_val_send =  exp_press_array[len(exp_press_array) - 3]
		    TITOT=(time_elapsed_inhale/(time_elapsed_inhale + time_elapsed_exhale + TOT_last))
#		    print('peak volume flow rate exhale')
#		    print(volume_peak_exhale)
		    if(int(peep_val_send) <= peep_val and patient_status == 1 and toggle_switch == '1' and lock == 0):
			peep_factor = peep_factor - (peep_val - peep_val_send)*0.1
		    if(int(peep_val_send) > peep_val and patient_status == 1 and toggle_switch == '1' and lock  == 0):
			peep_factor = peep_factor + (peep_val_send - peep_val)*0.1
		    if(int(peep_val_send) > peep_val+2):
			peep_count = peep_count + 1
			if(leak_percentage > 20 and trigger == '0'):
			    ser.write("ACK39")
			if(peep_count >=3 and peep_set == 0):
			    buzzer(4,1)
			    peep_set = 1
			    try:
			        ser.write('ACK08')
			    except:
			        print('unable to send')
		    else:
			if(peep_set == 1):
			    buzzer(4,0)
			    peep_set = 0
			ser.write("ACK49")
			peep_count = 0
			try:
			    ser.write('ACK58')
			except:
			    print('unable to send')
		    MVe_array.append(volume)
		    FiO2 = self.FiO2()
                    self.power()
		    print('FiO2 is')
		    print(FiO2)
		    if(toggle_switch == '1'):
		        time_elapsed_exhale = time_elapsed_exhale + 0.4
			time_elapsed_exhale_flow = time_elapsed_exhale_flow + 0.4
		    else:
			time_elapsed_exhale = time_elapsed_exhale + 0.1
			time_elapsed_exhale_flow = time_elapsed_exhale_flow + 0.1
		    if(len(MVe_array)>=5):
		        MVe= (MVe_array[1] + MVe_array[2] + MVe_array[3] + MVe_array[4])/4
			MVe = MVe * BPM
		        MVe_array.pop(0)
		        try:
			    leak_percentage = ((MVi - MVe)/MVi)*100
			    if(leak_percentage <= -1):
			        leak_percentage = 0
		        except:
			    leak_percentage = 0
#		        print('MVe is')
#		        print(MVe)
		   # packet_end_exhalation = 'ABC'
		        packet_end_exhalation = ('D@' + str(round(peep_val_send,2)) + ',' + str(int(BPM)) + ',' + str(FiO2) + ',' + str(round(volume_peak_exhale,2)*-1)  + "," + str(round((MVe/1000),2)) + "," + str(leak_percentage) + "," + str(trigflow_comp - trigflow) + "," + str(volume)  + "," + str(time_elapsed_exhale_flow) +  '#')
#			print('string issssssssssssssssssssssssssssssssssssss')
			print(packet_end_exhalation)
		        try:
	                    ser.write(packet_end_exhalation)
		        except:
			    print('BT send error in end of exhalation')
#		    try:
#		        with open('/home/pi/AgVa_5.0/data.csv', mode='a') as data:
 #  		            data = csv.writer(data, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
#		            now = datetime.datetime.now()
#		            data.writerow([str(now),str(self.measure_temp()),str(Pmean),str(volume),str(volume_peak_inhale),str(trigger),str(BPM),str(MVi),str(ratio), str(volume_peak_exhale),str(MVe)])
#		    except:
#			print('data file open error')
        except KeyboardInterrupt:
            return
    def NIV_CPAP(self):
        global starting_flag,change_setting,volume_flag,peep,peep_open,time_elapsed_exhale_flow,patient_trigger_flow,rise_time, rise_flag ,pressure_low_count,BIPAP_backup_flag,back_backup,inhale_loop,toggle_switch,loop,peep,patient_set,ABP_flag,exhale_time_last,TOT_last,TITOT,pressure_count,peep_count, SDP_flag,flow_plat,indiff,patient_status,pump_pressure,pump_pressure_low,pump_pressure_high,ratio_set,time_error_set,thread_mode_status,time_error, time_error_last,ABP_flag,P_plat_value,prev_mode,data,ratio,RR_time,indiff,peep_val, peep_hole, RR, BPM, MVi_array, MVi,volume, volume_peak_inhale, trigger, max_value, volume_peak_exhale, MVe_array, MVe,clock_t2
        thread_mode_status = True
	time_elapsed_exhale_flow = 0
	rise_time = 0
	starting_flag = 0
	volume_flag = 0
	patient_trigger_flow = 0
	rise_flag = '0'
	pressure_low_count = 0
	patient_set = 0
	exhale_time_last = time()
	TOT_last = time()
	GPIO.output(33, GPIO.LOW)
	peep_count = 0
	inhale_loop = "0"
	flag = 0
	loop = 1
	BIPAP_backup_flag = 0
	GPIO.output(35, GPIO.LOW)
	VTi_volume = 0
	leak_percentage = 0
	del MVi_array[:]
	del MVe_array[:]
	pressure_count = 0
	peep_set = 0
	ratio_set = 0
	time_error_set = 0
	trigger = '1'
	sol.ChangeDutyCycle(100)
        start_time=time()
	current_time = time()
	time_error_last = time()
	sending_time = 0.2
        try:
            while True:
		prev_mode = 31
		ABP_flag = 0
#		print('we are in PC_IMV mode')
		data= self.read_data()
#		print('mode is')
#		print(data)
		if(data == 1 or (current_time-start_time) >= 10):
		    if(current_time-start_time >= 10):
		        try:
			    f = open("/home/pi/AgVa_5.0/backup.txt","w")
			    f.write("310")
			    f.close()
			    back_backup = 1
			    f = open("/home/pi/AgVa_5.0/mode.txt","w")
			    f.write(str("350"))
			    f.close()
			    BIPAP_backup_flag = 1
			    change_setting = '1'
			except:
			    print("unable to fall back to PC_IMV mode")
			try:
			    ser.write("ACK38")
			except:
			    print("unable to send ACK38")
		    break;
#		print(inhale_time)
#		print(BPM)
#		print(PIP)
#		PIP = P_plat +1
	        inhale_time = 3.0
		trig_flow = trigflow + 5
#		print(VTi_max)
#		print('peep in main loop is')
#		print(peep)
#		print(peep_val)
#		print(P_plat)
#		print(P_plat_value)
#		print('here the pump pressure is')
#		print(pump_pressure)
	#	GPIO.output(22, GPIO.LOW)
		volFlow_rate = self.rate()
		indiff= self.ABP_pressure()
		if(indiff == -8888):
			ABP_flag = 1
			indiff = 0
		packet_exhalation = ('C@' + str(round(indiff,2)) + ',' + str(round(volFlow_rate,2)) + ',' + str(int(volume)) + ',' + str(int((TITOT)*100))+'#')
		if(sending_time > 0.02):
		    try:
			ser.write(packet_exhalation)
		    except:
			print('BT Error sending in Exhalation conditional')
		    send_last_time = time()
		sending_time =time() - send_last_time 
#		print('pump_pressure is')
#		print(pump_pressure)
                flow=self.rate()
#		print('before flow')
#		print(flow)
	#	flow = flow - peep_hole[int(peep_val)]
#		print('after flow')
#		print(flow)
                current_time=time()
            #    volume=0
		TOT_last = current_time - exhale_time_last
		peep_val_send = indiff
		SDP_flag = 0
		time_error = time() - time_error_last
		loop = 1
                if(flow>trig_flow or starting_flag == 0):
		    RR_time=current_time-start_time
		    starting_flag = 1
		    pump_pressure_now = pump_pressure
		    patient_trigger_flow = flow
		    inhale_loop = "1"
		    volume = 0
		    indiff = self.ABP_pressure()
		    sol.ChangeDutyCycle(100)
		    flow_flag = 0
#		    if((pump_pressure /2) < peep):
#			motor_1.ChangeDutyCycle(peep + 2)
#			pump_pressure_now = peep +2
#		    else:
#		    print('indiff right now is')
#		    print(indiff)
#		    if(indiff <= P_plat/2):
                    motor_1.ChangeDutyCycle(pump_pressure)
		    if(P_plat <= 22):
			motor_1.ChangeDutyCycle(pump_pressure)
#		    if(P_plat <= 17):
#			motor_1.ChangeDutyCycle(65)
#		        flow_flag = 0
#		    else:
#			motor_1.ChangeDutyCycle(pump_pressure)
#			flow_flag = 1
#			pump_pressure_now = pump_pressure /2
		    if(time_error < 0.7):
			time_error_set = 1
#			try:
#			    ser.write('ACK24')
#			except:
#			    print('unable to send data')
#		    if(time_error > 0.7):
#			time_error_set = 0
#			try:
#			    ser.write('ACK24')
#			except:
#			    print('unable to send data')
		    if(float(ratio) <= 0.25):
			ratio_set = 1
#			try:
#			    ser.write('ACK12')
#			except:
#			    print('unable to send data')
		    if(float(ratio) >= 0.75):
			ratio_set = 1
#			try:
#			    ser.write('ACK13')
#			except:
#			    print('unable to send data')
		    if(float(ratio) >0.25 and float(ratio) < 0.75):
			ratio_set = 0
#			try:
#			    ser.write('ACK22')
#			except:
#			    print('unable to send data')
#		    if(RR_time > cycle_time):
#			trigger='0'
#		    else:
		    trigger='1'
                 #   indiff= self.ABP_pressure()
                  #  print(indiff)
                    time_elapsed_inhale=0
                    indiff = 0
		    Pmean=0
		    del Pmean_array[:]
		    clock_t2= time()
#		    print('now it is')
#		    print(clock_t2)
		    volume=0
		    volume_peak_inhale=0
		    sending_time = 0.2
		    send_last_time=0
		    volFlow_rate = self.rate()
		    VTi_volume=0
		    pump_flag = 0
		   # flow_flag = 0
		    current_time = time()
		    peak_insp_pressure = 0
		    pump_pressure_now = pump_pressure /2
#		    flow_zero_intialize = time() # clock starts to detect zero flow
#		    flow_zero_flag = 0
#		    flow_zero_elapsed = 0
		    peak_flow = volFlow_rate
		    rise_flag = '0'
                    while(((peak_flow*0.60 <= volFlow_rate) and time_elapsed_inhale <= inhale_time) or time_elapsed_inhale <= 0.5):
                    #    motor_1.ChangeDutyCycle(pump_pressure)
                        #motor_2.ChangeDutyCycle(pump_pressure)
                        t3 = time()
			GPIO.output(37, GPIO.HIGH)
#			if(volFlow_rate <= 2):
#			    flow_zero_flag = 1
#			    flow_zero_initialize = time()
#		        if(flow_zero_flag == 1):
#			    flow_zero_elapsed = time() - flow_zero_initialize
#			    if(flow_zero_elapsed >= )
#			    flow_zero_elapsed = time() - flow_zero_initialize 
			if(ABP_flag == 0):
                            indiff= self.ABP_pressure()
                        time_elapsed_inhale= t3-current_time
			if(indiff >= P_plat * 0.8 and rise_flag == '0'):
				rise_time = time_elapsed_inhale
				rise_flag = '1'
                   #     print(indiff)
			Pmean_array.append(indiff)
			if(SDP_flag == 0):
			    volume=self.Flow()
			    volFlow_rate=self.rate()
			    if(volFlow_rate >= peak_flow):
				peak_flow = volFlow_rate
			if((indiff >= P_plat*motor_factor or time_elapsed_inhale >= 0.5) and flow_flag == 0): #(time_elapsed_inhale > 0.3 and flow_flag == 0) or indiff >= 10): #or indiff >= P_plat - 4): #or volFlow_rate >= flow_plat - 10):
			    flow_flag = 1
			    motor_1.ChangeDutyCycle(pump_pressure)
#			    print('CMHO reacged')
#			    print(indiff)
# 			if(P_plat <= 17 and (indiff >= P_plat/2 or time_elapsed_inhale >= 0.2) and flow_flag == 0): #(time_elapsed_inhale > 0.3 and flow_flag == 0) or indiff >= 10): #or indiff >= P_plat - 4): #or volFlow_rate >= flow_plat - 10):
# 			    flow_flag = 1
# 			    motor_1.ChangeDutyCycle(pump_pressure)
# 			if(flow_flag == 1 and volFlow_rate <= flow_plat):
# 			    motor_1.ChangeDutyCycle(pump_pressure)
			if(volFlow_rate >= flow_plat and flow_flag ==7):
			    flow_flag = 2
			if(volume == -9999):
			    SDP_flag = 1;
			if(sending_time > 0.02):
			    packet_inhalation =('A@' + str(round(indiff,2)) + ','+str(round(volFlow_rate,2)) + ',' + str(int(volume)) + ',' + str(trigger) + '#')
			    try:
				ser.write(packet_inhalation)
			    except:
				print('BT error send Inhalation')
#			    print('packet size isssssssssss')
#			    print(len(packet_inhalation))
#			    print('packet conatins ')
#			    print(packet_inhalation)
			    send_last_time= time()
#			if(volFlow_rate < flow_plat -3 and pump_pressure_now < pump_pressure):
#			    pump_pressure_now = pump_pressure_now + 4
#			    print('in 1st loop')
#			    motor_1.ChangeDutyCycle(pump_pressure_now)
#			elif(volFlow_rate > flow_plat + 3 and pump_pressure_now > pump_pressure):
#			    pump_pressure_now = pump_pressure_now -4
#			    print('in 2nd loop')
#			    motor_1.ChangeDutyCycle(pump_pressure_now)
			if(flow_flag == 3):
			    if(volFlow_rate <= flow_plat -5 and pump_pressure_now <= P_plat):
		#	   # if(pump_flag == 0):
			      #      pump_pressure_now  = pump_pressure_now + 5
			     #   else:
				pump_pressure_now = pump_pressure_now + 1
	#		    print('the pump_pressure now is')
	#		    print(pump_pressure_now)
	#		    print('pump pressure is')
	#		    print(pump_pressure)
			        motor_1.ChangeDutyCycle(pump_pressure_now)
			    if(volFlow_rate >= flow_plat and pump_pressure_now >= P_plat/2): #and pump_pressure_now >( pump_pressure /2)):
			        pump_pressure_now = pump_pressure_now - 1
			        motor_1.ChangeDutyCycle(pump_pressure_now)
			      #  pump_flag = 1
			sending_time=t3 - send_last_time
#			print('flow is')
#			print(volFlow_rate)
#			print(time_elapsed_inhale)
#			print(inhale_time)
			VTi_volume = max(volume,VTi_volume)
			peak_insp_pressure = max(peak_insp_pressure, indiff)
			volume_peak_inhale=max(volume_peak_inhale,volFlow_rate)
		    if(indiff >= (PIP+1)):
			pressure_count = pressure_count +1
			if(pressure_count >= 1):
		            try:
		  	        ser.write('ACK07')
			    except:
		      	        print('unable to send')
		    else:
			pressure_count = 0
			try:
			    ser.write('ACK57')
			except:
			    print('unable to send')
		    if(indiff <= (P_plat)*0.5):
			pressure_low_count = pressure_low_count +1
			if(pressure_low_count >= 3):
		            try:
		  	        ser.write('ACK62')
			    except:
		      	        print('unable to send')
		    else:
			pressure_low_count = 0
			try:
			    ser.write('ACK72')
			except:
			    print('unable to send')
                    flag=1
		    loop = 0
#		    print('volume is')
#		    print(VTi_volume)
		    motor_1.ChangeDutyCycle(pump_pressure)
	#	    sleep(0.01)
#		    motor_1.ChangeDutyCycle(peep_open)
		    sol.ChangeDutyCycle(0)
		 #   GPIO.output(35, GPIO.HIGH) # HIGH level triggered relay
		    try:
		        Pmean= sum(Pmean_array)/len(Pmean_array)
		    except:
			print("zero exception error")
		    print('P_plat is')
		    print(int(peak_insp_pressure))
		    print(P_plat)
                    RR.append(RR_time)
		    peak_insp_pressure = int(peak_insp_pressure)
		    if(Pmean > P_plat and patient_status == 1 and ABP_flag == 0 and Pmean < PIP  and pump_pressure >= 1 and pump_pressure <= 96):
			pump_pressure = pump_pressure - 1
			print('-1')
		    elif(Pmean < P_plat and patient_status == 1 and ABP_flag == 0 and Pmean < PIP and  pump_pressure >= 0 and pump_pressure <= 95):
			pump_pressure = pump_pressure + 1
			print('+1')
#		    print('volume is')
#		    print(volume*0.4)
#		    print('Vpeak Inspiratory')
#		    print(volume_peak_inhale)
		    MVi_array.append(VTi_volume)
#		    print('trigger is')
#		    print(trigger)
#		    print('Pmean is')
#		    print(Pmean)
		    GPIO.output(37, GPIO.LOW)
		if(flag == 1):
		    patient_set = 0
		    if((leak_percentage >= 90)  and patient_set == 0 ):
			buzzer(5,1)
			patient_set = 1
			patient_status = 0
		        print('disconnection')
#			trigger = '0'
			try:
			    ser.write('ACK05')
			except:
			    print('not able to send')
		#        GPIO.output(22, GPIO.HIGH)
		    if((leak_percentage < 90) and patient_set == 0):
			buzzer(5,0)
			patient_set = 1
			patient_status = 1
#			trigger = '1'
			try:
			    ser.write('ACK06')
			except:
			    print('not able to send')
                    if(len(RR) >= 5):
                        BPM = ( RR[1] + RR[2] + RR[3] + RR[4]) /4
                        BPM = ceil(60/BPM)
			if(BPM > 70):
			    try:
				ser.write('ACK09')
			    except:
				print('unable to send')
			else:
			    try:
				ser.write('ACK59')
			    except:
				print('unable to send')
 #                      print('BPM is')
  #                     print(BPM)
                        RR.pop(0)
		    if(len(MVi_array) >= 5):
		        MVi= (MVi_array[1] + MVi_array[2] + MVi_array[3] + MVi_array[4])/4
			MVi = MVi * BPM
#		        print('MVi value is')
#		        print(MVi)
		        MVi_array.pop(0)
		        packet_end_inhalation = ('B@' + str(round(peak_insp_pressure,2)) + "," + str(int(VTi_volume - (VTi_volume*leak_percentage)/200)) + "," + str(round(volume_peak_inhale,2)) + "," + str(int(Pmean)) + "," + str(round((MVi/1000),2)) + "," + str(round(patient_trigger_flow,2)) + "," + str(round(time_elapsed_inhale,2)) + "," + str(round(indiff,2)) + "," + str('0') + "," + str(round(rise_time,2)) +'#')
#		        print('packet length is')
#		        print(packet_end_inhalation)
		        try:
		            ser.write(packet_end_inhalation)
		        except:
			    print('BT Errror in end of inhalation loop')
                if(flag==1):
		    time_error_last = time()
		    time_elap = 0
		    q= time()
		    inhale_loop = "0"
		    buzzer(7,0)
		    volume = 0
		    sending_time = 200
		    send_last_time = 0
		    volume_peak_exhale = 0
		    while(time_elap < 0.4):
			w=time()
			time_elap=w-q
                        indiff = self.ABP_pressure()
			ratio = peep_val/P_plat
#                         if(ratio > 0.8 and time_elap >= 0.005 and toggle_switch == '1'):
#                     #        GPIO.output(35, GPIO.LOW)
#                             sleep(0.05)
#         #                    motor_1.ChangeDutyCycle(peep)
#                         elif(ratio >= 0.15 and ratio <= 0.4 and time_elap >= 0.3 and P_plat < 28 and toggle_switch == '1'):
#                    #         GPIO.output(35, GPIO.LOW)
#                             sleep(0.05)
#          #                   motor_1.ChangeDutyCycle(peep)
#                         elif(ratio >= 0.15 and ratio <= 0.4 and time_elap >= 0.3 and P_plat >=28 and toggle_switch == '1'):
#                     #        GPIO.output(35, GPIO.LOW)
#                             sleep(0.05)
#           #                  motor_1.ChangeDutyCycle(peep)
#                         elif(ratio > 0.4 and ratio <= 0.8 and time_elap >= 0.001 and toggle_switch == '1'):
#                    #         GPIO.output(35, GPIO.LOW)
#                             sleep(0.05)
#            #                 motor_1.ChangeDutyCycle(peep)
#                         elif(ratio <0.15 and ratio >=0.00 and time_elap >= 0.3 and toggle_switch == '1'):
#                         #    GPIO.output(35, GPIO.LOW)
#                             sleep(0.05)
#             #                motor_1.ChangeDutyCycle(peep)
			if(ABP_flag == 0):
			    indiff = self.ABP_pressure()
# 			if(indiff <= (peep + 1)):
# 			    print('turning off the relay')
			diff = self.SDP_pressure()
			if(SDP_flag == 0):
			    volFlow_rate = self.rate()
			    volume=self.Flow()
			if(diff == 700 or diff == -700 or SDP_flag == 1):
			    try:
				f = open("/home/pi/AgVa_5.0/mode.txt","w")
				f.write(str('11'))
				f.close()
				try:
				    ser.write('ACK30')
				except:
				    print('unable to send data')
			    except:
				print('unable to write mode')
			packet_exhalation= ('C@' + str(round(indiff,2)) + ',' + str(round(volFlow_rate,2)) +',' + str(int(volume)) + ',' + str(int((TITOT)*100)) + '#')
			if(sending_time > 0.02):
			    try:
				ser.write(packet_exhalation)
			    except:
				print('BT exhalation error')
			    send_last_time = time()
			sending_time = w- send_last_time
#			print('packet exhalation size  is')
#			print(volFlow_rate)
			peep_val_send = indiff
			volume_peak_exhale= min(volume_peak_exhale, volFlow_rate)
#			print('packet_exhal;ation conatins is')
#			print(packet_exhalation)
                    #motor_2.ChangeDutyCycle(0)
                    new_time=time()
                    time_elapsed_exhale=0.0
                    diff=self.SDP_pressure()
# 		    if(toggle_switch == "1" and volFlow_rate <= (volume_peak_exhale * 0.05) and indiff <= peep_val):
#                         sol.ChangeDutyCycle(100)
# 			sleep(0.3)
		    start_time=current_time
		    sending_time=200
		    time_elapsed_exhale_flow = 0
		    sol_flag  = 0
		    send_last_time = 0
                    while(( diff <= -10 and  time_elapsed_exhale <= 5.0) or (SDP_flag ==1 and time_elapsed_exhale<(cycle_time-inhale_time-0.5))):
                        time_elapsed_exhale=time()-new_time
			if(ABP_flag == 0):
			    indiff = self.ABP_pressure()
#		        if(toggle_switch == "1" and sol_flag == 0 and  volFlow_rate <= (volume_peak_exhale * 0.05) and indiff <= peep_val):
 #                           sol.ChangeDutyCycle(0)
#			    sol_flag  = 1
#			    sleep(0.3)
			if(SDP_flag == 0):
           		    volume=self.Flow()
			    volFlow_rate=self.rate()
			    diff = self.SDP_pressure()
# 			if(indiff <= (peep + 1)):
# 			    GPIO.output(35, GPIO.LOW)
# 			    print('turning off the relay')
			volume_peak_exhale=min(volume_peak_exhale,volFlow_rate)
 			if(volFlow_rate < -5):
 			    time_elapsed_exhale_flow = time_elapsed_exhale
			packet_exhalation = ('C@' + str(round(indiff,2)) + ',' + str(round(volFlow_rate,2)) + ',' + str(int(volume)) + ',' + str(int((TITOT)*100))+'#')
			if(sending_time > 0.02):
			    try:
				ser.write(packet_exhalation)
			    except:
				print('BT Error sending in Exhalation conditional')
			    send_last_time = time()
			sending_time =time() - send_last_time 
                        start_time=current_time
                    #    print(start_time)
                    flag=0 
		    loop = 1
		    inhale_loop = "1"
		    exhale_time_last = time()
		    time_elapsed_exhale = time_elapsed_exhale + 0.5
#		    print(volFlow_rate)
#		    print(time_elapsed_inhale /( time_elapsed_inhale + time_elapsed_exhale))
		    TITOT=(time_elapsed_inhale/(time_elapsed_inhale + time_elapsed_exhale + TOT_last))
#		    print('peak volume flow rate exhale')
#		    print(volume_peak_exhale)
		    if((peep_val_send) <= peep_val and patient_status == 1 and peep < 98 and peep_open < 98):
		#	if(toggle_switch == "1"):
			peep = peep + 1
		#	else:
		#	    peep_open = peep_open + 1
		    if((peep_val_send) > peep_val and patient_status == 1 and peep > 0 and peep_open > 0):
		#	if(toggle_switch == "1"):
			peep = peep - 1
		#	else:
		#	    peep_open = peep_open - 1
		    if(int(peep_val_send) > peep_val+2):
			peep_count = peep_count + 1
			if(leak_percentage > 20 and trigger == '0'):
			    ser.write("ACK39")
			if(peep_count >=3 and peep_set == 0):
			    buzzer(4,1)
			    peep_set = 1
			    try:
			        ser.write('ACK08')
			    except:
			        print('unable to send')
		    else:
			if(peep_set == 1):
			    buzzer(4,0)
			    peep_set = 0
			peep_count = 0
			ser.write("ACK49")
			try:
			    ser.write('ACK58')
			except:
			    print('unable to send')
		    FiO2 = self.FiO2()
                    self.power()
		    print('FiO2 is')
		    print(FiO2)
		    if(toggle_switch == '1'):
		        time_elapsed_exhale = time_elapsed_exhale + 0.4
			time_elapsed_exhale_flow = time_elapsed_exhale_flow + 0.4
		    else:
			time_elapsed_exhale = time_elapsed_exhale + 0.1
			time_elapsed_exhale_flow = time_elapsed_exhale_flow + 0.1
		    MVe_array.append(volume)
		    if(len(MVe_array)>=5):
		        MVe= (MVe_array[1] + MVe_array[2] + MVe_array[3] + MVe_array[4])/4
			MVe = MVe * BPM
		        MVe_array.pop(0)
		        try:
			    leak_percentage = ((MVi - MVe)/MVi)*100
			    if(leak_percentage <= -1):
			        leak_percentage = 0
		        except:
			    leak_percentage = 0
#		        print('MVe is')
#		        print(MVe)
		   # packet_end_exhalation = 'ABC'
		        packet_end_exhalation = ('D@' + str(round(peep_val_send,2)) + ',' + str(int(BPM)) + ',' + str(FiO2) + ',' + str(round(volume_peak_exhale,2)*-1)  + "," + str(round((MVe/1000),2)) + "," + str(leak_percentage) + "," + str(trigflow) + "," + str(volume) + "," + str(time_elapsed_exhale_flow) + '#')
#			print('string issssssssssssssssssssssssssssssssssssss')
#			print(packet_end_exhalation)
		        try:
	                    ser.write(packet_end_exhalation)
		        except:
			    print('BT send error in end of exhalation')
#		    try:
#		        with open('/home/pi/AgVa_5.0/data.csv', mode='a') as data:
 #  		            data = csv.writer(data, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
#		            now = datetime.datetime.now()
#		            data.writerow([str(now),str(self.measure_temp()),str(Pmean),str(volume),str(volume_peak_inhale),str(trigger),str(BPM),str(MVi),str(ratio), str(volume_peak_exhale),str(MVe)])
#		    except:
#			print('data file open error')
        except KeyboardInterrupt:
            return
##############################################################################################################

    def NIV_BIPAP(self):
        global starting_flag,change_setting,exp_press_array,volume_flag,peep_first,initial_pump_pressure,trigflow, exp_press,exp_press_list,peep_average,peep,peep_open,time_elapsed_exhale_flow,patient_trigger_flow,rise_time, rise_flag ,pressure_low_count,BIPAP_backup_flag,back_backup,inhale_loop,toggle_switch,loop,peep,patient_set,ABP_flag,exhale_time_last,TOT_last,TITOT,pressure_count,peep_count, SDP_flag,flow_plat,indiff,patient_status,pump_pressure,pump_pressure_low,pump_pressure_high,ratio_set,time_error_set,thread_mode_status,time_error, time_error_last,ABP_flag,P_plat_value,prev_mode,data,ratio,RR_time,indiff,peep_val, peep_hole, RR, BPM, MVi_array, MVi,volume, volume_peak_inhale, trigger, max_value, volume_peak_exhale, MVe_array, MVe,clock_t2
        thread_mode_status = True
	time_elapsed_exhale_flow = 0
	rise_time = 0
	starting_flag = 0
	volume_flag = 0
	patient_trigger_flow = 0
	rise_flag = '0'
	pressure_low_count = 0
	patient_set = 0
	exhale_time_last = time()
	TOT_last = time()
	peep_count = 0
	inhale_loop = "0"
	flag = 0
	loop = 1
	BIPAP_backup_flag = 0
	GPIO.output(35, GPIO.LOW)
	VTi_volume = 0
	GPIO.output(33, GPIO.LOW)
	leak_percentage = 0
	exp_press_list =[]
	del MVi_array[:]
	del MVe_array[:]
	pressure_count = 0
	peep_set = 0
	GPIO.output(33, GPIO.LOW)
	ratio_set = 0
	time_error_set = 0
	trigger = '1'
	sol.ChangeDutyCycle(100)
        start_time=time()
	current_time = time()
	time_error_last = time()
	sending_time = 0.2
	temp_peep = peep
#	temp_pump_pressure_flag = 1
        try:
            while True:
		prev_mode = 32
		ABP_flag = 0
#
		data= self.read_data()
#		temp_peep = peep
		print("!!!!!!" + str(P_plat) )
		print("!!!!!!!!!!" + str(initial_pump_pressure) + "!!!!!!!!!!!" + str(pump_pressure))
		if(data == 1 or (current_time-start_time) >= 10):
		    if(current_time-start_time >= 10):
		        try:
			    GPIO.output(33, GPIO.LOW)
			    f = open("/home/pi/AgVa_5.0/backup.txt","w")
			    f.write("320")
			    f.close()
			    back_backup = 1
			    f = open("/home/pi/AgVa_5.0/mode.txt","w")
			    f.write(str("350"))
			    f.close()
			    change_setting = '1'
			    BIPAP_backup_flag = 1
			except:
			    print("unable to fall back to PC_IMV mode")
			try:
			    ser.write("ACK38")
			except:
			    print("unable to send ACK38")
		    break;
#		print(inhale_time)
#		print(BPM)
#		print(PIP)
#		PIP = P_plat +1
		exp_press_array.append(indiff)
	        inhale_time = 3.0
		motor_1.ChangeDutyCycle(peep)
#		trig_flow = trigflow + 5
#		print(VTi_max)
#		print('peep in main loop is')
#		print(peep)
#		print(peep_val)
#		print(P_plat)
#		print(P_plat_value)
#		print('here the pump pressure is')
#		print(pump_pressure)
	#	GPIO.output(22, GPIO.LOW)
		volFlow_rate = self.rate()
		indiff= self.ABP_pressure()
		if(indiff == -8888):
			ABP_flag = 1
			indiff = 0
		packet_exhalation = ('C@' + str(round(indiff,2)) + ',' + str(round(volFlow_rate,2)) + ',' + str(int(volume)) + ',' + str(int((TITOT)*100))+'#')
		if(sending_time > 0.02):
		    try:
			ser.write(packet_exhalation)
		    except:
			print('BT Error sending in Exhalation conditional')
		    send_last_time = time()
		sending_time =time() - send_last_time 
#		print('pump_pressure is')
#		print(pump_pressure)
		trigger_comp = trigflow
                flow=self.rate()
		if(leak_percentage < 50):
		    trigger_comp = trigflow + (leak_percentage/5)
		else:
		    trigger_comp = trigflow
#		print('before flow')
#		print(flow)
	#	flow = flow - peep_hole[int(peep_val)]
#		print('after flow')
#		print(flow)
                current_time=time()
            #    volume=0
		TOT_last = current_time - exhale_time_last
#		peep_val_send = indiff
		SDP_flag = 0
		time_error = time() - time_error_last
		loop = 1
                if(flow>trigger_comp or starting_flag == 0):
		    RR_time=current_time-start_time
		    starting_flag = 1
		    pump_pressure_now = pump_pressure
		    patient_trigger_flow = flow
		    peep_val_send = temp_peep
		    if(len(exp_press_array) > 10):
			peep_val_send = exp_press_array[len(exp_press_array) -10]
		    del exp_press_array[:]
		    inhale_loop = "1"
		    volume = 0
		    indiff = self.ABP_pressure()
		    sol.ChangeDutyCycle(100)
		    flow_flag = 0
#		    if((pump_pressure /2) < peep):
#			motor_1.ChangeDutyCycle(peep + 2)
#			pump_pressure_now = peep +2
#		    else:
#		    print('indiff right now is')
#		    print(indiff)
#		    if(indiff <= P_plat/2):
                    motor_1.ChangeDutyCycle(pump_pressure)
		    if(P_plat <= 22):
			motor_1.ChangeDutyCycle(pump_pressure)
#		    if(P_plat <= 17):
#			motor_1.ChangeDutyCycle(65)
#		        flow_flag = 0
#		    else:
#			motor_1.ChangeDutyCycle(pump_pressure)
#			flow_flag = 1
#			pump_pressure_now = pump_pressure /2
		    if(time_error < 0.7):
			time_error_set = 1
#			try:
#			    ser.write('ACK24')
#			except:
#			    print('unable to send data')
#		    if(time_error > 0.7):
#			time_error_set = 0
#			try:
#			    ser.write('ACK24')
#			except:
#			    print('unable to send data')
		    if(float(ratio) <= 0.25):
			ratio_set = 1
#			try:
#			    ser.write('ACK12')
#			except:
#			    print('unable to send data')
		    if(float(ratio) >= 0.75):
			ratio_set = 1
#			try:
#			    ser.write('ACK13')
#			except:
#			    print('unable to send data')
		    if(float(ratio) >0.25 and float(ratio) < 0.75):
			ratio_set = 0
#			try:
#			    ser.write('ACK22')
#			except:
#			    print('unable to send data')
#		    if(RR_time > cycle_time):
#			trigger='0'
#		    else:
		    trigger='1'
                 #   indiff= self.ABP_pressure()
                  #  print(indiff)
                    time_elapsed_inhale=0
                    indiff = 0
		    Pmean=0
		    del Pmean_array[:]
		    clock_t2= time()
#		    print('now it is')
#		    print(clock_t2)
		    volume=0
		    volume_peak_inhale=0
		    sending_time = 0.2
		    send_last_time=0
		    volFlow_rate = self.rate()
		    VTi_volume=0
		    pump_flag = 0
		   # flow_flag = 0
		    current_time = time()
		    peak_insp_pressure = 0
		    pump_pressure_now = pump_pressure /2
#		    flow_zero_intialize = time() # clock starts to detect zero flow
#		    flow_zero_flag = 0
		    indiff = self.ABP_pressure()
		    peak_flow = volFlow_rate
		    rise_flag = '0'
                    while((((peak_flow*0.25 <= volFlow_rate) and time_elapsed_inhale <= inhale_time) or time_elapsed_inhale <= 0.3) and indiff < P_plat * 1.3 ):
#                        motor_1.ChangeDutyCycle(pump_pressure*2)
#                        print("initial pump press is ::::" + str(temp_pump_pressure))
                        t3 = time()
			GPIO.output(37, GPIO.HIGH)
			if(time_elapsed_inhale <0.3):
			    if(pump_pressure < 25):
				motor_1.ChangeDutyCycle(pump_pressure*1.5)
			    else:
				motor_1.ChangeDutyCycle(pump_pressure)
			else:
		            motor_1.ChangeDutyCycle(pump_pressure)
#			    flow_zero_elapsed = time() - flow_zero_initialize
#			    if(flow_zero_elapsed >= )
#			    flow_zero_elapsed = time() - flow_zero_initialize
			if(ABP_flag == 0):
                            indiff= self.ABP_pressure()
                        time_elapsed_inhale= t3-current_time
			if(indiff >= P_plat * 0.8 and rise_flag == '0'):
				rise_time = time_elapsed_inhale
				rise_flag = '1'
                   #     print(indiff)
			Pmean_array.append(indiff)
			if(SDP_flag == 0):
			    volume=self.Flow()
			    volFlow_rate=self.rate()
			    if(volFlow_rate >= peak_flow):
				peak_flow = volFlow_rate
			if((indiff >= P_plat*motor_factor or time_elapsed_inhale >= 0.5) and flow_flag == 0): #(time_elapsed_inhale > 0.3 and flow_flag == 0) or indiff >= 10): #or indiff >= P_plat - 4): #or volFlow_rate >= flow_plat - 10):
			    flow_flag = 1
			    motor_1.ChangeDutyCycle(pump_pressure)
#			    print('CMHO reacged')
#			    print(indiff)
# 			if(P_plat <= 17 and (indiff >= P_plat/2 or time_elapsed_inhale >= 0.2) and flow_flag == 0): #(time_elapsed_inhale > 0.3 and flow_flag == 0) or indiff >= 10): #or indiff >= P_plat - 4): #or volFlow_rate >= flow_plat - 10):
# 			    flow_flag = 1
# 			    motor_1.ChangeDutyCycle(pump_pressure)
# 			if(flow_flag == 1 and volFlow_rate <= flow_plat):
# 			    motor_1.ChangeDutyCycle(pump_pressure)
			if(volFlow_rate >= flow_plat and flow_flag ==7):
			    flow_flag = 2
			if(volume == -9999):
			    SDP_flag = 1;
			if(sending_time > 0.02):
			    packet_inhalation =('A@' + str(round(indiff,2)) + ','+str(round(volFlow_rate,2)) + ',' + str(int(volume)) + ',' + str(trigger) + '#')
			    try:
				ser.write(packet_inhalation)
			    except:
				print('BT error send Inhalation')
#			    print('packet size isssssssssss')
#			    print(len(packet_inhalation))
#			    print('packet conatins ')
#			    print(packet_inhalation)
			    send_last_time= time()
#			if(volFlow_rate < flow_plat -3 and pump_pressure_now < pump_pressure):
#			    pump_pressure_now = pump_pressure_now + 4
#			    print('in 1st loop')
#			    motor_1.ChangeDutyCycle(pump_pressure_now)
#			elif(volFlow_rate > flow_plat + 3 and pump_pressure_now > pump_pressure):
#			    pump_pressure_now = pump_pressure_now -4
#			    print('in 2nd loop')
#			    motor_1.ChangeDutyCycle(pump_pressure_now)
			if(flow_flag == 3):
			    if(volFlow_rate <= flow_plat -5 and pump_pressure_now <= P_plat):
		#	   # if(pump_flag == 0):
			      #      pump_pressure_now  = pump_pressure_now + 5
			     #   else:
				pump_pressure_now = pump_pressure_now + 1
	#		    print('the pump_pressure now is')
	#		    print(pump_pressure_now)
	#		    print('pump pressure is')
	#		    print(pump_pressure)
			        motor_1.ChangeDutyCycle(pump_pressure_now)
			    if(volFlow_rate >= flow_plat and pump_pressure_now >= P_plat/2): #and pump_pressure_now >( pump_pressure /2)):
			        pump_pressure_now = pump_pressure_now - 1
			        motor_1.ChangeDutyCycle(pump_pressure_now)
			      #  pump_flag = 1
			sending_time=t3 - send_last_time
#			print('flow is')
#			print(volFlow_rate)
#			print(time_elapsed_inhale)
#			print(inhale_time)
			VTi_volume = max(volume,VTi_volume)
			peak_insp_pressure = max(peak_insp_pressure, indiff)
			volume_peak_inhale=max(volume_peak_inhale,volFlow_rate)
		    if(indiff >= (PIP+2)):
			pressure_count = pressure_count +1
			if(pressure_count >= 5):
		            try:
		  	        ser.write('ACK07')
			    except:
		      	        print('unable to send')
		    else:
			pressure_count = 0
			try:
			    ser.write('ACK57')
			except:
			    print('unable to send')
		    if(indiff <= (P_plat)*0.5):
			pressure_low_count = pressure_low_count +1
			if(pressure_low_count >= 3):
		            try:
		  	        ser.write('ACK62')
			    except:
		      	        print('unable to send')
		    else:
			pressure_low_count = 0
			try:
			    ser.write('ACK72')
			except:
			    print('unable to send')
                    flag=1
		    loop = 0
#		    print('volume is')
#		    print(VTi_volume)
		    motor_1.ChangeDutyCycle(peep)
	#	    sleep(0.01)
		    if(toggle_switch == "0"):
			print("got in the first statement")
			GPIO.output(33, GPIO.HIGH)
		        motor_1.ChangeDutyCycle(peep)
			sol.ChangeDutyCycle(0)
		    if(toggle_switch == "1"):
			print("got in the 2nd statement")
		        motor_1.ChangeDutyCycle(peep)
			sol.ChangeDutyCycle(0)
		 #   GPIO.output(35, GPIO.HIGH) # HIGH level triggered relay
		    try:
		        Pmean= sum(Pmean_array)/len(Pmean_array)
		    except:
			print("zero exception error")
		    print('P_plat is')
		    print(int(peak_insp_pressure))
		    print(P_plat)
                    RR.append(RR_time)
		    peak_insp_pressure = int(peak_insp_pressure)
		    if(Pmean > P_plat and patient_status == 1 and ABP_flag == 0 and Pmean < PIP and pump_pressure > 0.5 * initial_pump_pressure  and pump_pressure >= 1 and pump_pressure <= 96):
			pump_pressure = pump_pressure - 1
			print('-1')
		    elif(Pmean < P_plat and patient_status == 1 and ABP_flag == 0 and Pmean < PIP and pump_pressure < 1.5 * initial_pump_pressure and  pump_pressure >= 0 and pump_pressure <= 95):
			pump_pressure = pump_pressure + 2
			print('+1')
#		    print('volume is')
#		    print(volume*0.4)
#		    print('Vpeak Inspiratory')
#		    print(volume_peak_inhale)
		    MVi_array.append(VTi_volume)
#		    print('trigger is')
#		    print(trigger)
#		    print('Pmean is')
#		    print(Pmean)
		    GPIO.output(37, GPIO.LOW)
# 		if(leak_percentage > 50):
# 			leak_percentage = 50.0
		if(flag == 1):
		    patient_set = 0
		    if((leak_percentage > 99)  and patient_set == 0 ):
			buzzer(5,1)
			patient_set = 1
			patient_status = 0
		        print('disconnection')
#			trigger = '0'
			try:
			    ser.write('ACK05')
			except:
			    print('not able to send')
		#        GPIO.output(22, GPIO.HIGH)
		    if((leak_percentage <= 99) and patient_set == 0):
			buzzer(5,0)
			patient_set = 1
			patient_status = 1
#			trigger = '1'
			try:
			    ser.write('ACK06')
			except:
			    print('not able to send')
                    if(len(RR) >= 5):
                        BPM = ( RR[1] + RR[2] + RR[3] + RR[4]) /4
                        BPM = ceil(60/BPM)
			if(BPM > 70):
			    try:
				ser.write('ACK09')
			    except:
				print('unable to send')
			else:
			    try:
				ser.write('ACK59')
			    except:
				print('unable to send')
 #                      print('BPM is')
  #                     print(BPM)
                        RR.pop(0)
		    if(len(MVi_array) >= 5):
		        MVi= (MVi_array[1] + MVi_array[2] + MVi_array[3] + MVi_array[4])/4
			MVi = MVi * BPM
#		        print('MVi value is')
#		        print(MVi)
		        MVi_array.pop(0)
		        packet_end_inhalation = ('B@' + str(round(peak_insp_pressure,2)) + "," + str(int(VTi_volume - (VTi_volume*leak_percentage)/200)) + "," + str(round(volume_peak_inhale,2)) + "," + str(int(Pmean)) + "," + str(round((MVi/1000),2)) + "," + str(round(patient_trigger_flow,2)) + "," + str(round(time_elapsed_inhale,2)) + "," + str(round(indiff,2)) + "," + str('0') + "," + str(round(rise_time,2)) +'#')
#		        print('packet length is')
#		        print(packet_end_inhalation)
		        try:
		            ser.write(packet_end_inhalation)
		        except:
			    print('BT Errror in end of inhalation loop')
                if(flag==1):
		    del exp_press_list[:]
		    time_error_last = time()
		    time_elap = 0
		    q= time()
		    inhale_loop = "0"
		    buzzer(7,0)
		    volume = 0
		    sol_flag = 0
		    sending_time = 200
		    send_last_time = 0
		    volume_peak_exhale = 0
		    while(time_elap < 0.3 or (toggle_switch == "1" and time_elap < 0.4)):
			w=time()
			time_elap=w-q
			exp_press_array.append(indiff)
                        indiff = self.ABP_pressure()
#			exp_press_list.append(indiff)
			ratio = 1.0
                        if(ratio > 0.8 and time_elap >= 0.005 and toggle_switch == '1'):
                    #        GPIO.output(35, GPIO.LOW)
                            sleep(0.05)
        #                    motor_1.ChangeDutyCycle(peep)
                        elif(ratio >= 0.15 and ratio <= 0.4 and time_elap >= 0.3 and P_plat < 28 and toggle_switch == '1'):
                   #         GPIO.output(35, GPIO.LOW)
                            sleep(0.05)
         #                   motor_1.ChangeDutyCycle(peep)
                        elif(ratio >= 0.15 and ratio <= 0.4 and time_elap >= 0.3 and P_plat >=28 and toggle_switch == '1'):
                    #        GPIO.output(35, GPIO.LOW)
                            sleep(0.05)
          #                  motor_1.ChangeDutyCycle(peep)
                        elif(ratio > 0.4 and ratio <= 0.8 and time_elap >= 0.001 and toggle_switch == '1'):
                   #         GPIO.output(35, GPIO.LOW)
                            sleep(0.05)
           #                 motor_1.ChangeDutyCycle(peep)
                        elif(ratio <0.15 and ratio >=0.00 and time_elap >= 0.3 and toggle_switch == '1'):
                        #    GPIO.output(35, GPIO.LOW)
                            sleep(0.05)
            #                motor_1.ChangeDutyCycle(peep)
			if(ABP_flag == 0):
			    indiff = self.ABP_pressure()
# 			if(indiff <= (peep + 1)):
# 			    print('turning off the relay')
			diff = self.SDP_pressure()
			if(SDP_flag == 0):
			    volFlow_rate = self.rate()
			    volume=self.Flow()
			if(diff == 700 or diff == -700 or SDP_flag == 1):
			    try:
				f = open("/home/pi/AgVa_5.0/mode.txt","w")
				f.write(str('11'))
				f.close()
				try:
				    ser.write('ACK30')
				except:
				    print('unable to send data')
			    except:
				print('unable to write mode')
# 		        if(indiff <= peep_val-2 and sol_flag == 0):
#                             sol.ChangeDutyCycle(100)
# 			    sol_flag = 1
# 			    holding_now = time()
# 			    holding_time = 0.3
# 			    peep_check = peep_val - trigflow - 1 #- 2.0
# 			    if(peep_check < 0):
# 			        peep_check = 0
# 			    while( time() - holding_now <= holding_time and time_elap <0.3):
# 				time_elap = time() - q
# 				indiff = self.ABP_pressure()
# 				GPIO.output(33, GPIO.LOW)
# 				if(volFlow_rate >= trigflow+20):
# 				    break;
# 			        volFlow_rate = self.rate()
# 			        volume=self.Flow()
# 				packet_exhalation= ('C@' + str(round(indiff,2)) + ',' + str(round(0,2)) +',' + str(int(volume)) + ',' + str(int((TITOT)*100)) + '#')
# 				if( 1 == 1):
# 			    	    try:
# 					ser.write(packet_exhalation)
# 			    	    except:
# 					print('BT exhalation error')
# 			    	    send_last_time = time()
# 				sending_time = w- send_last_time
# 				sleep(0.02)
			packet_exhalation= ('C@' + str(round(indiff,2)) + ',' + str(round(volFlow_rate,2)) +',' + str(int(volume)) + ',' + str(int((TITOT)*100)) + '#')
			if(sending_time > 0.02):
			    try:
				ser.write(packet_exhalation)
			    except:
				print('BT exhalation error')
			    send_last_time = time()
			sending_time = w- send_last_time
#			print('packet exhalation size  is')
#			print(volFlow_rate)
#			peep_val_send = indiff
			volume_peak_exhale= min(volume_peak_exhale, volFlow_rate)
#			print('packet_exhal;ation conatins is')
			temp_flag = 1
                    #motor_2.ChangeDutyCycle(0)
                    new_time=time()
                    time_elapsed_exhale=0.0
		    temp_peep = self.ABP_pressure()
                    diff=self.SDP_pressure()
# 		    if(toggle_switch == "1" and volFlow_rate <= (volume_peak_exhale * 0.05) and indiff <= peep_val):
#                         sol.ChangeDutyCycle(0)
# 			sleep(0.3)
		    start_time=current_time
		    sending_time=200
		    time_elapsed_exhale_flow = 0
		    sol_flag  = 0
		    send_last_time = 0
                    while(( diff <= 0 and  time_elapsed_exhale <= 5.0) or (SDP_flag ==1 and time_elapsed_exhale<(cycle_time-inhale_time-0.5)) or temp_flag == 1):
                        time_elapsed_exhale=time()-new_time
			exp_press_array.append(indiff)
			if(ABP_flag == 0):
			    indiff = self.ABP_pressure()
			exp_press_list.append(indiff)
# 		        if(toggle_switch == "1" and sol_flag == 0 and  volFlow_rate <= (volume_peak_exhale * 0.05) and indiff <= peep_val):
#                             sol.ChangeDutyCycle(0)
# 			    sol_flag  = 1
# 			    sleep(0.3)
			if((sol_flag == 0 and ((indiff <= peep_val+2 and volFlow_rate >= -10) or (indiff <= peep_val-2))) or diff >= 0 or time_elapsed_exhale >= 5.0 ):
			    peep_check = peep_val - trigflow - 1
			    if(peep_check < 0):
			        peep_check = 0
			    if(volFlow_rate >= trigflow):
				break;
			    GPIO.output(33, GPIO.LOW)
			    sol.ChangeDutyCycle(100)
			    holding_now = time()
			    holding_time = 0.3
# 			    peep_check = trigflow - peep_val - 2.0
# 			    if(peep_check < 0):
# 			        peep_check = 0
			    while( time() - holding_now <= holding_time and time_elapsed <= 5.0):
				time_elapsed_exhale = time() - new_time
				indiff = self.ABP_pressure()
#				GPIO.output(33, GPIO.LOW)
				if(volFlow_rate >= trigflow+20):
				    break;
			        volFlow_rate = self.rate()
			        volume=self.Flow()
				packet_exhalation= ('C@' + str(round(indiff,2)) + ',' + str(round(0,2)) +',' + str(int(volume)) + ',' + str(int((TITOT)*100)) + '#')
				if(1 == 1):
			    	    try:
					ser.write(packet_exhalation)
			    	    except:
					print('BT exhalation error')
			    	    send_last_time = time()
			        sending_time = time()- send_last_time
				sleep(0.02)
			    sol_flag = 1
			if(SDP_flag == 0):
           		    volume=self.Flow()
			    volFlow_rate=self.rate()
			    diff = self.SDP_pressure()
 			temp_flag = 0
# 			    GPIO.output(35, GPIO.LOW)
# 			    print('turning off the relay')
			volume_peak_exhale=min(volume_peak_exhale,volFlow_rate)
 			if(volFlow_rate < -5):
 			    time_elapsed_exhale_flow = time_elapsed_exhale
			packet_exhalation = ('C@' + str(round(indiff,2)) + ',' + str(round(volFlow_rate,2)) + ',' + str(int(volume)) + ',' + str(int((TITOT)*100))+'#')
			if(sending_time > 0.02):
			    try:
				ser.write(packet_exhalation)
			    except:
				print('BT Error sending in Exhalation conditional')
			    send_last_time = time()
			sending_time =time() - send_last_time 
                        start_time=current_time
   #                 peep_val_send = (sum(exp_press_list)/len(exp_press_list))
#		    print("AVERAGE PEEP IS   :::::::::::::" + str(sum(exp_press_list)/len(exp_press_list)))
                    flag=0 
		    loop = 1
		    inhale_loop = "1"
		    exhale_time_last = time()
		    time_elapsed_exhale = time_elapsed_exhale + 0.5
#		    print(volFlow_rate)
#		    print(time_elapsed_inhale /( time_elapsed_inhale + time_elapsed_exhale))
		    TITOT=(time_elapsed_inhale/(time_elapsed_inhale + time_elapsed_exhale + TOT_last))
#		    print('peak volume flow rate exhale')
#
		    if(int(peep_val_send) <= peep_val and patient_status == 1 and peep <= peep_first*1.5 and peep < 98 and peep_open < 98):
			if(toggle_switch == "1" ):
			    peep = peep + 2
			else:
			    peep_open = peep_open + 2
		    if(int(peep_val_send) > peep_val and patient_status == 1 and peep >= peep_first*0.5 and peep > 0 and peep_open > 0):
			if(toggle_switch == "1" and peep > 0.8*temp_peep  and indiff >2):
			    peep = peep - 1
			else:
			    peep_open = peep_open - 1
		    if(int(peep_val_send) > peep_val+2):
			peep_count = peep_count + 1
#			if(leak_percentage > 20 and trigger == '0'):
#			    ser.write("ACK39")
			if(peep_count >=6 and peep_set == 0):
			    buzzer(4,1)
			    peep_set = 1
			    try:
			        ser.write('ACK08')
			    except:
			        print('unable to send')
		    else:
			if(peep_set == 1):
			    buzzer(4,0)
			    peep_set = 0
			peep_count = 0
			ser.write("ACK49")
			try:
			    ser.write('ACK58')
			except:
			    print('unable to send')
		    FiO2 = self.FiO2()
                    self.power()
		    print('FiO2 is')
		    print(FiO2)
		    if(toggle_switch == '1'):
		        time_elapsed_exhale = time_elapsed_exhale + 0.4
			time_elapsed_exhale_flow = time_elapsed_exhale_flow + 0.4
		    else:
			time_elapsed_exhale = time_elapsed_exhale + 0.1
			time_elapsed_exhale_flow = time_elapsed_exhale_flow + 0.1
		    MVe_array.append(volume)
		    if(len(MVe_array)>=5):
		        MVe= (MVe_array[1] + MVe_array[2] + MVe_array[3] + MVe_array[4])/4
			MVe = MVe * BPM
		        MVe_array.pop(0)
		        try:
			    leak_percentage = ((MVi - MVe)/MVi)*100
			    if(leak_percentage <= -1):
			        leak_percentage = 0
		        except:
			    leak_percentage = 0
#		        print('MVe is')
#		        print(MVe)
		   # packet_end_exhalation = 'ABC'
		        packet_end_exhalation = ('D@' + str(round(peep_val_send,2)) + ',' + str(int(BPM)) + ',' + str(FiO2) + ',' + str(round(volume_peak_exhale,2)*-1)  + "," + str(round((MVe/1000),2)) + "," + str(leak_percentage) + "," + str(trigflow) + "," + str(volume) + "," + str(time_elapsed_exhale_flow) + '#')
#			print('string issssssssssssssssssssssssssssssssssssss')
#			print(packet_end_exhalation)
		        try:
	                    ser.write(packet_end_exhalation)
		        except:
			    print('BT send error in end of exhalation')
	#	    try:
	#	        with open('/home/pi/AgVa_5.0/data.csv', mode='a') as data:
   	#	            data = csv.writer(data, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
	#	            now = datetime.datetime.now()
	#	            data.writerow([str(now),str(self.measure_temp()),str(Pmean),str(volume),str(volume_peak_inhale),str(trigger),str(BPM),str(MVi),str(ratio), str(volume_peak_exhale),str(MVe)])
	#	    except:
	#		print('data file open error')
        except KeyboardInterrupt:
            return



#------------------------------------------------------
    def AIVENT(self):
        global peep_first,compliance_flag,volume_flag,lock,peep_factor,IH_time, EH_time, IHold, EHold,exp_press_array,exp_flow_array,peep,peep_open,time_elapsed_exhale_flow,patient_trigger_flow,rise_time, rise_flag,inhale_array,motor_factor,pressure_low_count,leak_comp_flag,running_avg, trigflow_comp, comp_flag,volume_comp,inhale_loop,pump_pressure_array,P_plat_array,compliance,compliance_array,toggle_switch,peep_open,loop,peep,peep_pwm,P_plat,P_plat_high,exhale_time_last, TOT_last,TITOT,P_plat_low,ABP_flag,pressure_count,peep_count,volume_count, SDP_flag,patient_set,flow_plat,ratio_set,patient_status,pump_pressure,pump_pressure_low, pump_pressure_high,time_error_set,thread_mode_status,time_error, time_error_last,prev_mode,P_plat_value,peep_hole, peep_val,data,ratio,RR_time,indiff, RR, BPM, MVi_array, MVi,volume, volume_peak_inhale, trigger, max_value, volume_peak_exhale, MVe_array, MVe,clock_t2
        thread_mode_status = True
	peep_factor = 2.0
	lock = 0
	volume_flag = 0
	time_elapsed_exhale_flow = 0
	patient_trigger_flow = 0
	rise_time = 0
	rise_flag = '0'
	pressure_low_count = 0
	del MVi_array[:]
	del MVe_array[:]
	del exp_flow_array[:]
	del exp_flow_array[:]
	trigflow_comp = 0
	comp_flag = '0'
	inhale_array = []
	motor_factor = 0.5
	volume_count = 0
	running_avg = 0.0
	volume_comp = 0
	exhale_time_last = time()
	TOT_last = time()
	peep_count = 0
	ratio_set = 0
	VTi_volume = 0
	leak_percentage = 0
	loop = 1
	inhale_loop = "0"
	GPIO.output(35, GPIO.LOW)
	flag = 0
	del compliance_array[:]
	pressure_count = 0
	volume_set = 0
	compliance_flag = 0
	patient_set = 0
	peep_set = 0
	sol.ChangeDutyCycle(100)
	peep_val_send = 0
	temp_peep = 0
	time_error_set = 0
        start_time=time() - 5000
	time_error_last = time()
	sending_time = 0.2
        try:
            while True:
		prev_mode = 24
#		print('we are in AIVENT mode')
		data= self.read_data()
#		print(data)
		hold_check()
		if(data == 1):
		    print("breaking the cuircuit in hereeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee")
		    break;
#		print(inhale_time)
#		print(BPM)
#		print(PIP)
#		print(VTi_max)
#		print("peep^^^^^^" + str(peep))
#		print(P_plat)
#		print(trigflow)
		indiff= self.ABP_pressure()
		exp_press_array.append(indiff)
		clock_t2 = time()
		volFlow_rate = self.rate()
#		indiff= self.ABP_pressure()
		packet_exhalation = ('C@' + str(round(indiff,2)) + ',' + str(round(volFlow_rate,2)) + ',' + str(int(volume)) + ',' + str(int((TITOT)*100))+'#')
		if(sending_time > 0.02):
		    try:
			ser.write(packet_exhalation)
		    except:
			print('BT Error sending in Exhalation conditional')
		    send_last_time = time()
		sending_time =time() - send_last_time 
#		print("THE TIME TAKEN IS")
#		print(cuurent_time - start_time)
                flow=self.rate()
	        if(indiff <= peep_val +1 and comp_flag == '1' and leak_comp_flag == '1'):
		    trigflow_comp = trigflow + running_avg + 3
		    comp_flag = '0'
		    running_avg = 0
#                flow=self.rate()
#		flow = flow - peep_hole[int(peep_val)]
                current_time=time()
		if(leak_comp_flag == '0'):
		    trigflow_comp = trigflow
           #     volume=0
		TOT_last = current_time - exhale_time_last
	#	peep_val_send = indiff
	        if(comp_flag == '0' and flow > 1):
		    running_avg = (running_avg + flow)/2
		time_error = time() - time_error_last
                if(indiff <= peep_val+2 and flow >= -10):
                    sol.ChangeDutyCycle(100)
		SDP_flag = 0
		loop = 1
#		print("THE TIME TAKEN IS")
#		print(current_time - start_time)
                if(flow >= trigflow  or (current_time-start_time)>cycle_time):
		    sol.ChangeDutyCycle(100)
                    motor_1.ChangeDutyCycle(100)
		    if(VTi_max <= 200):
			motor_1.ChangeDutyCycle(pump_pressure)
		    RR_time=current_time-start_time
		    pump_pressure_now = pump_pressure
		    patient_trigger_flow = flow
		    inhale_loop = "1"
		    GPIO.output(33, GPIO.LOW)
		    del inhale_array[:]
		    leak_comp_flag = leak_comp()
		    volume = 0
		    comp_flag = '1'
		    print(exp_press_array)
		    GPIO.output(35, GPIO.LOW)
 		    peep_val_send = temp_peep
 		    if(len(exp_press_array) > 10):
 			peep_val_send = exp_press_array[len(exp_press_array) - 4]
#		    print("peep is ::::::::::: " + str(peep))
 		    del exp_press_array[:]
		   # GPIO.output(19,GPIO.HIGH)
# 		    sol.ChangeDutyCycle(100)
#                     motor_1.ChangeDutyCycle(100)
# 		    if(P_plat <= 22):
# 			motor_1.ChangeDutyCycle(80)
#		    if(P_plat <= 17):
#			motor_1.ChangeDutyCycle(65)
#			pump_pressure_now = pump_pressure /2
 #                   RR_time=current_time-start_time
		    if(time_error < 0.7):
			time_error_set = 1
#			try:
#			    ser.write('ACK24')
#			except:
#			    print('unable to send data')
		    if(time_error > 0.7):
			time_error_set = 0
#			try:
#			    ser.write('ACK24')
#			except:
#			    print('unable to send data')
		    if(float(ratio) <= 0.25):
			ratio_set = 1
#			try:
#			    ser.write('ACK12')
#			except:
#			    print('unable to send data')
		    if(float(ratio) >= 0.75):
			ratio_set = 1
#			try:
#			    ser.write('ACK13')
#			except:
#			    print('unable to send data')
		    if(float(ratio) >0.25 and float(ratio) < 0.75):
			ratio_set = 0
#			try:
#			    ser.write('ACK22')
#			except:
#			    print('unable to send data')
		    if(RR_time > cycle_time):
			trigger='0'
		    else:
			trigger='1'
                    #motor_2.ChangeDutyCycle(pump_pressure)
                    indiff= self.ABP_pressure()
                  #  print(indiff)
                    time_elapsed_inhale=0
                    indiff = 0
		    volume=self.Flow()
#		    print('the value of volume is')
#		    print(volume)
		    if(volume == -9999):
			SDP_flag = 1
			print('well hrre you gooooooooooooooooooooooooooooooooo')
		    Pmean=0
		    del Pmean_array[:]
		    rise_flag = '0'
		    clock_t2= time()
		    volume_flag = 0
		    volume_reach_time = 0
		    volume=0
		    volume_peak_inhale=0
		    sending_time = 0.2
		    send_last_time=0
		    VTi_volume=0
		    flow_flag = 0
		    volFlow_rate = self.rate()
		    pump_flag = 0
		    pump_pressure_now = pump_pressure/2
		    peak_insp_pressure = 0
		    volume_comp = 0
		    peak_flow = volFlow_rate
                    while(indiff < PIP and time_elapsed_inhale <= inhale_time):
                    #    motor_1.ChangeDutyCycle(pump_pressure)
                        #motor_2.ChangeDutyCycle(pump_pressure)
			GPIO.output(37, GPIO.HIGH)
                        t3 = time()
#			print("inhale loop")
                        indiff= self.ABP_pressure()
                        time_elapsed_inhale= t3-current_time
  #                      volume_reach_time = time_elapsed_inhale
#			print(volume)
#			print(VTi_max)
#			print(indiff)
#			print(PIP)
			Pmean_array.append(indiff)
			if(indiff >= P_plat * 0.8 and rise_flag == '0'):
				rise_time = time_elapsed_inhale
				rise_flag = '1'
			if(SDP_flag == 0):
			    volume=self.Flow()
			    volume_comp = volume
			    volume_comp = volume_comp - ((volume_comp*leak_percentage)/100)
			    print('hello its me')
			    volFlow_rate=self.rate()
			if(volFlow_rate > peak_flow):
			    peak_flow = volFlow_rate
			if(volFlow_rate <  peak_flow * 0.25 and time_elapsed_inhale >= 0.5 and trigger == '1'):
			    break 
			if(sending_time > 0.02):
			    packet_inhalation =('A@' + str(round(indiff,2)) + ','+str(round(volFlow_rate,2)) + ',' + str(int(volume)) + ',' + str(trigger) + '#')
			    try:
				ser.write(packet_inhalation)
			    except:
				print('BT error send Inhalation')
#			    print('packet size isssssssssss')
#			    print(len(packet_inhalation))
#			    print('packet conatins ')
#			    print(packet_inhalation)
			    send_last_time= time()
			sending_time=t3 - send_last_time
#			print('inhale_time is')
			if((indiff >= P_plat*motor_factor or time_elapsed_inhale >= 0.5) and flow_flag == 0): #(time_elapsed_inhale > 0.3 and flow_flag == 0) or indiff >= 10): #or indiff >= P_plat - 4): #or volFlow_rate >= flow_plat - 10):
			    flow_flag = 1
			    motor_1.ChangeDutyCycle(pump_pressure)
#			    print('CMHO reacged')
#			    print(indiff)
# 			if(P_plat <= 17 and (indiff >= P_plat/2 or time_elapsed_inhale >= 0.2) and flow_flag == 0): #(time_elapsed_inhale > 0.3 and flow_flag == 0) or indiff >= 10): #or indiff >= P_plat - 4): #or volFlow_rate >= flow_plat - 10):
# 			    flow_flag = 1
# 			    motor_1.ChangeDutyCycle(pump_pressure)
# 			if(flow_flag == 1 and volFlow_rate <= flow_plat):
# 			    motor_1.ChangeDutyCycle(pump_pressure)
			if(volFlow_rate >= flow_plat and flow_flag ==1):
			    flow_flag = 2
			if(flow_flag == 3):
			    if(volFlow_rate <= flow_plat and pump_pressure_now <= 98):
				pump_pressure_now = pump_pressure_now + 1
			        motor_1.ChangeDutyCycle(pump_pressure_now)
			    if(volFlow_rate >= flow_plat and pump_pressure_now >= 2): #and pump_pressure_now >( pump_pressure /2)):
			        pump_pressure_now = pump_pressure_now - 1
			        motor_1.ChangeDutyCycle(pump_pressure_now)
# 			if(pump_pressure_now <= pump_pressure and volFlow_rate <= flow_plat):
# 			    if(pump_flag == 0):
# 			        pump_pressure_now  = pump_pressure_now + 1
# 			    else:
# 				pump_pressure_now = pump_pressure_now + 0.5
# 	#		    print('the pump_pressure now is')
# 	#		    print(pump_pressure_now)
# 	#		    print('pump pressure is')
# 	#		    print(pump_pressure)
# 			    motor_1.ChangeDutyCycle(pump_pressure_now)
		        if(volume_comp >= VTi_max and volume_flag == 0):
		    	    volume_reach_time = time_elapsed_inhale
		    	    GPIO.output(33, GPIO.HIGH)
		    	    volume_flag = 1
#			print(inhale_time)
			inhale_array.append(indiff)
			VTi_volume = max(volume,VTi_volume)
			peak_insp_pressure = max(peak_insp_pressure, indiff)
			volume_peak_inhale=max(volume_peak_inhale,volFlow_rate)
		    if(IHold == '1'):
			motor_1.ChangeDutyCycle(int(pump_pressure*0.9))
			GPIO.output(33, GPIO.HIGH)
			print("entering inspiratory hold")
			print(IH_time)
			ser.write("ACK66")
			instant_time = time()
		        sending_time = 0.2
		        send_last_time=0
			while(time() - instant_time < IH_time):
			    print(sending_time)
			    indiff= self.ABP_pressure()
			    volFlow_rate=self.rate()
			    if(sending_time > 0.02):
			        packet_inhalation =('A@' + str(round(indiff,2)) + ','+str(round(volFlow_rate,2)) + ',' + str(int(volume)) + ',' + str(trigger) + '#')
			        try:
				    ser.write(packet_inhalation)
			        except:
				    print('BT error send Inhalation')
			        send_last_time= time()
			    sending_time=time() - send_last_time
		    IHold = '0'
		    GPIO.output(33, GPIO.LOW)
		    if(indiff >= (PIP+2)):
			pressure_count = pressure_count +1
			if(pressure_count >= 3):
		            try:
		  	        ser.write('ACK07')
			    except:
		      	        print('unable to send')
		    else:
			pressure_count = 0
			try:
			    ser.write('ACK57')
			except:
			    print('unable to send')
		    if(indiff <= (P_plat)*0.5):
			pressure_low_count = pressure_low_count +1
			if(pressure_low_count >= 3):
		            try:
		  	        ser.write('ACK62')
			    except:
		      	        print('unable to send')
		    else:
			pressure_low_count = 0
			try:
			    ser.write('ACK72')
			except:
			    print('unable to send')
		    GPIO.output(37, GPIO.LOW)
		    MVi_array.append(VTi_volume)
		    VTi_volume = VTi_volume - ((VTi_volume*leak_percentage)/200)
                    flag=1
		    loop = 0
		    if(VTi_volume < (VTi_max*0.8) and indiff >= PIP and patient_status == 1):
		        ser.write("ACK61")
		    else:
			ser.write("ACK71")
		    if(VTi_volume < (VTi_max*0.8) and pump_pressure >= 95 and patient_status == 1):
			ser.write("ACK60")
		    else:
			ser.write("ACK70")
		    toggle_switch = str(toggle())
		    if(EHold == '1'):
			toggle_switch ='1'
	#	    print("the toffle switch in here issssssssssssssssssssssssssss " + toggle_switch)
		#    motor_1.ChangeDutyCycle(peep)
		    sleep(0.01)
		   # GPIO.output(35, GPIO.HIGH) # low level triggered relay
		    if(toggle_switch == "0"):
#			print("got in the first statement")
		        motor_1.ChangeDutyCycle(peep_open)
			sol.ChangeDutyCycle(0)
		    if(toggle_switch == "1"):
#			print("got in the 2nd statement")
#			GPIO.output(33, GPIO.HIGH)
		        motor_1.ChangeDutyCycle(peep)
			sol.ChangeDutyCycle(0)
	#		sleep(0.01)
	#		GPIO.output(21, GPIO.HIGH)
		    inhale_array_length = len(inhale_array)
		    if(inhale_array_length % 2 != 0):
			inhale_array.append(0)
			inhale_array_length = inhale_array_length + 1
		    inhale_first_half = inhale_array[0:inhale_array_length/2]
		    inhale_second_half = inhale_array[inhale_array_length/2:]
		    first_max = max(inhale_first_half)
		    second_max = max(inhale_second_half)
		    if(first_max - P_plat > 0.5 and motor_factor > 0.2):
		        motor_factor = motor_factor - 0.01
			if(first_max - P_plat > 2):
			    motor_factor = motor_factor - 0.01
		    if(P_plat - first_max > 0.5 and motor_factor <= 1.0):
			motor_factor = motor_factor + 0.01
			if(P_plat - first_max > 2):
			    motor_factor = motor_factor + 0.01
		    if(VTi_volume < (VTi_max * 0.8)):
			volume_count = volume_count + 1
			if(volume_count >= 3):
			    buzzer(7,1)
			    try:
			        ser.write('ACK36')
			    except:
			        print('unable to send')
		    else:
			buzzer(7,0)
			volume_count = 0
			try:
			    ser.write('ACK46')
			except:
			    print('unable to send')
		    try:
		        Pmean= sum(Pmean_array)/len(Pmean_array)
		    except:
			print("zero exception error")
		    try:
		        if(compliance_flag == 0):
		            compliance_array.append(VTi_volume/(peak_insp_pressure - peep_val))
			    if(len(compliance_array) == 2):
			        compliance = (sum(compliance_array)/len(compliance_array)) * 0.75
			        pressure = (VTi_max/compliance) + peep_val
			        if(pressure > PIP):
				    pressure  = PIP - 2
	        	        P_plat_value=find_nearest(P_plat_array,pressure)
	        	        index= P_plat_array.index(P_plat_value)
				inst_pump_pressure = pump_pressure_array[index]
				if(inst_pump_pressure > pump_pressure):
	        	            pump_pressure = pump_pressure_array[index]  
			        compliance_flag = 1
		    except:
			print("error")
#		    print('Pmean is')
#		    print('Here it comessssssssss')
#		    print(int(VTi_volume))
#		    print(VTi_max)
#		    print(peak_insp_pressure)
#		    print(patient_status)
                    RR.append(RR_time)
#		    print(pump_pressure_high)
		    indiff = int(indiff)
		    if((VTi_volume > VTi_max*1.1 or (volume_reach_time < inhale_time*0.8 and trigger == '0' and VTi_volume >= VTi_max)) and compliance_flag == 1 and peak_insp_pressure >= peep_val+5 and  ABP_flag == 0 and SDP_flag == 0 and patient_status == 1 and pump_pressure >= 5 and pump_pressure <= 95):
			pump_pressure = pump_pressure - 1
			P_plat = indiff
#			print('-1')
		    elif((VTi_volume < VTi_max or volume_reach_time >= inhale_time*0.8) and ABP_flag == 0 and compliance_flag == 1 and SDP_flag == 0 and indiff < PIP and patient_status ==1 and pump_pressure >= 4 and pump_pressure <= 95):
			pump_pressure = pump_pressure + 1
# 			if((VTi_max - VTi_volume) > 100 and leak_percentage < 70  and VTi_max > 100):
# 			    if(((VTi_max - VTi_volume)/3 ) > 5):
# 				pump_pressure = pump_pressure + 5
# 			    else:
# 			        pump_pressure = pump_pressure + ((VTi_max - VTi_volume)/30)
# 			elif(VTi_max > 100):
# 			    pump_pressure = pump_pressure + 1
# 			if(VTi_max <= 100):
# 			    pump_pressure = pump_pressure + 5
			P_plat = indiff
		    if(pump_pressure > 95):
			pump_pressure = 95
		    if(patient_status == 1 and peep_val_send < peep_val and peep <= peep_first + 10  and peep >= 4 and  peep < 90 and toggle_switch == '1'):
			peep = peep + 1
		    elif(patient_status == 1 and peep_val_send > peep_val and peep >= peep_first - 10  and peep > 5 and peep < 92 and toggle_switch == '1'):
			peep = peep - 1
			print("reducing the peeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeep")
		    print("Diag peep is " + str(peep))
#		    print(VTi_max)
#		    print(VTi_volume)
	#	    MVi_array.append(VTi_volume)
#		    print(peak_insp_pressure)
#		    print(patient_status)
#		    print(P_plat)
#		    print(pump_pressure)
		if(flag == 1):
	   	    patient_set = 0
		    if((leak_percentage > 90 or indiff <= 5) and patient_set == 0 ):
			buzzer(5,1)
			patient_set = 1
			patient_status = 0
#		        print('disconnection')
#			trigger = '0'
			try:
			    ser.write('ACK05')
			except:
			    print('not able to send')
		       # GPIO.output(22, GPIO.HIGH)
		    if(( leak_percentage <= 90 or indiff > 5) and patient_set == 0):
			buzzer(5,0)
			patient_set = 1
			patient_status = 1
#			trigger = '1'
			try:
			    ser.write('ACK06')
			except:
			    print('not able to send')
                    if(len(RR) >= 5):
                        BPM = ( RR[1] + RR[2] + RR[3] + RR[4]) /4
                        BPM = ceil(60/BPM)
			if(BPM > 70):
			    try:
				ser.write('ACK09')
			    except:
				print('unable to send')
			else:
			    try:
				ser.write('ACK59')
			    except:
				print('unable to send')
 #                       print('BPM is')
  #                      print(BPM)
                        RR.pop(0)
		    if(len(MVi_array) >= 5):
		        MVi= (MVi_array[1] + MVi_array[2] + MVi_array[3] + MVi_array[4])/4
			MVi = MVi * BPM
#		        print('MVi value is')
#		        print(MVi)
		        MVi_array.pop(0)
		        packet_end_inhalation = ('B@' + str(round(peak_insp_pressure,2)) + "," + str(int(VTi_volume)) + "," + str(round(volume_peak_inhale,2)) + "," + str(round(Pmean,2)) + "," + str(round((MVi/1000),2))  +  "," + str(round(patient_trigger_flow,2)) + "," + str(round(time_elapsed_inhale,2)) + "," + str(round(indiff,2)) + "," + str('0') + "," + str(round(rise_time,2)) +'#')
#		        print('packet length is')
		        print(packet_end_inhalation)
		        try:
		            ser.write(packet_end_inhalation)
		        except:
			    print('BT Errror in end of inhalation loop')
                if(flag==1):
		    time_error_last = time()
		    time_elap = 0
		    inhale_loop = "0"
		    q= time()
		    del exp_flow_array[:]
#	            del exp_flow_array[:]
		    volume = 0
		    sending_time = 200
		    sol_flag = 0
		    send_last_time = 0
		    volume_peak_exhale = 0
		    exhale_break = '0'
		    while(time_elap < 0.1 or (toggle_switch == "1" and time_elap < 0.3)):
			w=time()
			time_elap=w-q
#			exp_flow = self.rate()
 #                       print("print in CE is :: " + str(peep))
                        indiff = self.ABP_pressure()
			exp_press_array.append(indiff)
			peak_insp_pressure = int(peak_insp_pressure)
			if(peak_insp_pressure <= 0):
			    peak_insp_pressure = 1
			ratio = 1.0
                        if(ratio > 0.8 and time_elap >= 0.2 and toggle_switch == "1" and exhale_break == '0'):
                            GPIO.output(35, GPIO.LOW)
                            sleep(0.05)
			    print("1 condition breaking")
 #                           motor_1.ChangeDutyCycle(peep)
			    exhale_break = '0'
                        elif(ratio >= 0.15 and ratio <= 0.4 and time_elap >= 0.4 and P_plat < 28 and toggle_switch == "1" and exhale_break == '0'):
                            GPIO.output(35, GPIO.LOW)
                            sleep(0.05)
			    print("2 condition breaking")
  #                          motor_1.ChangeDutyCycle(peep)
			    exhale_break = '0'
                        elif(ratio >= 0.15 and ratio <= 0.4 and time_elap >= 0.4 and P_plat >=28 and toggle_switch == "1" and exhale_break == '0'):
                            GPIO.output(35, GPIO.LOW)
                            sleep(0.05)
			    print("3 condition breaking")
   #                         motor_1.ChangeDutyCycle(peep)
			    exhale_break = '0'
                        elif(ratio > 0.4 and ratio <= 0.8 and time_elap >= 0.3 and toggle_switch == "1" and exhale_break == '0'):
                            GPIO.output(35, GPIO.LOW)
                            sleep(0.05)
			    print("4 condition breaking")
    #                        motor_1.ChangeDutyCycle(peep)
			    exhale_break = '0'
                        elif(ratio <0.15 and ratio >=0.00 and time_elap >= 0.4 and toggle_switch == "1" and exhale_break == '0'):
                            GPIO.output(35, GPIO.LOW)
                            sleep(0.05)
			    print("5 condition breaking")
     #                       motor_1.ChangeDutyCycle(peep)
			    exhale_break = '0'
			diff = self.SDP_pressure()
			if(SDP_flag == 0):
			    volFlow_rate = self.rate()
			    volume=self.Flow()
			if(diff == 700 or diff == -700 or SDP_flag == 1):
			    try:
				f = open("/home/pi/AgVa_5.0/mode.txt","w")
				f.write(str('11'))
				f.close()
				try:
				    ser.write('ACK30')
				except:
				    print('unable to send data')
			    except:
				print('unable to write mode')
			packet_exhalation= ('C@' + str(round(indiff,2)) + ',' + str(round(volFlow_rate,2)) +',' + str(int(volume)) + ',' + str(int((TITOT)*100)) + '#')
			if(sending_time > 0.02):
			    try:
				ser.write(packet_exhalation)
			    except:
				print('BT exhalation error')
			    send_last_time = time()
			sending_time = w- send_last_time
			print('ratio time toggle  break ' + str(ratio) +  ",," + str(time_elap) + "," + str(toggle_switch) + "," + str(exhale_break))
#			print((packet_exhalation))
			volume_peak_exhale= min(volume_peak_exhale, volFlow_rate)
# 		        if(toggle_switch == "1" and indiff <= peep_val and sol_flag == 0):
#                             sol.ChangeDutyCycle(100)
# 			    holding_now = time()
# 			    holding_time = 0.3
# 			    peep_check = peep_val - trigflow - 1
# 			    if(peep_check < 0):
# 			        peep_check = 0
# 			    while( time() - holding_now <= holding_time and time_elap < 0.3):
# 				GPIO.output(33, GPIO.LOW)
# 				time_elap = time() - q
# 				indiff = self.ABP_pressure()
# 				if(peep_val <= peep_check):
# 				    break;
# 			        volFlow_rate = self.rate()
# 			        volume=self.Flow()
# 				packet_exhalation= ('C@' + str(round(indiff,2)) + ',' + str(round(0,2)) +',' + str(int(volume)) + ',' + str(int((TITOT)*100)) + '#')
# 				if( 1 == 1):
# 			    	    try:
# 					ser.write(packet_exhalation)
# 			    	    except:
# 					print('BT exhalation error')
# 			    	    send_last_time = time()
# 				sending_time = w- send_last_time
# 				sleep(0.02)
# 				sol_flag = 1
#			print('packet_exhal;ation conatins is')
#			print(packet_exhalation)
# 		    if(toggle_switch == "1" and indiff <= peep_val-peep_factor and sol_flag == 0):
#                         sol.ChangeDutyCycle(100)
# 			holding_now = time()
# 			holding_time = 0.3
# 			sending_time = 2.0
# 			send_last_time = 0
# 			while( time() - holding_now <= holding_time):
# 			    indiff = self.ABP_pressure()
# 			    volFlow_rate = self.rate()
# 			    volume=self.Flow()
# 			    packet_exhalation= ('C@' + str(round(indiff,2)) + ',' + str(round(0,2)) +',' + str(int(volume)) + ',' + str(int((TITOT)*100)) + '#')
# 			    if(1 == 1):
# 			        try:
# 				    ser.write(packet_exhalation)
# 			    	except:
# 				    print('BT exhalation error')
# 			    	send_last_time = time()
# 			    sending_time = time()- send_last_time
# 			    sleep(0.02)
# 			    sol_flag = 1
#		    print("----------------------" + str(peep))
		    temp_peep = self.ABP_pressure()
#
                    new_time=time()
                    time_elapsed_exhale=0.0
                    diff=self.SDP_pressure()
		    time_elapsed_exhale_flow = 0
		    start_time=current_time
		    sending_time=200
		    send_last_time = 0
#		    sol_flag  = 0
                    while((time_elapsed_exhale<(cycle_time-inhale_time-0.3) and (diff <= 0 or indiff > peep_val* 1.5)) or (SDP_flag ==1 and time_elapsed_exhale<(cycle_time-inhale_time-0.5))):
                        time_elapsed_exhale=time()-new_time
#			exp_flow = self.rate()
 #                       exp_flow_array.append(exp_flow)
			indiff = self.ABP_pressure()
			exp_press_array.append(indiff)
			if((toggle_switch == "1" and sol_flag == 0 and ((indiff <= peep_val+2 and volFlow_rate >= -10) or (indiff <= peep_val-2))) or (time_elapsed_exhale>(cycle_time-inhale_time-0.3)) ):
			    peep_check = peep_val - trigflow - 1
			    if(peep_check < 0):
			        peep_check = 0
			    if(volFlow_rate >= trigflow):
			        break;
			    sol.ChangeDutyCycle(100)
			    sol_flag = 1
			    holding_now = time()
			    holding_time = 0.3
# 			    peep_check = trigflow - peep_val - 2.0
# 			    if(peep_check < 0):
# 			        peep_check = 0
			    while( time() - holding_now <= holding_time and (time_elapsed_exhale<(cycle_time-inhale_time-0.3))):
				GPIO.output(33, GPIO.LOW)
				time_elapsed_exhale = time() - new_time
				indiff = self.ABP_pressure()
				if(volFlow_rate >= trigflow+peep_val):
				    break;
			        volFlow_rate = self.rate()
			        volume=self.Flow()
				packet_exhalation= ('C@' + str(round(indiff,2)) + ',' + str(round(0,2)) +',' + str(int(volume)) + ',' + str(int((TITOT)*100)) + '#')
				if(1 == 1):
			    	    try:
					ser.write(packet_exhalation)
			    	    except:
					print('BT exhalation error')
			    	    send_last_time = time()
				sending_time = time()- send_last_time
				sleep(0.02)
				sol_flag = 1
			if(SDP_flag == 0):
           		    volume=self.Flow()
			    diff = self.SDP_pressure()
			    volFlow_rate=self.rate()
			volume_peak_exhale=min(volume_peak_exhale,volFlow_rate)
 			if(volFlow_rate < -5):
 			    time_elapsed_exhale_flow = time_elapsed_exhale
			packet_exhalation = ('C@' + str(round(indiff,2)) + ',' + str(round(volFlow_rate,2)) + ',' + str(int(volume)) + ',' + str(int((TITOT)*100))+'#')
			if(sending_time > 0.02):
			    try:
				ser.write(packet_exhalation)
			    except:
				print('BT Error sending in Exhalation conditional')
			    send_last_time = time()
			sending_time =time() - send_last_time 
                        start_time=current_time
                    #    print(start_time)
#		    flow_for_peep = 0.25 * (min(exp_flow_array))
                    if(int(peep_val_send) >=  peep_val - 1 and int(peep_val_send) <= peep_val+1):
                        lock = 1
                    elif(int(peep_val_send) > peep_val+1 or int(peep_val_send) < peep_val -1):
                        lock = 0
		    if(EHold == '1'):
			GPIO.output(33, GPIO.HIGH)
			print("Entering expiratory hold")
			instant_time = time()
			ser.write("ACK67")
		        sending_time = 0.2
		        send_last_time=0
			while(time() - instant_time < EH_time):
			    indiff= self.ABP_pressure()
			    print(time() - instant_time)
			    volFlow_rate=self.rate()
			    if(sending_time > 0.02):
			        packet_exhalation = ('C@' + str(round(indiff,2)) + ',' + str(round(volFlow_rate,2)) + ',' + str(int(volume)) + ',' + str(int((TITOT)*100))+'#')
			        try:
				    ser.write(packet_exhalation)
			        except:
				    print('BT error send Inhalation')
			        send_last_time= time()
			    sending_time=time() - send_last_time
			peep_val_send = indiff
			GPIO.output(33, GPIO.LOW)
		    EHold = '0'
 #                   print(flow_for_peep)
  #                  value1 = find_nearest(exp_flow_array , flow_for_peep)
   #                 peep_array_index = exp_flow_array.index(value1)
#                    peep_val_send = exp_press_array[peep_array_index]
                    flag=0
		    inhale_loop = "1"
		    loop = 1
		    exhale_time_last = time()
		    time_elapsed_exhale = time_elapsed_exhale + 0.5
#		    peep_val_send = temp_peep
#		    print(time_elapsed_inhale /( time_elapsed_inhale + time_elapsed_exhale))
		    TITOT=(time_elapsed_inhale/(time_elapsed_inhale + time_elapsed_exhale + TOT_last))
#		    if(len(exp_press_array) > 10):
#			peep_val_send =  exp_press_array[len(exp_press_array) - 3]
		    if(int(peep_val_send) <= peep_val and patient_status == 1 and toggle_switch == '1' and lock == 0):
			peep_factor = peep_factor - (peep_val - peep_val_send)*0.1
		    if(int(peep_val_send) > peep_val and patient_status == 1 and toggle_switch == '1' and lock == 0):
			peep_factor = peep_factor + (peep_val_send - peep_val)*0.1
		    if(int(peep_val_send) > peep_val+2):
			peep_count = peep_count + 1
			if(leak_percentage > 20 and trigger == '0'):
			    ser.write("ACK39")
			if(peep_count >=3 and peep_set == 0):
			    buzzer(4,1)
			    peep_set = 1
			    try:
			        ser.write('ACK08')
			    except:
			        print('unable to send')
		    else:
			if(peep_set == 1):
			    buzzer(4,0)
			    peep_set = 0
			peep_count = 0
			ser.write("ACK49")
			try:
			    ser.write('ACK58')
			except:
			    print('unable to send')
		    MVe_array.append(volume)
		    FiO2 = self.FiO2()
                    self.power()
#		    print('FiO2 is')
#		    print(FiO2)
		    if(toggle_switch == '1'):
		        time_elapsed_exhale = time_elapsed_exhale + 0.4
			time_elapsed_exhale_flow = time_elapsed_exhale_flow + 0.4
		    else:
			time_elapsed_exhale = time_elapsed_exhale + 0.1
			time_elapsed_exhale_flow = time_elapsed_exhale_flow + 0.1
		    if(len(MVe_array)>=5):
		        MVe= (MVe_array[1] + MVe_array[2] + MVe_array[3] + MVe_array[4])/4
			MVe = MVe * BPM
		        MVe_array.pop(0)
		        try:
			    leak_percentage = ((MVi - MVe)/MVi)*100
			    if(leak_percentage <= -1):
			        leak_percentage = 0
		        except:
			    leak_percentage = 0
#		        print('MVe is')
#		        print(MVe)
		   # packet_end_exhalation = 'ABC'
		        packet_end_exhalation = ('D@' + str(round(peep_val_send,2)) + ',' + str(int(BPM)) + ',' + str(FiO2) + ',' + str(round(volume_peak_exhale,2)*-1)  + "," + str(round((MVe/1000),2))  + "," + str(leak_percentage) + "," + str(trigflow_comp - trigflow) + "," + str(volume)  +  "," + str(time_elapsed_exhale_flow) + '#')
#			print('string issssssssssssssssssssssssssssssssssssss')
#			print(packet_end_exhalation)
		        try:
	                    ser.write(packet_end_exhalation)
		        except:
			    print('BT send error in end of exhalation')
	#	    try:
	#	        with open('/home/pi/AgVa_5.0/data.csv', mode='a') as data:
   	#	            data = csv.writer(data, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
	#	            now = datetime.datetime.now()
	#	            data.writerow([str(now),str(self.measure_temp()),str(Pmean),str(volume),str(volume_peak_inhale),str(trigger),str(BPM),str(MVi),str(ratio), str(volume_peak_exhale),str(MVe)])
	#	    except:
	#		print('data file open error')
        except KeyboardInterrupt:
            return
#-------------------------------------------------------------
    def Spont(self):
        global exp_press_array,exp_flow_array,peep,peep_open,time_elapsed_exhale_flow,patient_trigger_flow,rise_time, rise_flag, inhale_array,motor_factor,pressure_low_count,leak_comp_flag,running_avg,comp_flag, trigflow_comp,back_backup,inhale_loop,toggle_switch,loop,peep_open,toggle_switch,peep,patient_set,TITOT,exhale_time_last, TOT_last,patient_status,pressure_count,peep_count, ABP_flag, SDP_flag, pump_pressure_high, pump_pressure_low, pump_pressure,flow_plat,ratio_set,time_error_set,thread_mode_status,ABP_flag,time_error, time_error_last,P_plat_value,prev_mode,data,ratio,RR_time,indiff, RR, BPM, MVi_array, MVi,volume, volume_peak_inhale, trigger, max_value, volume_peak_exhale, MVe_array, MVe,clock_t2
        thread_mode_status = True
	time_elapsed_exhale_flow = 0
	rise_time = 0
	patient_trigger_flow = 0
	rise_flag = '0'
	pressure_low_count = 0
	pressure_count = 0
	trigflow_comp = 0
	comp_flag = '0'
	del MVi_array[:]
	del MVe_array[:]
	del exp_press_array[:]
	del exp_flow_array[:]
	inhale_array = []
	motor_factor = 0.5
	running_avg = 0.0
	peep_count = 0
	exhale_time_last = time()
	GPIO.output(35, GPIO.LOW)
	TOT_last = time()
	ratio_set = 0
	inhale_loop = "0"
	loop = 1
	VTi_volume = 0
	leak_percentage = 0
	flag = 0
	patient_set = 0
	peep_set = 0
	time_error_set = 0
	flag = 0
	flow_plat = 50
	sol.ChangeDutyCycle(100)
	trigger = '1'
	peep_val_send = 0
	temp_peep = 0
	current_time=time()
        start_time=time() 
	time_error_last = time()
	sending_time = 0.2
        try:
            while True:
		prev_mode = 14
		ABP_flag = 0
		print('we are in Spont mode Got in here yipeeeeeeeeeeeeeeee')
		data= self.read_data()
	#	print('mode is')
	#	print(data)
		if(data == 1 or (current_time-start_time) > 10):
		    if( current_time-start_time >= 10):
		        try:
			    f = open("/home/pi/AgVa_5.0/backup.txt","w")
			    f.write("140")
			    f.close()
			    back_backup = 1
			    f = open("/home/pi/AgVa_5.0/mode.txt","w")
			    f.write(str("220"))
			    f.close()
			    ser.write("ACK38")
			except:
			    print("unable to fall back to PC_IMV mode")
		    break;
	#	print(inhale_time)
	#	print(BPM)
	#	print(PIP)
	#	print(VTi_max)
	#	print(P_plat)
	#	print(trigflow)
		inhale_time = 3.0
		indiff= self.ABP_pressure()
		exp_press_array.append(indiff)
		volFlow_rate = self.rate()
		#indiff= self.ABP_pressure()
		if(indiff == -8888):
			ABP_flag = 1
			indiff = 0
		packet_exhalation = ('C@' + str(round(indiff,2)) + ',' + str(round(volFlow_rate,2)) + ',' + str(int(volume)) + ',' + str(int((TITOT)*100))+'#')
		if(sending_time > 0.02):
		    try:
			ser.write(packet_exhalation)
		    except:
			print('BT Error sending in Exhalation conditional')
		    send_last_time = time()
		sending_time =time() - send_last_time 
	#	print("THE FLOW IN HERE IS ")
	#	volflow_rate = volFlow_rate + 4
                flow=self.rate()
	        if(indiff <= peep_val+1 and comp_flag == '1' and leak_comp_flag == '1'):
		    trigflow_comp = trigflow + running_avg + 3
		    comp_flag = '0'
		    running_avg = 0
 #               flow=self.rate()
#		flow = flow - peep_hole[int(peep_val)]
	#	print("THE FLOWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW IN HERE IS")
		print(flow)
		if(leak_comp_flag == '0'):
		    trigflow_comp = trigflow
                current_time=time()
        #        volume=0
	        if(comp_flag == '0' and flow > 1):
		    running_avg = (running_avg + flow)/2
		TOT_last = current_time - exhale_time_last
#		peep_val_send = indiff
		SDP_flag = 0
		flow_flag_unset = 0
		loop = 1
		time_error = time() - time_error_last
                if(flow>trigflow_comp):
		    RR_time=current_time-start_time
		    pump_pressure_now = pump_pressure
		    patient_trigger_flow = flow
		    inhale_loop = "1"
		    del inhale_array[:]
		    leak_comp_flag = leak_comp()
		    volume = 0
		    comp_flag = '1'
		    flow_flag = 0
	#	    GPIO.output(19,GPIO.HIGH)
		    sol.ChangeDutyCycle(100)
 		    peep_val_send = temp_peep
 		    if(len(exp_press_array) > 10):
 		        peep_val_send = exp_press_array[len(exp_press_array) - 4]
 		    del exp_press_array[:]
#                    motor_1.ChangeDutyCycle(pump_pressure)
                    motor_1.ChangeDutyCycle(100)
		    if(P_plat <= 22):
			motor_1.ChangeDutyCycle(80)
#		    if(P_plat <= 17):
#			motor_1.ChangeDutyCycle(65)
# 			pump_pressure_now = pump_pressure /2
                    #RR_time=current_time-start_time
		    if(time_error < 0.7):
			time_error_set = 1
#			try:
#			    ser.write('ACK24')
#			except:
#			    print('unable to send data')
		    if(time_error > 0.7):
			time_error_set = 0
#			try:
#			    ser.write('ACK24')
#			except:
#			    print('unable to send data')
		    if(float(ratio) <= 0.25):
			ratio_set = 1
#			try:
#			    ser.write('ACK12')
#			except:
#			    print('unable to send data')
		    if(float(ratio) >= 0.75):
			ratio_set = 1
#			try:
#			    ser.write('ACK13')
#			except:
#			    print('unable to send data')
		    if(float(ratio) >0.25 and float(ratio) < 0.75):
			ratio_set = 0
#			try:
#			    ser.write('ACK22')
#			except:
#			    print('unable to send data')
#		    if(RR_time > cycle_time):
#			trigger='0'
#		    else:
		    trigger='1'
		    rise_flag = '0'
                 #   indiff= self.ABP_pressure()
                  #  print(indiff)
                    time_elapsed_inhale=0
		    now = time()
		    GPIO.output(37, GPIO.HIGH)
		    time_elapsed = 0
		    volume_peak_inhale=0
		    flow_flag = 0
		    volFlow_rate = self.rate()
		    pump_flag = 0
                    #print("flow is  " + str(volFlow_rate)) 
#		    pump_pressure_now = pump_pressure /2
#		    while(time_elapsed < 0.3):
#			time_elapsed = time() - now
#			indiff = self.ABP_pressure()
#			if(indiff >= (P_plat*1.15)):
#			    ser.write('ACK07')
#			if(SDP_flag == 0):
#			    volFlow_rate = self.rate()
#			    volume_peak_inhale=max(volume_peak_inhale,volFlow_rate)
#			if(indiff >= P_plat/2 and flow_flag == 0 and time_elapsed_inhale >= 0.3): #(time_elapsed_inhale > 0.3 and flow_flag == 0) or indiff >= 10): #or indiff >= P_plat - 4): #or volFlow_rate >= flow_plat - 10):
#			    flow_flag = 1
#			    print('CMHO reacged')
#			    print(indiff)
#			if(flow_flag == 1 and volFlow_rate <= flow_plat):
#			    motor_1.ChangeDutyCycle(pump_pressure)
#			if(volFlow_rate >= flow_plat and flow_flag ==1):
#			    flow_flag = 2
#			if(flow_flag == 2):
#			    if(volFlow_rate <= flow_plat and pump_pressure_now <= 98):
#				pump_pressure_now = pump_pressure_now + 1
#			        motor_1.ChangeDutyCycle(pump_pressure_now)
#			    if(volFlow_rate >= flow_plat and pump_pressure_now >= 2): #and pump_pressure_now >( pump_pressure /2)):
#			        pump_pressure_now = pump_pressure_now - 1
#			        motor_1.ChangeDutyCycle(pump_pressure_now)
#			packet_inhalation =('A@' + str(int(indiff)) + ','+str(int(volFlow_rate)) + ',' + str(int(volume)) + ',' + str(trigger) + '#')
#			if(sending_time >0.7):
#			    try:
#			        ser.write(packet_inhalation)
#				sending_time = 0.4
#			    except:
#			        print('BT error send Inhalation')
		    time_elapsed  = 0
		    Pmean = 0
		    del Pmean_array[:]
		    Pmean_array.append(1)
		    volFlow_rate = self.rate()
		    clock_t2= time()
#		    print('now it is')
#		    print(clock_t2)
		    volume=0
		    sending_time = 0.2
		    send_last_time=0
		    VTi_volume=0
		    peak_insp_pressure = 0
		    flow_flag_100 = 0
		    pump_pressure_now = pump_pressure /2
		    volFlow_rate_previous = volFlow_rate
		    peak_flow = volFlow_rate
                    while((peak_flow*0.40 <= volFlow_rate  and time_elapsed_inhale <= inhale_time) or time_elapsed_inhale <= 0.5):
                    #    motor_1.ChangeDutyCycle(pump_pressure)
                        #motor_2.ChangeDutyCycle(pump_pressure)
                        t3 = time()
			GPIO.output(37, GPIO.HIGH)
			print(pump_pressure)
			#GPIO.output(35,GPIO.HIGH)
			if(ABP_flag == 0):
                            indiff= self.ABP_pressure()
                        time_elapsed_inhale= t3-current_time
			if(indiff >= P_plat * 0.8 and rise_flag == '0'):
				rise_time = time_elapsed_inhale
				rise_flag = '1'
                   #     print(indiff)
			Pmean_array.append(indiff)
			if(SDP_flag == 0):
			    volume=self.Flow()
			    volFlow_rate=self.rate()
			    if(volFlow_rate >= peak_flow):
				peak_flow = volFlow_rate
			if(volume == -9999):
			    SDP_flag = 1;
			if(sending_time > 0.02):
			    packet_inhalation =('A@' + str(round(indiff,2)) + ','+str(round(volFlow_rate,2)) + ',' + str(int(volume)) + ',' + str(trigger) + '#')
			    try:
				ser.write(packet_inhalation)
			    except:
				print('BT error send Inhalation')
#			    print('packet size isssssssssss')
#			    print(len(packet_inhalation))
#			    print('packet conatins ')
	#		    print(packet_inhalation)
			    send_last_time= time()
			sending_time=t3 - send_last_time
 			if((indiff >= P_plat*motor_factor or time_elapsed_inhale >= 0.5) and flow_flag == 0): #(time_elapsed_inhale > 0.3 and flow_flag == 0) or indiff >= 10): #or indiff >= P_plat - 4): #or volFlow_rate >= flow_plat - 10):
			    flow_flag = 1
			    motor_1.ChangeDutyCycle(pump_pressure)
			    print('Changing the pWM in 1st')
	#		print(flow_plat)
# 			if(P_plat <= 17 and (indiff >= P_plat/2 or time_elapsed_inhale >= 0.2) and flow_flag == 0): #(time_elapsed_inhale > 0.3 and flow_flag == 0) or indiff >= 10): #or indiff >= P_plat - 4): #or volFlow_rate >= flow_plat - 10):
# 			    flow_flag = 1
# 			    motor_1.ChangeDutyCycle(pump_pressure)
			    print("changing pwm in 2nd")
# 			if(flow_flag == 1 ): #and volFlow_rate <= flow_plat):
# 			    motor_1.ChangeDutyCycle(pump_pressure)
# 			    print("chaning motor duty cyle")
			if(volFlow_rate >= flow_plat and flow_flag ==1):
			    flow_flag = 2
			if(flow_flag == 3):
			    if(volFlow_rate <= flow_plat and pump_pressure_now <= 98):
				pump_pressure_now = pump_pressure_now + 1
			        motor_1.ChangeDutyCycle(pump_pressure_now)
				print("got in the 1st loop")
			    if(volFlow_rate >= flow_plat and indiff  >=  PIP): #and pump_pressure_now >( pump_pressure /2)):
			        pump_pressure_now = pump_pressure_now - 1
			        motor_1.ChangeDutyCycle(pump_pressure_now)
# 			if(pump_pressure_now <= pump_pressure and volFlow_rate <= flow_plat):
# 			    if(pump_flag == 0):
# 			        pump_pressure_now  = pump_pressure_now + 1
# 			    else:
# 				pump_pressure_now = pump_pressure_now + 0.5
# 	#		    print('the pump_pressure now is')
# 	#		    print(pump_pressure_now)
# 	#		    print('pump pressure is')
# 	#		    print(pump_pressure)
# 			    motor_1.ChangeDutyCycle(pump_pressure_now)
# 			elif(volFlow_rate > flow_plat and pump_pressure_now >( pump_pressure /2)):
# 			    pump_pressure_now = pump_pressure_now - 0.5
# 			    motor_1.ChangeDutyCycle(pump_pressure_now)
# 			    pump_flag = 1
#			print(time_elapsed_inhale)
#			print(inhale_time)
                                print("got in the 2nd loop")
			VTi_volume = max(volume,VTi_volume)
			inhale_array.append(indiff)
			peak_insp_pressure = max(peak_insp_pressure, indiff)
			volume_peak_inhale=max(volume_peak_inhale,volFlow_rate)
		    if(indiff >= (P_plat+3)):
			pressure_count = pressure_count +1
			if(pressure_count >= 3):
		            try:
		  	        ser.write('ACK07')
			    except:
		      	        print('unable to send')
		    else:
			pressure_count = 0
			try:
			    ser.write('ACK57')
			except:
			    print('unable to send')
		    if(indiff <= (P_plat)*0.5):
			pressure_low_count = pressure_low_count +1
			if(pressure_low_count >= 3):
		            try:
		  	        ser.write('ACK62')
			    except:
		      	        print('unable to send')
		    else:
			pressure_low_count = 0
			try:
			    ser.write('ACK72')
			except:
			    print('unable to send')
                    flag=1
		    loop = 0
		    toggle_switch = str(toggle())
		    sleep(0.01)
		    if(toggle_switch == "0"):
			print("got in the first statement")
		        motor_1.ChangeDutyCycle(peep_open)
			sol.ChangeDutyCycle(0)
		    if(toggle_switch == "1"):
			print("got in the 2nd statement")
		        motor_1.ChangeDutyCycle(peep)
			sol.ChangeDutyCycle(100)
		    inhale_array_length = len(inhale_array)
		    if(inhale_array_length % 2 != 0):
			inhale_array.append(0)
			inhale_array_length = inhale_array_length + 1
		    inhale_first_half = inhale_array[0:inhale_array_length/2]
		    inhale_second_half = inhale_array[inhale_array_length/2:]
		    first_max = max(inhale_first_half)
		    second_max = max(inhale_second_half)
		    if(first_max - P_plat > 0.5 and motor_factor > 0.2):
		        motor_factor = motor_factor - 0.01
			if(first_max - P_plat > 2):
			    motor_factor = motor_factor - 0.01
		    if(P_plat - first_max > 0.5 and motor_factor <= 1.0):
			motor_factor = motor_factor + 0.01
			if(P_plat - first_max > 2):
			    motor_factor = motor_factor + 0.01
		    #GPIO.output(35, GPIO.HIGH) # low level triggered relay
		    try:
		        Pmean= sum(Pmean_array)/len(Pmean_array)
		    except:
			print("zero exception error")
		  #  motor_1.ChangeDutyCycle(peep)
		    GPIO.output(19, GPIO.LOW)
	#	    print("TOGGLE SWITCH IS:")
	#	    print(toggle_switch)
	#	    print(P_plat)
                    RR.append(RR_time)
		    indiff = int(indiff)
		    if(indiff > P_plat and patient_status == 1 and ABP_flag == 0 and indiff < PIP and pump_pressure >= 20 and pump_pressure <= 97):
			pump_pressure = pump_pressure - 1
	#		print(peak_insp_pressure)
		    elif(indiff < P_plat and patient_status == 1 and ABP_flag == 0 and indiff < PIP and  pump_pressure >= 19 and pump_pressure <= 95):
			pump_pressure = pump_pressure + 1
	#		print('+1')
		    if(patient_status == 1 and peep_val_send < peep_val and peep > 10 and peep < 90):
			peep = peep + 2
		    elif(patient_status == 1 and peep_val_send > peep_val and peep > 10 and peep < 90):
			peep = peep - 1
		   # print('')
	#	    print("THE INHALE VOLUME IS : " + str(VTi_volume))
#		    print(peak_insp_pressure)
		  #  print(volume_peak_inhale)
		    MVi_array.append(VTi_volume)
#		    print('trigger is')
#		    print(trigger)
#		    print('Pmean is')
#		    print(Pmean)
		    GPIO.output(37, GPIO.LOW)		    
		if(flag == 1):
		    patient_set = 0
		    if((peak_insp_pressure <= P_plat * 0.5) and patient_set == 0 ):
			buzzer(5,1)
			patient_set = 1
			patient_status = 0
	#	        print('disconnection')
#			trigger = '0'
			try:
			    ser.write('ACK05')
			except:
			    print('not able to send')
		       # GPIO.output(22, GPIO.HIGH)
		    if((peak_insp_pressure> P_plat * 0.5) and patient_set == 0):
			patient_set = 1
			patient_status = 1
			buzzer(5,0)
			try:
			    ser.write('ACK06')
			except:
			    print('not able to send')
                    if(len(RR) >= 5):
                        BPM = ( RR[1] + RR[2] + RR[3] + RR[4]) /4
                        BPM = ceil(60/BPM)
			if(BPM > 70):
			    try:
				ser.write('ACK09')
			    except:
				print('unable to send')
			else:
			    try:
				ser.write('ACK59')
			    except:
				print('unable to send')
 #                      print('BPM is')
  #                     print(BPM)
                        RR.pop(0)
		    if(len(MVi_array) >= 5):
		        MVi= (MVi_array[1] + MVi_array[2] + MVi_array[3] + MVi_array[4])/4
			MVi = MVi * BPM
		        print('MVi value is .............. : ')
		        print(MVi_array)
		        MVi_array.pop(0)
		        packet_end_inhalation = ('B@' + str(round(peak_insp_pressure,2)) + "," + str(int(VTi_volume - (VTi_volume*leak_percentage)/100)) + "," + str(round(volume_peak_inhale,2)) + "," + str(round(Pmean,2)) + "," + str(round((MVi/1000),2)) + "," + str(round(patient_trigger_flow,2)) + "," + str(round(time_elapsed_inhale,2)) + "," + str(round(indiff,2)) + "," + str('0') + "," + str(round(rise_time,2)) +'#')
#		        print('packet length is')
	#	        print(packet_end_inhalation)
		        try:
		            ser.write(packet_end_inhalation)
		        except IOError:
			    print('BT Errror in end of inhalation loop')
                if(flag==1):
		    time_error_last = time()
		    time_elap = 0
		    q= time()
		    del exp_press_array[:]
		    del exp_flow_array[:]
		    volume = 0
		    inhale_loop = "0"
		    buzzer(7,0)
		    sending_time = 200
		    send_last_time = 0
		    volume_peak_exhale = 0
		    while(time_elap < 0.1):
			w=time()
			time_elap=w-q
			ratio = peep_val/P_plat
#			exp_flow = self.rate()
 #                       exp_flow_array.append(exp_flow)
                        indiff = self.ABP_pressure()
			exp_press_array.append(indiff)
                  #      GPIO.output(19, GPIO.HIGH)
   #                     GPIO.output(35, GPIO.LOW)
                        if(ratio > 0.8 and time_elap >= 0.2):
                         #2   GPIO.output(35, GPIO.LOW)
                            sleep(0.05)
        #                    motor_1.ChangeDutyCycle(peep)
                        elif(ratio >= 0.15 and ratio <= 0.4 and time_elap >= 0.12 and P_plat < 28):
                           # GPIO.output(35, GPIO.LOW)
                            sleep(0.05)
         #                   motor_1.ChangeDutyCycle(peep)
                        elif(ratio >= 0.15 and ratio <= 0.4 and time_elap >= 0.4 and P_plat >=28):
                            #GPIO.output(35, GPIO.LOW)
                            sleep(0.05)
          #                  motor_1.ChangeDutyCycle(peep)
                        elif(ratio > 0.4 and ratio <= 0.8 and time_elap >= 0.3):
                            #GPIO.output(35, GPIO.LOW)
                            sleep(0.05)
           #                 motor_1.ChangeDutyCycle(peep)
                        elif(ratio <0.15 and ratio >=0.00 and time_elap >= 0.4):
                            #GPIO.output(35, GPIO.LOW)
                            sleep(0.05)
            #                motor_1.ChangeDutyCycle(peep)
			diff = self.SDP_pressure()
			if(ABP_flag == 0):
			    indiff = self.ABP_pressure()
			if(SDP_flag == 0):
			    volFlow_rate = self.rate()
			    volume=self.Flow()
			if(diff == 700 or diff == -700 or SDP_flag == 1):
			    try:
				f = open("/home/pi/AgVa_5.0/mode.txt","w")
				f.write(str('11'))
				f.close()
				try:
				    ser.write('ACK30')
				except:
				    print('unable to send data')
			    except:
				print('unable to write mode')
			packet_exhalation= ('C@' + str(round(indiff,2)) + ',' + str(round(volFlow_rate,2)) +',' + str(int(volume)) + ',' + str(int((TITOT)*100)) + '#')
			if(sending_time > 0.02):
			    try:
				ser.write(packet_exhalation)
			    except:
				print('BT exhalation error')
			    send_last_time = time()
			sending_time = w- send_last_time
#			print('packet exhalation size  is')
#			print((packet_exhalation))
			volume_peak_exhale= min(volume_peak_exhale, volFlow_rate)
	#		print('packet_exhal;ation conatins is')
	#		print(packet_exhalation)
                    if(toggle_switch == "1" and volFlow_rate <= (volume_peak_exhale*0.05) and indiff <= peep_val):
			sol.ChangeDutyCycle(100)
		    temp_peep = self.ABP_pressure()
                    new_time=time()
                    time_elapsed_exhale=0.0
                    diff=self.SDP_pressure()
		    start_time=current_time
		    sending_time=200
		    time_elapsed_exhale_flow = 0
		    send_last_time = 0
		    sol_flag = 0
                    while(( diff < 0 and time_elapsed_exhale < 5.0) or (SDP_flag ==1 and time_elapsed_exhale<(cycle_time-inhale_time-0.5))):
                        if(toggle_switch == "1" and sol_flag  == 0 and volFlow_rate <= (volume_peak_exhale*0.05) and indiff <= peep_val):
                            sol.ChangeDutyCycle(100)
			    sol_flag  = 1
		#	    sleep(0.3)
                        time_elapsed_exhale=time()-new_time
#			exp_flow = self.rate()
 #                       exp_flow_array.append(exp_flow)
			if(ABP_flag == 0):
			    indiff = self.ABP_pressure()
			exp_press_array.append(indiff)
			if(SDP_flag == 0):
           		    volume=self.Flow()
			    volFlow_rate=self.rate()
			    diff = self.SDP_pressure()
			volume_peak_exhale=min(volume_peak_exhale,volFlow_rate)
 			if(volFlow_rate < -5):
 			    time_elapsed_exhale_flow = time_elapsed_exhale
			packet_exhalation = ('C@' + str(round(indiff,2)) + ',' + str(round(volFlow_rate,2)) + ',' + str(int(volume)) + ',' + str(int((TITOT)*100))+'#')
			if(sending_time > 0.02):
			    try:
				ser.write(packet_exhalation)
			    except:
				print('BT Error sending in Exhalation conditional')
			    send_last_time = time()
			sending_time =time() - send_last_time 
                        start_time=current_time
                        print(volume)
#		    flow_for_peep = 0.25 * (min(exp_flow_array))
 #                   print(flow_for_peep)
  #                  value1 = find_nearest(exp_flow_array , flow_for_peep)
   #                 peep_array_index = exp_flow_array.index(value1)
              #      peep_val_send = exp_press_array[peep_array_index]	
                    flag=0
		    exhale_time_last = time()
		    inhale_loop = "1"
		    loop = 1
		    time_elapsed_exhale = time_elapsed_exhale + 0.5
#		    peep_val_send = temp_peep
#		    print(time_elapsed_inhale /( time_elapsed_inhale + time_elapsed_exhale))
		    TITOT=(time_elapsed_inhale/(time_elapsed_inhale + time_elapsed_exhale + TOT_last))
#		    if(len(exp_press_array) > 10):
#		        peep_val_send =  exp_press_array[len(exp_press_array) - 3]
		    if(int(peep_val_send) <= peep_val and patient_status == 1 and peep < 100 and peep_open < 100):
			if(toggle_switch == "1"):
			    peep = peep + 2
			else:
			    peep_open = peep_open + 2
		    if(int(peep_val_send) > peep_val and patient_status == 1 and peep > 0 and peep_open > 0):
			if(toggle_switch == "1"):
			    peep = peep - 1
			else:
			    peep_open = peep_open - 1
		    if(int(peep_val_send) > peep_val+2):
			peep_count = peep_count + 1
			if(leak_percentage > 20 and trigger == '0'):
			    ser.write("ACK39")
			if(peep_count >=3 and peep_set == 0):
			    buzzer(4,1)
			    peep_set = 1
			    try:
			        ser.write('ACK08')
			    except:
			        print('unable to send')
		    else:
			if(peep_set == 1):
			    buzzer(4,0)
			    peep_set = 0
			peep_count = 0
			ser.write("ACK49")
			try:
			    ser.write('ACK58')
			except:
			    print('unable to send')
		    MVe_array.append(volume)
		    FiO2 = self.FiO2()
                    self.power()
		    print('MVi is')
		    print(MVi)
		    if(toggle_switch == '1'):
		        time_elapsed_exhale = time_elapsed_exhale + 0.4
			time_elapsed_exhale_flow = time_elapsed_exhale_flow + 0.4
		    else:
			time_elapsed_exhale = time_elapsed_exhale + 0.1
			time_elapsed_exhale_flow = time_elapsed_exhale_flow + 0.1
		    if(len(MVe_array)>=5):
		        MVe= (MVe_array[1] + MVe_array[2] + MVe_array[3] + MVe_array[4])/4
			MVe = MVe * BPM
		        MVe_array.pop(0)
		        print('MVe is ....................: ' + str(MVe))
		        try:
			    leak_percentage = ((MVi - MVe)/MVi)*100
			    if(leak_percentage <= -1):
			        leak_percentage = 0
		        except:
			    leak_percentage = 0
		        print('Leak percentage is')
		        print(leak_percentage)
		        packet_end_exhalation = ('D@' + str(round(peep_val_send,2)) + ',' + str(int(BPM)) + ',' + str(FiO2) + ',' + str(round(volume_peak_exhale,2)*-1)  + "," + str(round((MVe/1000),2)) + "," + str(leak_percentage) + "," + str(trigflow_comp - trigflow) + "," + str(volume)  + "," + str(time_elapsed_exhale_flow) + '#')
			print('MVE array is')
			print(MVe_array)
		        try:
	                    ser.write(packet_end_exhalation)
		        except:
			    print('BT send error in end of exhalation')
	#	    try:
	#	        with open('/home/pi/AgVa_5.0/data.csv', mode='a') as data:
   	#	            data = csv.writer(data, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
	#	            now = datetime.datetime.now()
	#	            data.writerow([str(now),str(self.measure_temp()),str(Pmean),str(volume),str(volume_peak_inhale),str(trigger),str(BPM),str(MVi),str(ratio), str(volume_peak_exhale),str(MVe)])
	#	    except:
	#		print('data file open error')
        except KeyboardInterrupt:
            return
#------------------------------------------------------------------
#-----------------------------------------------------------
    def NIV_AVAPS(self):
        global change_setting,volume_flag,peep,peep_open,time_elapsed_exhale_flow,patient_trigger_flow,rise_time, rise_flag,pressure_low_count,primitive_inhale_time,inhale_time,backup_flag,BIPAP_backup_flag,back_backup,inhale_loop,toggle_switch,loop,peep,patient_set,ABP_flag,exhale_time_last,TOT_last,TITOT,pressure_count,peep_count, SDP_flag,flow_plat,indiff,patient_status,pump_pressure,pump_pressure_low,pump_pressure_high,ratio_set,time_error_set,thread_mode_status,time_error, time_error_last,ABP_flag,P_plat_value,prev_mode,data,ratio,RR_time,indiff,peep_val, peep_hole, RR, BPM, MVi_array, MVi,volume, volume_peak_inhale, trigger, max_value, volume_peak_exhale, MVe_array, MVe,clock_t2
        thread_mode_status = True
	time_elapsed_exhale_flow = 0
	rise_time = 0
	volume_flag = 0
	patient_trigger_flow = 0
	rise_flag = '0'
	pressure_low_count= 0
	patient_set = 0
	exhale_time_last = time()
	TOT_last = time()
	peep_count = 0
	inhale_loop = "0"
	flag = 0
	backup_counter = 0
	loop = 1
	del MVi_array[:]
	del MVe_array[:]
	VTi_volume = 0
	leak_percentage = 0
	GPIO.output(35, GPIO.LOW)
	pressure_count = 0
	peep_set = 0
	ratio_set = 0
	time_error_set = 0
	sol.ChangeDutyCycle(100)
        start_time=time()
	current_time = time()
	time_error_last = time()
	sending_time = 0.2
        try:
            while True:
		prev_mode = 35
		ABP_flag = 0
		print('we are in AVAPS MODE')
		data= self.read_data()
		print(backup_flag)
#		print(data)
		if(((current_time-start_time) >= 10) or BIPAP_backup_flag == 1):
		    if(current_time-start_time >= 10 or BIPAP_backup_flag == 1):
#			change_setting = '1'
			backup_flag = 1
			try:
			    ser.write("ACK38")
			except:
			    print("unable to send")
		    else:
			motor_1.ChangeDutyCycle(peep)
		        break;
		if(data == 1):
		    motor_1.ChangeDutyCycle(peep)
		    break;
		if(backup_flag == 1):
		    inhale_time = primitive_inhale_time
		else:
		    inhale_time = 3.0
#		print(PIP)
#		PIP = P_plat +1
#	        inhale_time = 3.0
		#trigflow = 15
#		trig_flow = trigflow + 5
#		print('peep in main loop is')
#		print(peep)
#		print(peep_val)
#		print(P_plat)
#		print(P_plat_value)
#		print('here the pump pressure is')
#		print(pump_pressure)
	#	GPIO.output(22, GPIO.LOW)
		volFlow_rate = self.rate()
		indiff= self.ABP_pressure()
		if(indiff == -8888):
			ABP_flag = 1
			indiff = 0
		packet_exhalation = ('C@' + str(round(indiff,2)) + ',' + str(round(volFlow_rate,2)) + ',' + str(int(volume)) + ',' + str(int((TITOT)*100))+'#')
		if(sending_time > 0.02):
		    try:
			ser.write(packet_exhalation)
		    except:
			print('BT Error sending in Exhalation conditional')
		    send_last_time = time()
		sending_time =time() - send_last_time 
		print('CYCLE TIMEIS')
		print(cycle_time)
                flow=self.rate()
#		print(current_time - start_time)
#		print(flow)
	#	flow = flow - peep_hole[int(peep_val)]
#		print('after flow')
#		print(flow)
                current_time=time()
        #        volume=0
		TOT_last = current_time - exhale_time_last
		peep_val_send = indiff
		SDP_flag = 0
		time_error = time() - time_error_last
		print(current_time - start_time)
		loop = 1
                if(flow>trigflow or ((current_time-start_time)>cycle_time and backup_flag == 1)):
		    RR_time=current_time-start_time
		    pump_pressure_now = pump_pressure
		    volume = 0
		    inhale_loop = "1"
		    indiff = self.ABP_pressure()
		    patient_trigger_flow = flow
		    sol.ChangeDutyCycle(100)
		    flow_flag = 0
#		    if((pump_pressure /2) < peep):
#			motor_1.ChangeDutyCycle(peep + 2)
#			pump_pressure_now = peep +2
#		    else:
#		    print('indiff right now is')
#		    print(indiff)
#		    if(indiff <= P_plat/2):
                    motor_1.ChangeDutyCycle(pump_pressure)
		    if(P_plat <= 22):
			motor_1.ChangeDutyCycle(pump_pressure)
#		    if(P_plat <= 17):
#			motor_1.ChangeDutyCycle(65)
#		        flow_flag = 0
#		    else:
#			motor_1.ChangeDutyCycle(pump_pressure)
#			flow_flag = 1
#			pump_pressure_now = pump_pressure /2
		    if(time_error < 0.7):
			time_error_set = 1
#			try:
#			    ser.write('ACK24')
#			except:
#			    print('unable to send data')
#		    if(time_error > 0.7):
#			time_error_set = 0
#			try:
#			    ser.write('ACK24')
#			except:
#			    print('unable to send data')
		    if(float(ratio) <= 0.25):
			ratio_set = 1
#			try:
#			    ser.write('ACK12')
#			except:
#			    print('unable to send data')
		    if(float(ratio) >= 0.75):
			ratio_set = 1
#			try:
#			    ser.write('ACK13')
#			except:
#			    print('unable to send data')
		    if(float(ratio) >0.25 and float(ratio) < 0.75):
			ratio_set = 0
#			try:
#			    ser.write('ACK22')
#			except:
#			    print('unable to send data')
		    if(RR_time > cycle_time):
			trigger='0'
			backup_counter = 0
		    else:
			trigger='1'
			if(backup_flag == 1):
			    backup_counter = backup_counter + 1
			if(BIPAP_backup_flag == 1 and backup_counter >= 4):
			    sol.ChangeDutyCycle(0)
			    motor_1.ChangeDutyCycle(peep_open)
			    try:
				f = open("/home/pi/AgVa_5.0/backup.txt","r")
				backup_setting = f.readline()
				f.close()
				f = open("/home/pi/AgVa_5.0/mode.txt","w")
				f.write(str(backup_setting))
				f.close()
				BIPAP_backup_flag = 0
				try:
				    ser.write("ACK48")
				except:
				    print("getting back")
				motor_1.ChangeDutyCycle(peep)
				break;
			    except:
				print("unable to go back to the original mode")
			if(backup_counter >= 4 and backup_flag == 1):
			    backup_flag = 0
			    try:
				ser.write("ACK48")
			    except:
				print("unable to send")
                 #   indiff= self.ABP_pressure()
                  #  print(indiff)
                    time_elapsed_inhale=0
                    indiff = 0
		    Pmean=0
		    del Pmean_array[:]
		    clock_t2= time()
#		    print('now it is')
#		    print(clock_t2)
		    volume=0
		    volume_peak_inhale=0
		    sending_time = 0.2
		    rise_flag = '0'
		    send_last_time=0
		    volFlow_rate = self.rate()
		    VTi_volume=0
		    pump_flag = 0
		   # flow_flag = 0
		    current_time = time()
		    peak_insp_pressure = 0
		    pump_pressure_now = pump_pressure /2
#		    flow_zero_intialize = time() # clock starts to detect zero flow
#		    flow_zero_flag = 0
#		    flow_zero_elapsed = 0
		    peak_flow = volFlow_rate
                    while(((peak_flow*0.25 <= volFlow_rate) and time_elapsed_inhale <= inhale_time) or time_elapsed_inhale <= 0.5):
                    #    motor_1.ChangeDutyCycle(pump_pressure)
                        #motor_2.ChangeDutyCycle(pump_pressure)
                        t3 = time()
			GPIO.output(37, GPIO.HIGH)
#			if(volFlow_rate <= 2):
#			    flow_zero_flag = 1
#			    flow_zero_initialize = time()
#		        if(flow_zero_flag == 1):
#			    flow_zero_elapsed = time() - flow_zero_initialize
#			    if(flow_zero_elapsed >= )
#			    flow_zero_elapsed = time() - flow_zero_initialize 
			if(ABP_flag == 0):
                            indiff= self.ABP_pressure()
                        time_elapsed_inhale= t3-current_time
                        print(indiff)
			if(indiff >= P_plat * 0.8 and rise_flag == '0'):
				rise_time = time_elapsed_inhale
				rise_flag = '1'
			Pmean_array.append(indiff)
			if(SDP_flag == 0):
			    volume=self.Flow()
			    volFlow_rate=self.rate()
			    if(volFlow_rate >= peak_flow):
				peak_flow = volFlow_rate
			if((indiff >= P_plat/2 or time_elapsed_inhale >= 0.4) and flow_flag == 0): #(time_elapsed_inhale > 0.3 and flow_flag == 0) or indiff >= 10): #or indiff >= P_plat - 4): #or volFlow_rate >= flow_plat - 10):
			    flow_flag = 1
			    motor_1.ChangeDutyCycle(pump_pressure)
#			    print('CMHO reacged')
#			    print(indiff)
			if(P_plat <= 17 and (indiff >= P_plat/2 or time_elapsed_inhale >= 0.2) and flow_flag == 0): #(time_elapsed_inhale > 0.3 and flow_flag == 0) or indiff >= 10): #or indiff >= P_plat - 4): #or volFlow_rate >= flow_plat - 10):
			    flow_flag = 1
			    motor_1.ChangeDutyCycle(pump_pressure)
# 			if(flow_flag == 1 and volFlow_rate <= flow_plat):
# 			    motor_1.ChangeDutyCycle(pump_pressure)
			if(volFlow_rate >= flow_plat and flow_flag ==7):
			    flow_flag = 2
			if(volume == -9999):
			    SDP_flag = 1;
			if(sending_time > 0.02):
			    packet_inhalation =('A@' + str(round(indiff,2)) + ','+str(round(volFlow_rate,2)) + ',' + str(int(volume)) + ',' + str(trigger) + '#')
			    try:
				ser.write(packet_inhalation)
			    except:
				print('BT error send Inhalation')
#			    print('packet size isssssssssss')
#			    print(len(packet_inhalation))
#			    print('packet conatins ')
#			    print(packet_inhalation)
			    send_last_time= time()
#			if(volFlow_rate < flow_plat -3 and pump_pressure_now < pump_pressure):
#			    pump_pressure_now = pump_pressure_now + 4
#			    print('in 1st loop')
#			    motor_1.ChangeDutyCycle(pump_pressure_now)
#			elif(volFlow_rate > flow_plat + 3 and pump_pressure_now > pump_pressure):
#			    pump_pressure_now = pump_pressure_now -4
#			    print('in 2nd loop')
#			    motor_1.ChangeDutyCycle(pump_pressure_now)
			if(flow_flag == 3):
			    if(volFlow_rate <= flow_plat -5 and pump_pressure_now <= P_plat):
		#	   # if(pump_flag == 0):
			      #      pump_pressure_now  = pump_pressure_now + 5
			     #   else:
				pump_pressure_now = pump_pressure_now + 1
	#		    print('the pump_pressure now is')
	#		    print(pump_pressure_now)
	#		    print('pump pressure is')
	#		    print(pump_pressure)
			        motor_1.ChangeDutyCycle(pump_pressure_now)
			    if(volFlow_rate >= flow_plat and pump_pressure_now >= P_plat/2): #and pump_pressure_now >( pump_pressure /2)):
			        pump_pressure_now = pump_pressure_now - 1
			        motor_1.ChangeDutyCycle(pump_pressure_now)
			      #  pump_flag = 1
			sending_time=t3 - send_last_time
#			print('flow is')
#			print(volFlow_rate)
#			print(time_elapsed_inhale)
#			print(inhale_time)
			VTi_volume = max(volume,VTi_volume)
			peak_insp_pressure = max(peak_insp_pressure, indiff)
			volume_peak_inhale=max(volume_peak_inhale,volFlow_rate)
		    if(indiff >= (PIP+1)):
			pressure_count = pressure_count +1
			if(pressure_count >= 1):
		            try:
		  	        ser.write('ACK07')
			    except:
		      	        print('unable to send')
		    else:
			pressure_count = 0
			try:
			    ser.write('ACK57')
			except:
			    print('unable to send')
		    if(indiff <= (P_plat)*0.5):
			pressure_low_count = pressure_low_count +1
			if(pressure_low_count >= 3):
		            try:
		  	        ser.write('ACK62')
			    except:
		      	        print('unable to send')
		    else:
			pressure_low_count = 0
			try:
			    ser.write('ACK72')
			except:
			    print('unable to send')
                    flag=1
		    loop = 0
#		    print('volume is')
		    MVi_array.append(VTi_volume)
		    motor_1.ChangeDutyCycle(peep)
		    VTi_volume = VTi_volume - ((VTi_volume*leak_percentage)/200)
		    if(toggle_switch == "0"):
			print("got in the first statement")
		        motor_1.ChangeDutyCycle(peep)
			sol.ChangeDutyCycle(0)
		    if(toggle_switch == "1"):
			print("got in the 2nd statement")
		        motor_1.ChangeDutyCycle(peep)
			sol.ChangeDutyCycle(0)
		 #   GPIO.output(35, GPIO.HIGH) # HIGH level triggered relay
		    try:
		        Pmean= sum(Pmean_array)/len(Pmean_array)
		    except:
			print("zero exception error")
		    print('P_plat is')
		    print(int(peak_insp_pressure))
		    print(P_plat)
                    RR.append(RR_time)
		    peak_insp_pressure = int(peak_insp_pressure)
		    if(VTi_volume > VTi_max and peak_insp_pressure > P_plat and patient_status == 1 and ABP_flag == 0 and peak_insp_pressure < (PIP+2)  and pump_pressure >= 20 and pump_pressure <= 99):
			pump_pressure = pump_pressure - 1
			print('-1')
		    elif((VTi_volume < VTi_max or peak_insp_pressure < P_plat) and patient_status == 1 and ABP_flag == 0 and peak_insp_pressure < PIP and  pump_pressure >= 20 and pump_pressure <= 95):
			if(VTi_max - VTi_volume > 100):
			    pump_pressure = pump_pressure + 2
			    print('+2')
			else:
			    pump_pressure = pump_pressure + 1
			    print('+1')
		    print(VTi_max)
		    print(VTi_volume)
		    print(PIP)
		    print(pump_pressure)
#		    MVi_array.append(VTi_volume)
		    print(pump_pressure_high)
#		    print(trigger)
#		    print('Pmean is')
#		    print(Pmean)
		    GPIO.output(37, GPIO.LOW)
		if(flag == 1):
		    patient_set = 0
		    if((leak_percentage >= 90)  and patient_set == 0 ):
			buzzer(5,1)
			patient_set = 1
			patient_status = 0
		        print('disconnection')
#			trigger = '0'
			try:
			    ser.write('ACK05')
			except:
			    print('not able to send')
		#        GPIO.output(22, GPIO.HIGH)
		    if((leak_percentage < 90) and patient_set == 0):
			buzzer(5,0)
			patient_set = 1
			patient_status = 1
#			trigger = '1'
			try:
			    ser.write('ACK06')
			except:
			    print('not able to send')
                    if(len(RR) >= 5):
                        BPM = ( RR[1] + RR[2] + RR[3] + RR[4]) /4
                        BPM = ceil(60/BPM)
			if(BPM > 70):
			    try:
				ser.write('ACK09')
			    except:
				print('unable to send')
			else:
			    try:
				ser.write('ACK59')
			    except:
				print('unable to send')
 #                      print('BPM is')
  #                     print(BPM)
                        RR.pop(0)
		    if(len(MVi_array) >= 5):
		        MVi= (MVi_array[1] + MVi_array[2] + MVi_array[3] + MVi_array[4])/4
			MVi = MVi * BPM
#		        print('MVi value is')
#		        print(MVi)
		        MVi_array.pop(0)
		        packet_end_inhalation = ('B@' + str(round(peak_insp_pressure,2)) + "," + str(int(VTi_volume)) + "," + str(round(volume_peak_inhale,2)) + "," + str(int(Pmean)) + "," + str(round((MVi/1000),2)) + "," + str(round(patient_trigger_flow,2)) + "," + str(round(time_elapsed_inhale,2)) + "," + str(round(indiff,2)) + "," + str('0') + "," + str(round(rise_time,2)) +'#')
#		        print('packet length is')
#		        print(packet_end_inhalation)
		        try:
		            ser.write(packet_end_inhalation)
		        except:
			    print('BT Errror in end of inhalation loop')
                if(flag==1):
		    time_error_last = time()
		    time_elap = 0
		    q= time()
		    inhale_loop = "0"
		    buzzer(7,0)
		    volume = 0
		    sending_time = 200
		    send_last_time = 0
		    volume_peak_exhale = 0
		    while(time_elap < 0.3 or (toggle_switch == "1" and time_elap < 0.4)):
			w=time()
			time_elap=w-q
                        indiff = self.ABP_pressure()
			ratio = peep_val/P_plat
                        if(ratio > 0.8 and time_elap >= 0.005 and toggle_switch == '1'):
                    #        GPIO.output(35, GPIO.LOW)
                            sleep(0.05)
       #                     motor_1.ChangeDutyCycle(peep)
                        elif(ratio >= 0.15 and ratio <= 0.4 and time_elap >= 0.3 and P_plat < 28 and toggle_switch == '1'):
                   #         GPIO.output(35, GPIO.LOW)
                            sleep(0.05)
        #                    motor_1.ChangeDutyCycle(peep)
                        elif(ratio >= 0.15 and ratio <= 0.4 and time_elap >= 0.3 and P_plat >=28 and toggle_switch == '1'):
                    #        GPIO.output(35, GPIO.LOW)
                            sleep(0.05)
         #                   motor_1.ChangeDutyCycle(peep)
                        elif(ratio > 0.4 and ratio <= 0.8 and time_elap >= 0.001 and toggle_switch == '1'):
                   #         GPIO.output(35, GPIO.LOW)
                            sleep(0.05)
          #                  motor_1.ChangeDutyCycle(peep)
                        elif(ratio <0.15 and ratio >=0.00 and time_elap >= 0.3 and toggle_switch == '1'):
                        #    GPIO.output(35, GPIO.LOW)
                            sleep(0.05)
           #                 motor_1.ChangeDutyCycle(peep)
			if(ABP_flag == 0):
			    indiff = self.ABP_pressure()
# 			if(indiff <= (peep + 1)):
# 			    print('turning off the relay')
			diff = self.SDP_pressure()
			if(SDP_flag == 0):
			    volFlow_rate = self.rate()
			    volume=self.Flow()
			if(diff == 700 or diff == -700 or SDP_flag == 1):
			    try:
				f = open("/home/pi/AgVa_5.0/mode.txt","w")
				f.write(str('11'))
				f.close()
				try:
				    ser.write('ACK30')
				except:
				    print('unable to send data')
			    except:
				print('unable to write mode')
			packet_exhalation= ('C@' + str(round(indiff,2)) + ',' + str(round(volFlow_rate,2)) +',' + str(int(volume)) + ',' + str(int((TITOT)*100)) + '#')
			if(sending_time > 0.02):
			    try:
				ser.write(packet_exhalation)
			    except:
				print('BT exhalation error')
			    send_last_time = time()
			sending_time = w- send_last_time
#			print('packet exhalation size  is')
#			print(volFlow_rate)
			volume_peak_exhale= min(volume_peak_exhale, volFlow_rate)
#			print('packet_exhal;ation conatins is')
#			print(packet_exhalation)
                    #motor_2.ChangeDutyCycle(0)
                    new_time=time()
                    time_elapsed_exhale=0.0
                    diff=self.SDP_pressure()
		    if(toggle_switch == "1" and volFlow_rate <= (volume_peak_exhale * 0.05) and indiff <= peep_val):
                        sol.ChangeDutyCycle(0)
			sleep(0.3)
		    start_time=current_time
		    sending_time=200
		    sol_flag  = 0
		    time_elapsed_exhale_flow = 0
		    send_last_time = 0
                    while(( diff <= 0 and ((backup_flag == 0 and time_elapsed_exhale <= 5.0 ) or (backup_flag == 1 and time_elapsed_exhale<(cycle_time-inhale_time)))) or (SDP_flag ==1 and time_elapsed_exhale<(cycle_time-inhale_time-0.5))):
                        time_elapsed_exhale=time()-new_time
			if(ABP_flag == 0):
			    indiff = self.ABP_pressure()
		        if(toggle_switch == "1" and sol_flag == 0 and volFlow_rate <= (volume_peak_exhale * 0.05) and indiff <= peep_val):
                            sol.ChangeDutyCycle(0)
			    sol_flag = 1
			    sleep(0.3)
			if(SDP_flag == 0):
           		    volume=self.Flow()
			    volFlow_rate=self.rate()
			    diff = self.SDP_pressure()
# 			if(indiff <= (peep + 1)):
# 			    GPIO.output(35, GPIO.LOW)
# 			    print('turning off the relay')
			volume_peak_exhale=min(volume_peak_exhale,volFlow_rate)
 			if(volFlow_rate < -5):
 			    time_elapsed_exhale_flow = time_elapsed_exhale
			packet_exhalation = ('C@' + str(round(indiff,2)) + ',' + str(round(volFlow_rate,2)) + ',' + str(int(volume)) + ',' + str(int((TITOT)*100))+'#')
			if(sending_time > 0.02):
			    try:
				ser.write(packet_exhalation)
			    except:
				print('BT Error sending in Exhalation conditional')
			    send_last_time = time()
			sending_time =time() - send_last_time 
                        start_time=current_time
                    #    print(start_time)
                    flag=0 
		    loop = 1
		    inhale_loop = "1"
		    exhale_time_last = time()
		    time_elapsed_exhale = time_elapsed_exhale + 0.5
#		    print(volFlow_rate)
#		    print(time_elapsed_inhale /( time_elapsed_inhale + time_elapsed_exhale))
		    TITOT=(time_elapsed_inhale/(time_elapsed_inhale + time_elapsed_exhale + TOT_last))
#		    print('peak volume flow rate exhale')
#		    print(volume_peak_exhale)
		    if(int(peep_val_send) <= peep_val and patient_status == 1 and peep < 100 and peep_open < 100):
			if(toggle_switch == "1"):
			    peep = peep + 2
			else:
			    peep_open = peep_open + 2
		    if(int(peep_val_send) > peep_val and patient_status == 1 and peep > 0 and peep_open > 0):
			if(toggle_switch == "1"):
			    peep = peep - 1
			else:
			    peep_open = peep_open - 1
		    if(int(peep_val_send) > peep_val+2):
			peep_count = peep_count + 1
			if(leak_percentage > 20 and trigger == '0'):
			    ser.write("ACK39")
			if(peep_count >=3 and peep_set == 0):
			    buzzer(4,1)
			    peep_set = 1
			    try:
			        ser.write('ACK08')
			    except:
			        print('unable to send')
		    else:
			if(peep_set == 1):
			    buzzer(4,0)
			    peep_set = 0
			peep_count = 0
			ser.write("ACK49")
			try:
			    ser.write('ACK58')
			except:
			    print('unable to send')
		    FiO2 = self.FiO2()
                    self.power()
		    print('FiO2 is')
		    print(FiO2)
		    if(toggle_switch == '1'):
		        time_elapsed_exhale = time_elapsed_exhale + 0.4
			time_elapsed_exhale_flow = time_elapsed_exhale_flow + 0.4
		    else:
			time_elapsed_exhale = time_elapsed_exhale + 0.1
			time_elapsed_exhale_flow = time_elapsed_exhale_flow + 0.1
		    MVe_array.append(volume)
		    if(len(MVe_array)>=5):
		        MVe= (MVe_array[1] + MVe_array[2] + MVe_array[3] + MVe_array[4])/4
			MVe = MVe * BPM
		        MVe_array.pop(0)
		        try:
			    leak_percentage = ((MVi - MVe)/MVi)*100
			    if(leak_percentage <= -1):
			        leak_percentage = 0
		        except:
			    leak_percentage = 0
#		        print('MVe is')
#		        print(MVe)
		   # packet_end_exhalation = 'ABC'
		        packet_end_exhalation = ('D@' + str(round(peep_val_send,2)) + ',' + str(int(BPM)) + ',' + str(FiO2) + ',' + str(round(volume_peak_exhale,2)*-1)  + "," + str(round((MVe/1000),2)) + "," + str(leak_percentage) + "," + str(trigflow) + "," + str(volume) + "," + str(time_elapsed_exhale_flow) + '#')
#			print('string issssssssssssssssssssssssssssssssssssss')
#			print(packet_end_exhalation)
		        try:
	                    ser.write(packet_end_exhalation)
		        except:
			    print('BT send error in end of exhalation')
	#	    try:
	#	        with open('/home/pi/AgVa_5.0/data.csv', mode='a') as data:
   	#	            data = csv.writer(data, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
	#	            now = datetime.datetime.now()
	#	            data.writerow([str(now),str(self.measure_temp()),str(Pmean),str(volume),str(volume_peak_inhale),str(trigger),str(BPM),str(MVi),str(ratio), str(volume_peak_exhale),str(MVe)])
	#	    except:
	#		print('data file open error')
        except KeyboardInterrupt:
            return
#-------------------------------------------------------------------
    def NIV_APAP(self):
        global peep,peep_open,time_elapsed_exhale_flow,patient_trigger_flow,rise_time, rise_flag,pressure_low_count,backup_flag,BIPAP_backup_flag,inhale_loop,toggle_switch,loop,peep,patient_set,exhale_time_last, TOT_last,flow_plat,TITOT,pressure_count,pump_pressure_low,ABP_flag, SDP_flag,pump_pressure,pump_pressure_high, patient_status,ratio_set,time_error_set,thread_mode_status,time_error, time_error_last,ABP_flag,P_plat_value,prev_mode,data,ratio,RR_time,indiff,peep_val, peep_hole, RR, BPM, MVi_array, MVi,volume, volume_peak_inhale, trigger, max_value, volume_peak_exhale, MVe_array, MVe,clock_t2
        thread_mode_status = True
	time_elapsed_exhale_flow = 0
	rise_time =0
	patient_trigger_flow = 0
	rise_flag = '0'
	pressure_low_count = 0
	pressure_count = 0
	patient_set = 0
	exhale_time_last = time()
	TOT_last = time()
	ratio_set = 0
	inhale_loop = "0"
	del MVi_array[:]
	del MVe_array[:]
	GPIO.output(35, GPIO.LOW)
	VTi_volume = 0
	leak_percentage = 0
	loop = 1
	peep_set = 0
	flag = 0
	inhale_time = 2
	BPM = 10
	sol.ChangeDutyCycle(100)
	VTi_max = 500
	time_error_set = 0
        start_time=time()
	current_time = time()
	time_error_last = time()
	sending_time = 0.2
        try:
            while True:
		prev_mode = 34
		ABP_flag = 0
		print('we are in NIV_APAP mode')
		data= self.read_data()
#		print('mode is')
#		print(data)
		if((current_time-start_time) >= 10):
		    if(current_time-start_time >= 10):
		        try:
			    f = open("/home/pi/AgVa_5.0/backup.txt","w")
			    f.write("340")
			    f.close()
			    BIPAP_backup_flag = 1
			    backup_flag = 1
			    f = open("/home/pi/AgVa_5.0/mode.txt","w")
			    f.write(str("350"))
			    f.close()
			except:
			    print("unable to fall back to PC_IMV mode")
			try:
			    ser.write("ACK38")
			except:
			    print("unable to send ACK38")
		if(data == 1):
		    break;
#		print(inhale_time)
#		print(BPM)
#		print(PIP)
#		print(VTi_max)
		inhale_time = 3.0
		BPM = 10
		VTi_max = 500
		PIP = P_plat + 3
		cycle_time = BPM/60
		trig_flow =trigflow +  5
#		print(peep)
		print(peep_val)
		print(P_plat)
		print(P_plat_value)
		print('here the pump pressure is')
		print(pump_pressure)
	#	GPIO.output(22, GPIO.LOW)
		volFlow_rate = self.rate()
		indiff= self.ABP_pressure()
		if(indiff == -8888):
			ABP_flag = 1
			indiff = 0
		packet_exhalation = ('C@' + str(round(indiff,2)) + ',' + str(round(volFlow_rate,2)) + ',' + str(int(volume)) + ',' + str(int((TITOT)*100))+'#')
		if(sending_time > 0.02):
		    try:
			ser.write(packet_exhalation)
		    except:
			print('BT Error sending in Exhalation conditional')
		    send_last_time = time()
		sending_time =time() - send_last_time 
#		print('pump_pressure is')
#		print(pump_pressure)
                flow=self.rate()
#		print('before flow')
		print(flow)
#		flow = flow - peep_hole[int(peep_val)]
#		print('after flow')
#		print(flow)
                current_time=time()
       #         volume=0
		TOT_last = current_time - exhale_time_last
		SDP_flag = 0
		peep_val_send = indiff
		loop = 1
		time_error = time() - time_error_last
                if(flow>trig_flow):
		    pump_pressure_now = pump_pressure
		    patient_trigger_flow = flow
		    inhale_loop = "1"
		    indiff = self.ABP_pressure()
		    volume = 0
		    sol.ChangeDutyCycle(100)
		    flow_flag = 0
#		    if((pump_pressure /2) < peep):
#			motor_1.ChangeDutyCycle(peep + 2)
#			pump_pressure_now = peep +2
#		    else:
#		    print('indiff right now is')
#		    print(indiff)
#		    if(indiff <= P_plat/2):
                    motor_1.ChangeDutyCycle(pump_pressure)
		    #    flow_flag = 0
#		    else:
#			motor_1.ChangeDutyCycle(pump_pressure)
#			flow_flag = 1
#			pump_pressure_now = pump_pressure /2
                    RR_time=current_time-start_time
		    if(time_error < 0.7):
			time_error_set = 1
#			try:
#			    ser.write('ACK24')
#			except:
#			    print('unable to send data')
		    if(time_error > 0.7):
			time_error_set = 0
#			try:
#			    ser.write('ACK24')
#			except:
#			    print('unable to send data')
		    if(float(ratio) <= 0.25):
			ratio_set = 1
#			try:
#			    ser.write('ACK12')
#			except:
#			    print('unable to send data')
		    if(float(ratio) >= 0.75):
			ratio_set = 1
#			try:
#			    ser.write('ACK13')
#			except:
#			    print('unable to send data')
		    if(float(ratio) >0.25 and float(ratio) < 0.75):
			ratio_set = 0
#			try:
#			    ser.write('ACK22')
#			except:
#			    print('unable to send data')
		    if(RR_time > cycle_time):
			trigger='0'
		    elif(flow>trigflow):
			trigger='1'
                 #   indiff= self.ABP_pressure()
                  #  print(indiff)
                    time_elapsed_inhale=0
                    indiff = 0
		    Pmean=0
		    del Pmean_array[:]
		    clock_t2= time()
#		    print('now it is')
#		    print(clock_t2)
		    volume=0
		    volume_peak_inhale=0
		    sending_time = 0.2
		    send_last_time=0
		    volFlow_rate = self.rate()
		    VTi_volume=0
		    pump_flag = 0
		   # flow_flag = 0
		    current_time = time()
		    peak_insp_pressure = 0
#		    flow_zero_intialize = time() # clock starts to detect zero flow
#		    flow_zero_flag = 0
#		    flow_zero_elapsed = 0
		    peak_flow = volFlow_rate
		    rise_flag = '0'
                    while((peak_flow*0.40 <= volFlow_rate and time_elapsed_inhale <= inhale_time)  and time_elapsed_inhale <= inhale_time):
                    #    motor_1.ChangeDutyCycle(pump_pressure)
                        #motor_2.ChangeDutyCycle(pump_pressure)
                        t3 = time()
			GPIO.output(37, GPIO.HIGH)
#			if(volFlow_rate <= 2):
#			    flow_zero_flag = 1
#			    flow_zero_initialize = time()
#		        if(flow_zero_flag == 1):
#			    flow_zero_elapsed = time() - flow_zero_initialize
#			    if(flow_zero_elapsed >= )
#			    flow_zero_elapsed = time() - flow_zero_initialize 
			if(ABP_flag == 0):
                            indiff= self.ABP_pressure()
                        time_elapsed_inhale= t3-current_time
			if(indiff >= P_plat * 0.8 and rise_flag == '0'):
				rise_time = time_elapsed_inhale
				rise_flag = '1'
                   #     print(indiff)
			Pmean_array.append(indiff)
			if(SDP_flag == 0):
			    volume=self.Flow()
			    volFlow_rate=self.rate()
			    if(volFlow_rate >= peak_flow):
				peak_flow = volFlow_rate
#			if(indiff >= P_plat/2 and flow_flag == 0 and time_elapsed_inhale >= 0.3): #(time_elapsed_inhale > 0.3 and flow_flag == 0) or indiff >= 10): #or indiff >= P_plat - 4): #or volFlow_rate >= flow_plat - 10):
#			    flow_flag = 1
#			    print('CMHO reacged')
#			    print(indiff)
#			if(flow_flag == 1 and volFlow_rate <= flow_plat):
#			    motor_1.ChangeDutyCycle(pump_pressure)
#			if(volFlow_rate >= flow_plat and flow_flag ==1):
#			    flow_flag = 2
			if(volume == -9999):
			    SDP_flag = 1;
			if(sending_time > 0.02):
			    packet_inhalation =('A@' + str(round(indiff,2)) + ','+str(round(volFlow_rate,2)) + ',' + str(int(volume)) + ',' + str(trigger) + '#')
			    try:
				ser.write(packet_inhalation)
			    except:
				print('BT error send Inhalation')
#			    print('packet size isssssssssss')
#			    print(len(packet_inhalation))
#			    print('packet conatins ')
#			    print(packet_inhalation)
			    send_last_time= time()
#			if(volFlow_rate < flow_plat -3 and pump_pressure_now < pump_pressure):
#			    pump_pressure_now = pump_pressure_now + 4
#			    print('in 1st loop')
#			    motor_1.ChangeDutyCycle(pump_pressure_now)
#			elif(volFlow_rate > flow_plat + 3 and pump_pressure_now > pump_pressure):
#			    pump_pressure_now = pump_pressure_now -4
#			    print('in 2nd loop')
#			    motor_1.ChangeDutyCycle(pump_pressure_now)
#			if(flow_flag == 2):
#			    if(volFlow_rate <= flow_plat -5 and pump_pressure_now <= P_plat):
		#	   # if(pump_flag == 0):
			      #      pump_pressure_now  = pump_pressure_now + 5
			     #   else:
#				pump_pressure_now = pump_pressure_now + 1
	#		    print('the pump_pressure now is')
	#		    print(pump_pressure_now)
	#		    print('pump pressure is')
	#		    print(pump_pressure)
#			        motor_1.ChangeDutyCycle(pump_pressure_now)
#			    if(volFlow_rate >= flow_plat and pump_pressure_now >= P_plat/2): #and pump_pressure_now >( pump_pressure /2)):
#			        pump_pressure_now = pump_pressure_now - 1
#			        motor_1.ChangeDutyCycle(pump_pressure_now)
			      #  pump_flag = 1
			sending_time=t3 - send_last_time
#			print('flow is')
			print(volFlow_rate)
#			print(time_elapsed_inhale)
#			print(inhale_time)
			VTi_volume = max(volume,VTi_volume)
			peak_insp_pressure = max(peak_insp_pressure, indiff)
			volume_peak_inhale=max(volume_peak_inhale,volFlow_rate)
		    if(indiff >= (P_plat+3)):
			pressure_count = pressure_count +1
			if(pressure_count >= 3):
		            try:
		  	        ser.write('ACK07')
			    except:
		      	        print('unable to send')
		    else:
			pressure_count = 0
			try:
			    ser.write('ACK57')
			except:
			    print('unable to send')
		    if(indiff <= (P_plat)*0.5):
			pressure_low_count = pressure_low_count +1
			if(pressure_low_count >= 3):
		            try:
		  	        ser.write('ACK62')
			    except:
		      	        print('unable to send')
		    else:
			pressure_low_count = 0
			try:
			    ser.write('ACK72')
			except:
			    print('unable to send')
                    flag=1
		    loop = 0
#		    print('volume is')
#		    print(VTi_volume)
		    motor_1.ChangeDutyCycle(pump_pressure)
	#	    sleep(0.01)
#		    GPIO.output(35, GPIO.HIGH) # low level triggered relay
		    try:
		        Pmean= sum(Pmean_array)/len(Pmean_array)
		    except:
			print("zero exception error")
#		    print('Pmean is')
		    MVi_array.append(VTi_volume)
                    VTi_volume = VTi_volume - ((VTi_volume*leak_percentage)/100)
		    print('P_plat is')
		    print(int(peak_insp_pressure))
		    print(P_plat)
#		    print(Pmean)
                    RR.append(RR_time)
#		    print('volume is')
		    peak_insp_pressure = int(peak_insp_pressure)
		    if(VTi_volume > VTi_max and peak_insp_pressure > P_plat and patient_status == 1 and ABP_flag == 0 and peak_insp_pressure < (PIP+3) and  pump_pressure >= 10 and pump_pressure <= 96):
			pump_pressure = pump_pressure - 1
			print('-1')
		    elif((VTi_volume < VTi_max or peak_insp_pressure < P_plat) and patient_status == 1 and ABP_flag == 0 and peak_insp_pressure < PIP and  pump_pressure >= 10 and pump_pressure <= 95):
			pump_pressure = pump_pressure + 1
			print('+1')
#		    print(volume*0.4)
#		    print('Vpeak Inspiratory')
#		    print(volume_peak_inhale)
#		    print('trigger is')
#		    print(trigger)
#		    print('Pmean is')
#		    print(Pmean)
		    GPIO.output(37, GPIO.LOW)
		if(flag == 1):
		    patient_set = 0
		    if((leak_percentage > 95 or peak_insp_pressure <= P_plat * 0.3) and patient_set == 0 ):
			buzzer(5,1)
			patient_set = 1
			patient_status = 0
		        print('disconnection')
#			trigger = '0'
			try:
			    ser.write('ACK05')
			except:
			    print('not able to send')
		  #      GPIO.output(22, GPIO.HIGH)
		    if((leak_percentage <= 95 or peak_insp_pressure> P_plat * 0.3) and patient_set == 0):
			buzzer(5,0)
			patient_set = 1
			patient_status = 1
#			trigger = '1'
			try:
			    ser.write('ACK06')
			except:
			    print('not able to send')
                    if(len(RR) >= 5):
                        BPM = ( RR[1] + RR[2] + RR[3] + RR[4]) /4
                        BPM = ceil(60/BPM)
			if(BPM > 70):
			    try:
				ser.write('ACK09')
			    except:
				print('unable to send')
			else:
			    try:
				ser.write('ACK59')
			    except:
				print('unable to send')
 #                      print('BPM is')
  #                     print(BPM)
                        RR.pop(0)
		    if(len(MVi_array) >= 5):
		        MVi= (MVi_array[1] + MVi_array[2] + MVi_array[3] + MVi_array[4])/4
			MVi = MVi * BPM
#		        print('MVi value is')
#		        print(MVi)
		        MVi_array.pop(0)
		        packet_end_inhalation = ('B@' + str(round(peak_insp_pressure,2)) + "," + str(int(VTi_volume)) + "," + str(round(volume_peak_inhale,2)) + "," + str(round(Pmean,2)) + "," + str(round((MVi/1000),2))  + "," + str(round(patient_trigger_flow,2)) + "," + str(round(time_elapsed_inhale,2)) + "," + str(round(indiff,2)) + "," + str('0') + "," + str(round(rise_time,2)) +'#')
#		        print('packet length is')
		        print(packet_end_inhalation)
		        try:
		            ser.write(packet_end_inhalation)
		        except:
			    print('BT Errror in end of inhalation loop')
                if(flag==1):
		    time_error_last = time()
		    time_elap = 0
		    q= time()
		    volume = 0
		    inhale_loop = "0"
		    buzzer(7,0)
		    sending_time = 200
		    send_last_time = 0
		    volume_peak_exhale = 0
		    while(time_elap < 0.5):
			w=time()
			time_elap=w-q
#			if(time_elap > 0.2): # for motor relay
#			    print('turning off the relay')
#		    	    GPIO.output(35, GPIO.LOW)
#			    sleep(0.05)
#			    motor_1.ChangeDutyCycle(peep)
			if(ABP_flag == 0):
			    indiff = self.ABP_pressure()
# 			if(indiff <= (peep + 1)):
# 			    GPIO.output(35, GPIO.LOW)
# 			    print('turning off the relay')
			diff = self.SDP_pressure()
			if(SDP_flag == 0):
			    volFlow_rate = self.rate()
			    volume=self.Flow()
			if(diff == 700 or diff == -700 or SDP_flag == 1):
			    try:
				f = open("/home/pi/AgVa_5.0/mode.txt","w")
				f.write(str('11'))
				f.close()
				try:
				    ser.write('ACK30')
				except:
				    print('unable to send data')
			    except:
				print('unable to write mode')
			packet_exhalation= ('C@' + str(round(indiff,2)) + ',' + str(round(volFlow_rate,2)) +',' + str(int(volume)) + ',' + str(int((TITOT)*100)) + '#')
			if(sending_time > 0.02):
			    try:
				ser.write(packet_exhalation)
			    except:
				print('BT exhalation error')
			    send_last_time = time()
			sending_time = w- send_last_time
#			print('packet exhalation size  is')
			print(volFlow_rate)
			volume_peak_exhale= min(volume_peak_exhale, volFlow_rate)
#			print('packet_exhal;ation conatins is')
#			print(packet_exhalation)
                    #motor_2.ChangeDutyCycle(0)
                    new_time=time()
                    time_elapsed_exhale=0.0
                    diff=self.SDP_pressure()
		    time_elapsed_exhale_flow = 0
		    start_time=current_time
		    sending_time=200
		    send_last_time = 0
                    while(( diff <= 0 and time_elapsed_exhale <= 5.0) or (SDP_flag ==1 and time_elapsed_exhale<(cycle_time-inhale_time-0.5))):
                        time_elapsed_exhale=time()-new_time
			if(ABP_flag == 0):
			    indiff = self.ABP_pressure()
			if(SDP_flag == 0):
           		    volume=self.Flow()
			    volFlow_rate=self.rate()
			    diff = self.SDP_pressure()
# 			if(indiff <= (peep + 1)):
# 			    GPIO.output(35, GPIO.LOW)
# 			    print('turning off the relay')
			volume_peak_exhale=min(volume_peak_exhale,volFlow_rate)
 			if(volFlow_rate < -5):
 			    time_elapsed_exhale_flow = time_elapsed_exhale
			packet_exhalation = ('C@' + str(round(indiff,2)) + ',' + str(round(volFlow_rate,2)) + ',' + str(int(volume)) + ',' + str(int((TITOT)*100))+'#')
			if(sending_time > 0.02):
			    try:
				ser.write(packet_exhalation)
			    except:
				print('BT Error sending in Exhalation conditional')
			    send_last_time = time()
			sending_time =time() - send_last_time 
                        start_time=current_time
                    #    print(start_time)
                    flag=0
		    loop = 1
		    inhale_loop = "1"
		    exhale_time_last = time()
		    time_elapsed_exhale = time_elapsed_exhale + 0.5
		    print(volFlow_rate)
#		    print(time_elapsed_inhale /( time_elapsed_inhale + time_elapsed_exhale))
		    TITOT=(time_elapsed_inhale/(time_elapsed_inhale + time_elapsed_exhale + TOT_last))
#		    print('peak volume flow rate exhale')
#		    print(volume_peak_exhale)
# 		    if(peep_val_send >= 15):
# 			try:
# 			    ser.write('ACK08')
# 			except:
# 			    print('unable to send')
# 		    else:
# 			try:
# 			    ser.write('ACK58')
# 			except:
# 			    print('unable to send')
		    MVe_array.append(volume)
		    FiO2 = self.FiO2()
                    self.power()
		    print('FiO2 is')
		    print(FiO2)
# 		    if(toggle_switch == '1'):
		    time_elapsed_exhale = time_elapsed_exhale + 0.5
		    time_elapsed_exhale_flow = time_elapsed_exhale_flow + 0.5
# 		    else:
# 			time_elapsed_exhale = time_elapsed_exhale + 0.1
# 			time_elapsed_exhale_flow = time_elapsed_exhale_flow + 0.1
		    if(len(MVe_array)>=5):
		        MVe= (MVe_array[1] + MVe_array[2] + MVe_array[3] + MVe_array[4])/4
			MVe = MVe * BPM
		        MVe_array.pop(0)
		        try:
			    leak_percentage = ((MVi - MVe)/MVi)*100
			    if(leak_percentage <= -1):
			        leak_percentage = 0
		        except:
			    leak_percentage = 0
#		        print('MVe is')
#		        print(MVe)
		   # packet_end_exhalation = 'ABC'
		        packet_end_exhalation = ('D@' + str(round(peep_val_send,2)) + ',' + str(int(BPM)) + ',' + str(FiO2) + ',' + str(round(volume_peak_exhale,2)*-1)  + "," + str(round((MVe/1000),2)) + "," + str(leak_percentage) + "," + str(trigflow) + "," + str(volume) + "," + str(time_elapsed_exhale_flow) + '#')
#			print('string issssssssssssssssssssssssssssssssssssss')
			print(packet_end_exhalation)
		        try:
	                    ser.write(packet_end_exhalation)
		        except:
			    print('BT send error in end of exhalation')
#		    try:
#		        with open('/home/pi/AgVa_5.0/data.csv', mode='a') as data:
 #  		            data = csv.writer(data, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
#		            now = datetime.datetime.now()
#		            data.writerow([str(now),str(self.measure_temp()),str(Pmean),str(volume),str(volume_peak_inhale),str(trigger),str(BPM),str(MVi),str(ratio), str(volume_peak_exhale),str(MVe)])
#		    except:
#			print('data file open error')
        except KeyboardInterrupt:
            return
#-----------------------------------------------------------
#--------------------------------------------------------------------
    def PRVC(self):
        global peep_first,compliance_flag,volume_flag,lock,peep_factor,IH_time, EH_time, IHold, EHold,exp_press_array,exp_flow_array,peep,peep_open,time_elapsed_exhale_flow,patient_trigger_flow,rise_time, rise_flag,inhale_array,motor_factor,pressure_low_count,leak_comp_flag,running_avg,trigflow_comp, comp_flag,volume_comp,inhale_loop,pump_pressure_array,P_plat_array,compliance,compliance_array,toggle_switch,peep_open,loop,peep,peep_pwm,P_plat,P_plat_high,exhale_time_last, TOT_last,TITOT,P_plat_low,ABP_flag,pressure_count,peep_count,volume_count, SDP_flag,patient_set,flow_plat,ratio_set,patient_status,pump_pressure,pump_pressure_low, pump_pressure_high,time_error_set,thread_mode_status,time_error, time_error_last,prev_mode,P_plat_value,peep_hole, peep_val,data,ratio,RR_time,indiff, RR, BPM, MVi_array, MVi,volume, volume_peak_inhale, trigger, max_value, volume_peak_exhale, MVe_array, MVe,clock_t2
        thread_mode_status = True
	time_elapsed_exhale_flow = 0
	rise_time = 0
	volume_flag = 0
	lock = 0
	peep_factor = 2.0
	patient_trigger_flow = 0
	rise_flag = '0'
	pressure_low_count = 0
	trigflow_comp = 0
	running_avg = 0.0
	inhale_array = []
	del MVi_array[:]
	del MVe_array[:]
	del exp_press_array[:]
	del exp_flow_array[:]
	motor_factor = 0.5
	comp_flag = '0'
	volume_count = 0
	volume_comp = 0
	exhale_time_last = time()
	TOT_last = time()
	peep_count = 0
	ratio_set = 0
	VTi_volume = 0
	leak_percentage = 0
	inhale_loop = "0"
	flag = 0
	GPIO.output(35, GPIO.LOW)
	loop = 1
	pressure_count = 0
	del compliance_array[:]
	volume_set = 0
	compliance_flag = 0
	patient_set = 0
	peep_set = 0
	sol.ChangeDutyCycle(100)
	peep_val_send = 0
	temp_peep = 0
	time_error_set = 0
        start_time=time() - 5000
	time_error_last = time()
	sending_time = 0.2
        try:
            while True:
		prev_mode = 23
		print('we are in PRVC mode')
		data= self.read_data()
		print(data)
		hold_check()
		if(data == 1):
		    print("breaking the cuircuit in hereeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee")
		    break;
#		print(inhale_time)
#		print(BPM)
#		print(PIP)
#		print(VTi_max)
#		print(peep)
#		print(P_plat)
#		print(trigflow)
		indiff= self.ABP_pressure()
		exp_press_array.append(indiff)
		clock_t2 = time()
		volFlow_rate = self.rate()
#		indiff= self.ABP_pressure()
		packet_exhalation = ('C@' + str(round(indiff,2)) + ',' + str(round(volFlow_rate,2)) + ',' + str(int(volume)) + ',' + str(int((TITOT)*100))+'#')
		if(sending_time > 0.02):
		    try:
			ser.write(packet_exhalation)
		    except:
			print('BT Error sending in Exhalation conditional')
		    send_last_time = time()
		sending_time =time() - send_last_time 
#		print("THE TIME TAKEN IS")
#		print(cuurent_time - start_time)
                flow=self.rate()
	        if(indiff <= peep_val +1 and comp_flag == '1' and leak_comp_flag == '1'):
		    trigflow_comp = trigflow + running_avg + 3
		    comp_flag = '0'
		    running_avg = 0
        #        peep_val_send = temp_peep
	#	if(len(exp_press_array) > 10):
	#	    peep_val_send = exp_press_array[len(exp_press_array) - 4]
                current_time=time()
		if(leak_comp_flag == '0'):
		    trigflow_comp = trigflow
         #       del exp_press_array[:]
		TOT_last = current_time - exhale_time_last
#		peep_val_send = indiff
                if(indiff <= peep_val+2 and flow >= -10):
                    sol.ChangeDutyCycle(100)
	        if(comp_flag == '0' and flow > 1):
		    running_avg = (running_avg + flow)/2
		time_error = time() - time_error_last
		SDP_flag = 0
		loop = 1
		print("THE TIME TAKEN IS")
		print(current_time - start_time)
                if(flow >= trigflow  or (current_time-start_time)>cycle_time):
		    sol.ChangeDutyCycle(100)
                    motor_1.ChangeDutyCycle(100)
		    if(VTi_max <= 200 or P_plat <= 22):
			motor_1.ChangeDutyCycle(80)
		    RR_time=current_time-start_time
		    inhale_loop = "1"
		    volume = 0
		    GPIO.output(33, GPIO.LOW)
		    patient_trigger_flow = flow
		    del inhale_array[:]
		    leak_comp_flag = leak_comp()
		    comp_flag = '1'
		    pump_pressure_now = pump_pressure
		    GPIO.output(35, GPIO.LOW)
		    peep_val_send = temp_peep
 		    if(len(exp_press_array) > 10):
 			peep_val_send = exp_press_array[len(exp_press_array) - 4]
# 			pump_pressure_now = peep +2
 		    del exp_press_array[:]
		   # GPIO.output(19,GPIO.HIGH)
# 		    sol.ChangeDutyCycle(100)
#                     motor_1.ChangeDutyCycle(100)
# 		    if(P_plat <= 22):
# 			motor_1.ChangeDutyCycle(80)
#		    if(P_plat <= 17):
#			motor_1.ChangeDutyCycle(65)
#			pump_pressure_now = pump_pressure /2
 #                   RR_time=current_time-start_time
		    if(time_error < 0.7):
			time_error_set = 1
#			try:
#			    ser.write('ACK24')
#			except:
#			    print('unable to send data')
		    if(time_error > 0.7):
			time_error_set = 0
#			try:
#			    ser.write('ACK24')
#			except:
#			    print('unable to send data')
		    if(float(ratio) <= 0.25):
			ratio_set = 1
#			try:
#			    ser.write('ACK12')
#			except:
#			    print('unable to send data')
		    if(float(ratio) >= 0.75):
			ratio_set = 1
#			try:
#			    ser.write('ACK13')
#			except:
#			    print('unable to send data')
		    if(float(ratio) >0.25 and float(ratio) < 0.75):
			ratio_set = 0
#			try:
#			    ser.write('ACK22')
#			except:
#			    print('unable to send data')
		    if(RR_time > cycle_time):
			trigger='0'
		    else:
			trigger='1'
                    #motor_2.ChangeDutyCycle(pump_pressure)
                    indiff= self.ABP_pressure()
                  #  print(indiff)
                    time_elapsed_inhale=0
                    indiff = 0
		    volume=self.Flow()
#		    print('the value of volume is')
#		    print(volume)
		    if(volume == -9999):
			SDP_flag = 1
			print('well hrre you gooooooooooooooooooooooooooooooooo')
		    Pmean=0
		    del Pmean_array[:]
		    clock_t2= time()
#		    print('now it is')
#		    print(clock_t2)
		    volume=0
		    volume_peak_inhale=0
		    sending_time = 0.2
		    send_last_time=0
		    VTi_volume=0
		    flow_flag = 0
		    volFlow_rate = self.rate()
		    pump_flag = 0
		    pump_pressure_now = pump_pressure/2
		    peak_insp_pressure = 0
		    volume_comp = 0
		    peak_flow = volFlow_rate
		    rise_flag = '0'
                    while(indiff <= PIP and time_elapsed_inhale <= inhale_time):
                    #    motor_1.ChangeDutyCycle(pump_pressure)
                        #motor_2.ChangeDutyCycle(pump_pressure)
			GPIO.output(37, GPIO.HIGH)
                        t3 = time()
#			print("inhale loop")
                        indiff= self.ABP_pressure()
                        time_elapsed_inhale= t3-current_time
 #                       print(time_elapsed_inhale)
#			print(volume)
#			print(VTi_max)
			if(indiff >= P_plat * 0.8 and rise_flag == '0'):
				rise_time = time_elapsed_inhale
				rise_flag = '1'
#			print(indiff)
#			print(PIP)
			Pmean_array.append(indiff)
			if(SDP_flag == 0):
			    volume=self.Flow()
			    volume_comp = volume
			    volume_comp = volume_comp - ((volume_comp*leak_percentage)/100)
#			    print('hello its me')
			    volFlow_rate=self.rate()
			if(volFlow_rate > peak_flow):
			    peak_flow = volFlow_rate
			if(volFlow_rate <  peak_flow * 0.50 and time_elapsed_inhale >= 0.5 and trigger == '1'):
			    break 
			if(sending_time > 0.02):
			    packet_inhalation =('A@' + str(round(indiff,2)) + ','+str(round(volFlow_rate,2)) + ',' + str(int(volume)) + ',' + str(trigger) + '#')
			    try:
				ser.write(packet_inhalation)
			    except:
				print('BT error send Inhalation')
#			    print('packet size isssssssssss')
#			    print(len(packet_inhalation))
#			    print('packet conatins ')
#			    print(packet_inhalation)
			    send_last_time= time()
			sending_time=t3 - send_last_time
			print(pump_pressure)
			if((indiff >= P_plat*motor_factor or time_elapsed_inhale >= 0.5) and flow_flag == 0): #(time_elapsed_inhale > 0.3 and flow_flag == 0) or indiff >= 10): #or indiff >= P_plat - 4): #or volFlow_rate >= flow_plat - 10):
			    flow_flag = 1
			    motor_1.ChangeDutyCycle(pump_pressure)
#			    print('CMHO reacged')
#			    print(indiff)
# 			if(P_plat <= 17 and (indiff >= P_plat/2 or time_elapsed_inhale >= 0.2) and flow_flag == 0): #(time_elapsed_inhale > 0.3 and flow_flag == 0) or indiff >= 10): #or indiff >= P_plat - 4): #or volFlow_rate >= flow_plat - 10):
# 			    flow_flag = 1
# 			    motor_1.ChangeDutyCycle(pump_pressure)
# 			if(flow_flag == 1 and volFlow_rate <= flow_plat):
# 			    motor_1.ChangeDutyCycle(pump_pressure)
			if(volFlow_rate >= flow_plat and flow_flag ==1):
			    flow_flag = 2
			if(flow_flag == 3):
			    if(volFlow_rate <= flow_plat and pump_pressure_now <= 98):
				pump_pressure_now = pump_pressure_now + 1
			        motor_1.ChangeDutyCycle(pump_pressure_now)
			    if(volFlow_rate >= flow_plat and pump_pressure_now >= 2): #and pump_pressure_now >( pump_pressure /2)):
			        pump_pressure_now = pump_pressure_now - 1
			        motor_1.ChangeDutyCycle(pump_pressure_now)
# 			if(pump_pressure_now <= pump_pressure and volFlow_rate <= flow_plat):
# 			    if(pump_flag == 0):
# 			        pump_pressure_now  = pump_pressure_now + 1
# 			    else:
# 				pump_pressure_now = pump_pressure_now + 0.5
# 	#		    print('the pump_pressure now is')
# 	#		    print(pump_pressure_now)
# 	#		    print('pump pressure is')
# 	#		    print(pump_pressure)
# 			    motor_1.ChangeDutyCycle(pump_pressure_now)
# 			elif(volFlow_rate > flow_plat and pump_pressure_now >( pump_pressure /2)):
# 			    pump_pressure_now = pump_pressure_now - 0.5
# 			    motor_1.ChangeDutyCycle(pump_pressure_now)
# 			    pump_flag = 1
#			print(inhale_time)
			inhale_array.append(indiff)
			VTi_volume = max(volume,VTi_volume)
			peak_insp_pressure = max(peak_insp_pressure, indiff)
			volume_peak_inhale=max(volume_peak_inhale,volFlow_rate)
		    if(IHold == '1'):
			motor_1.ChangeDutyCycle(int(pump_pressure*0.9))
			GPIO.output(33, GPIO.HIGH)
			print("entering inspiratory hold")
			print(IH_time)
			instant_time = time()
			ser.write("ACK66")
		        sending_time = 0.2
		        send_last_time=0
			while(time() - instant_time < IH_time):
			    print(sending_time)
			    indiff= self.ABP_pressure()
			    volFlow_rate=self.rate()
			    if(sending_time > 0.02):
			        packet_inhalation =('A@' + str(round(indiff,2)) + ','+str(round(volFlow_rate,2)) + ',' + str(int(volume)) + ',' + str(trigger) + '#')
			        try:
				    ser.write(packet_inhalation)
			        except:
				    print('BT error send Inhalation')
			        send_last_time= time()
			    sending_time=time() - send_last_time
		    IHold = '0'
		    GPIO.output(33, GPIO.LOW)
		    if(indiff >= (PIP+2)):
			pressure_count = pressure_count +1
			if(pressure_count >= 3):
		            try:
		  	        ser.write('ACK07')
			    except:
		      	        print('unable to send')
		    else:
			pressure_count = 0
			try:
			    ser.write('ACK57')
			except:
			    print('unable to send')
		    if(indiff <= (P_plat)*0.5):
			pressure_low_count = pressure_low_count +1
			if(pressure_low_count >= 3):
		            try:
		  	        ser.write('ACK62')
			    except:
		      	        print('unable to send')
		    else:
			pressure_low_count = 0
			try:
			    ser.write('ACK72')
			except:
			    print('unable to send')
		    GPIO.output(37, GPIO.LOW)
		    MVi_array.append(VTi_volume)
		    VTi_volume = VTi_volume - ((VTi_volume*leak_percentage)/200)
                    flag=1
		    if(VTi_volume < (VTi_max*0.8) and indiff >= PIP and patient_status == 1):
		        ser.write("ACK61")
		    else:
			ser.write("ACK71")
		    if(VTi_volume < (VTi_max*0.8) and pump_pressure >= 95 and patient_status == 1):
			ser.write("ACK60")
		    else:
			ser.write("ACK70")
		    loop = 0
		    toggle_switch = str(toggle())
		    if(EHold == '1'):
			toggle_switch ='1'
	#	    print("the toffle switch in here issssssssssssssssssssssssssss " + toggle_switch)
		#    motor_1.ChangeDutyCycle(peep)
		    sleep(0.01)
		   # GPIO.output(35, GPIO.HIGH) # low level triggered relay
		    if(toggle_switch == "0"):
			print("got in the first statement")
		        motor_1.ChangeDutyCycle(peep_open)
			sol.ChangeDutyCycle(0)
		    if(toggle_switch == "1"):
			print("got in the 2nd statement")
	#		GPIO.output(33, GPIO.HIGH)
		        motor_1.ChangeDutyCycle(peep)
			sol.ChangeDutyCycle(0)
	#		sleep(0.01)
	#		GPIO.output(21, GPIO.HIGH)
		    inhale_array_length = len(inhale_array)
		    if(inhale_array_length % 2 != 0):
			inhale_array.append(0)
			inhale_array_length = inhale_array_length + 1
		    inhale_first_half = inhale_array[0:inhale_array_length/2]
		    inhale_second_half = inhale_array[inhale_array_length/2:]
		    first_max = max(inhale_first_half)
		    second_max = max(inhale_second_half)
		    if(first_max - P_plat > 0.5 and motor_factor > 0.2):
		        motor_factor = motor_factor - 0.01
			if(first_max - P_plat > 2):
			    motor_factor = motor_factor - 0.01
		    if(P_plat - first_max > 0.5 and motor_factor <= 1.0):
			motor_factor = motor_factor + 0.01
			if(P_plat - first_max > 2):
			    motor_factor = motor_factor + 0.01
		    if(VTi_volume < (VTi_max * 0.8)):
			volume_count = volume_count + 1
			if(volume_count >= 3):
			    buzzer(7,1)
			    try:
			        ser.write('ACK36')
			    except:
			        print('unable to send')
		    else:
			buzzer(7,0)
			volume_count = 0
			try:
			    ser.write('ACK46')
			except:
			    print('unable to send')
		    try:
		        Pmean= sum(Pmean_array)/len(Pmean_array)
		    except:
			print("zero exception error")
		    try:
		        if(compliance_flag == 0):
		            compliance_array.append(VTi_volume/(peak_insp_pressure - peep_val))
			    if(len(compliance_array) == 2):
			        compliance = (sum(compliance_array)/len(compliance_array)) * 0.4
			        pressure = (VTi_max/compliance) + peep_val
			        if(pressure > PIP):
				    pressure  = PIP - 2
	        	        P_plat_value=find_nearest(P_plat_array,pressure)
	        	        index= P_plat_array.index(P_plat_value)
				inst_pump_pressure = pump_pressure_array[index]
				if(inst_pump_pressure > pump_pressure):
	        	            pump_pressure = pump_pressure_array[index]  
			        compliance_flag = 1
		    except:
			print("error")
#		    print('Pmean is')
#		    print('Here it comessssssssss')
#		    print(int(VTi_volume))
#		    print(VTi_max)
#		    print(peak_insp_pressure)
#		    print(patient_status)
                    RR.append(RR_time)
#		    print(pump_pressure_high)
		    indiff = int(indiff)
		    if(VTi_volume > VTi_max  and indiff >= P_plat and peak_insp_pressure >= peep_val+5 and  compliance_flag == 1 and ABP_flag == 0 and SDP_flag == 0 and patient_status == 1 and pump_pressure >= 5 and pump_pressure <= 99):
			pump_pressure = pump_pressure - 1
#			P_plat = indiff
			print('-1')
		    elif(VTi_volume < VTi_max and ABP_flag == 0 and compliance_flag == 1 and SDP_flag == 0 and indiff < PIP and patient_status ==1 and pump_pressure >= 4 and pump_pressure <= 95):
			if((VTi_max - VTi_volume) > 100 and leak_percentage < 70 and VTi_max > 100):
			    if(((VTi_max - VTi_volume)/3 ) > 5):
				pump_pressure = pump_pressure + 5
			    else:
			        pump_pressure = pump_pressure + ((VTi_max - VTi_volume)/30)
			elif(VTi_max > 100):
			    pump_pressure = pump_pressure + 1
			if(VTi_max <= 100):
			    pump_pressure = pump_pressure + 5
#			P_plat = indiff
		    if(pump_pressure > 95):
			pump_pressure = 95
		    if(patient_status == 1 and peep_val_send < peep_val and peep <= peep_first + 10 and peep >= 4 and  peep < 90 and toggle_switch == '1'):
			peep = peep + 1
		    elif(patient_status == 1 and peep_val_send > peep_val and peep >= peep_first - 10 and peep > 5 and peep < 92 and toggle_switch == '1'):
			peep = peep - 1
			print("reducing the peeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeep")
		    print("Diagnostics")
		    print(VTi_max)
		    print(VTi_volume)
	#	    MVi_array.append(VTi_volume)
		    print(peak_insp_pressure)
		    print(patient_status)
		    print(P_plat)
		    print(pump_pressure)
		if(flag == 1):
	   	    patient_set = 0
		    if((leak_percentage > 90 or indiff <= 5)  and patient_set == 0 ):
			buzzer(5,1)
			patient_set = 1
			patient_status = 0
		        print('disconnection')
#			trigger = '0'
			try:
			    ser.write('ACK05')
			except:
			    print('not able to send')
		       # GPIO.output(22, GPIO.HIGH)
		    if((leak_percentage <= 90 or indiff > 5) and patient_set == 0):
			buzzer(5,0)
			patient_set = 1
			patient_status = 1
#			trigger = '1'
			try:
			    ser.write('ACK06')
			except:
			    print('not able to send')
                    if(len(RR) >= 5):
                        BPM = ( RR[1] + RR[2] + RR[3] + RR[4]) /4
                        BPM = ceil(60/BPM)
			if(BPM > 70):
			    try:
				ser.write('ACK09')
			    except:
				print('unable to send')
			else:
			    try:
				ser.write('ACK59')
			    except:
				print('unable to send')
 #                       print('BPM is')
  #                      print(BPM)
                        RR.pop(0)
		    if(len(MVi_array) >= 5):
		        MVi= (MVi_array[1] + MVi_array[2] + MVi_array[3] + MVi_array[4])/4
			MVi = MVi * BPM
#		        print('MVi value is')
#		        print(MVi)
		        MVi_array.pop(0)
		        packet_end_inhalation = ('B@' + str(round(peak_insp_pressure,2)) + "," + str(int(VTi_volume)) + "," + str(round(volume_peak_inhale,2)) + "," + str(round(Pmean,2)) + "," + str(round((MVi/1000),2)) + "," + str(round(patient_trigger_flow,2)) + "," + str(round(time_elapsed_inhale,2)) + "," + str(round(indiff,2)) + "," + str('0') + "," + str(round(rise_time,2)) + '#')
#		        print('packet length is')
		        print(packet_end_inhalation)
		        try:
		            ser.write(packet_end_inhalation)
		        except:
			    print('BT Errror in end of inhalation loop')
                if(flag==1):
		    time_error_last = time()
		    time_elap = 0
		    q= time()
		    del exp_flow_array[:]
		    del exp_press_array[:]
		    volume = 0
		    sol_flag = 0
		    sending_time = 200
		    inhale_loop = "0"
		    send_last_time = 0
		    volume_peak_exhale = 0
		    exhale_break = '0'
		    while(time_elap < 0.1 or (toggle_switch == "1" and time_elap < 0.3)):
			w=time()
			time_elap=w-q
#			exp_flow = self.rate()
#                        exp_flow_array.append(exp_flow)
                        indiff = self.ABP_pressure()
			exp_press_array.append(indiff)
			peak_insp_pressure = int(peak_insp_pressure)
			if(peak_insp_pressure <= 0):
			    peak_insp_pressure = 1
			ratio = 1.0
                        if(ratio > 0.8 and time_elap >= 0.2 and toggle_switch == "1" and exhale_break == '0'):
                            GPIO.output(35, GPIO.LOW)
                            sleep(0.05)
#                            motor_1.ChangeDutyCycle(peep)
			    exhale_break = '1'
                        elif(ratio >= 0.15 and ratio <= 0.4 and time_elap >= 0.4 and P_plat < 28 and toggle_switch == "1" and exhale_break == '0'):
                            GPIO.output(35, GPIO.LOW)
                            sleep(0.05)
#                            motor_1.ChangeDutyCycle(peep)
			    exhale_break = '1'
                        elif(ratio >= 0.15 and ratio <= 0.4 and time_elap >= 0.4 and P_plat >=28 and toggle_switch == "1" and exhale_break == '0'):
                            GPIO.output(35, GPIO.LOW)
                            sleep(0.05)
 #                           motor_1.ChangeDutyCycle(peep)
			    exhale_break = '1'
                        elif(ratio > 0.4 and ratio <= 0.8 and time_elap >= 0.3 and toggle_switch == "1" and exhale_break == '0'):
                            GPIO.output(35, GPIO.LOW)
                            sleep(0.05)
  #                          motor_1.ChangeDutyCycle(peep)
			    exhale_break = '1'
                        elif(ratio <0.15 and ratio >=0.00 and time_elap >= 0.4 and toggle_switch == "1" and exhale_break == '0'):
                            GPIO.output(35, GPIO.LOW)
                            sleep(0.05)
   #                         motor_1.ChangeDutyCycle(peep)
			    exhale_break = '1'
			diff = self.SDP_pressure()
			if(SDP_flag == 0):
			    volFlow_rate = self.rate()
			    volume=self.Flow()
			if(diff == 700 or diff == -700 or SDP_flag == 1):
			    try:
				f = open("/home/pi/AgVa_5.0/mode.txt","w")
				f.write(str('11'))
				f.close()
				try:
				    ser.write('ACK30')
				except:
				    print('unable to send data')
			    except:
				print('unable to write mode')
			packet_exhalation= ('C@' + str(round(indiff,2)) + ',' + str(round(volFlow_rate,2)) +',' + str(int(volume)) + ',' + str(int((TITOT)*100)) + '#')
			if(sending_time > 0.02):
			    try:
				ser.write(packet_exhalation)
			    except:
				print('BT exhalation error')
			    send_last_time = time()
			sending_time = w- send_last_time
# 		        if(toggle_switch == "1" and indiff <= peep_val and sol_flag == 0):
#                             sol.ChangeDutyCycle(100)
# 			    holding_now = time()
# 			    holding_time = 0.3
# 			    sol_flag = 1
# 			    peep_check = peep_val - trigflow - 1
# 			    if(peep_check < 0):
# 			        peep_check = 0
# 			    while( time() - holding_now <= holding_time and time_elap < 0.3):
# 				GPIO.output(33, GPIO.LOW)
# 				time_elap = time() - q
# 				indiff = self.ABP_pressure()
# 				if(indiff <= peep_check):
# 				    break;
# 			        volFlow_rate = self.rate()
# 			        volume=self.Flow()
# 				packet_exhalation= ('C@' + str(round(indiff,2)) + ',' + str(round(0,2)) +',' + str(int(volume)) + ',' + str(int((TITOT)*100)) + '#')
# 				if(1 == 1):
# 			    	    try:
# 					ser.write(packet_exhalation)
# 			    	    except:
# 					print('BT exhalation error')
# 			    	    send_last_time = time()
# 				sending_time = w- send_last_time
# 				sleep(0.02)
#			print('packet exhalation size  is')
#			print((packet_exhalation))
			volume_peak_exhale= min(volume_peak_exhale, volFlow_rate)
			print('packet_exhal;ation conatins is')
			print(packet_exhalation)
# 		    if(toggle_switch == "1" and indiff <= peep_val-peep_factor and sol_flag == 0):
#                         sol.ChangeDutyCycle(100)
# 			holding_now = time()
# 			holding_time = 0.3
# 			sol_flag = 1
# 			sending_time = 2.0
# 			send_last_time = 0
# 			while( time() - holding_now <= holding_time):
# 			    indiff = self.ABP_pressure()
# 			    volFlow_rate = self.rate()
# 			    volume=self.Flow()
# 			    packet_exhalation= ('C@' + str(round(indiff,2)) + ',' + str(round(0,2)) +',' + str(int(volume)) + ',' + str(int((TITOT)*100)) + '#')
# 			    if(1 == 1):
# 			        try:
# 				    ser.write(packet_exhalation)
# 			    	except:
# 				    print('BT exhalation error')
# 			    	send_last_time = time()
# 			    sending_time = time()- send_last_time
# 			    sleep(0.02)
		    print("------------------------------------" + str(peep))
		    temp_peep = self.ABP_pressure()
                    new_time=time()
                    time_elapsed_exhale=0.0
                    diff=self.SDP_pressure()
		    start_time=current_time
		    time_elapsed_exhale_flow = 0
		    sending_time=200
#		    sol_flag = 0
		    send_last_time = 0
                    while((time_elapsed_exhale<(cycle_time-inhale_time-0.3) and (diff <= 0 or indiff > peep_val* 1.5)) or (SDP_flag ==1 and time_elapsed_exhale<(cycle_time-inhale_time-0.5))):
                        time_elapsed_exhale=time()-new_time
#			exp_flow = self.rate()
 #                       exp_flow_array.append(exp_flow)
			indiff = self.ABP_pressure()
			exp_press_array.append(indiff)
			if((toggle_switch == "1" and sol_flag  == 0 and ((indiff <= peep_val+2 and volFlow_rate >= -10) or (indiff <= peep_val-2))) or (time_elapsed_exhale>(cycle_time-inhale_time-0.3)) ):
			    peep_check = peep_val - trigflow - 1
			    if(peep_check < 0):
			        peep_check = 0
			    if(volFlow_rate >= trigflow):
			        break;
			    sol.ChangeDutyCycle(100)
			    sol_flag  =  1
			    holding_now = time()
			    holding_time = 0.3
# 			    peep_check = trigflow - peep_val -2.0
# 			    if(peep_check < 0):
# 			        peep_check = 0
			    while( time() - holding_now <= holding_time and (time_elapsed_exhale<(cycle_time-inhale_time-0.3))):
				GPIO.output(33, GPIO.LOW)
				time_elapsed_exhale = time() - new_time
				indiff = self.ABP_pressure()
				if(volFlow_rate >= trigflow+peep_val):
				    break;
			        volFlow_rate = self.rate()
			        volume=self.Flow()
				packet_exhalation= ('C@' + str(round(indiff,2)) + ',' + str(round(0,2)) +',' + str(int(volume)) + ',' + str(int((TITOT)*100)) + '#')
				if(1 == 1):
			    	    try:
					ser.write(packet_exhalation)
			    	    except:
					print('BT exhalation error')
			    	    send_last_time = time()
				sending_time = time()- send_last_time
				sleep(0.02)
			if(SDP_flag == 0):
           		    volume=self.Flow()
			    diff = self.SDP_pressure()
			    volFlow_rate=self.rate()
			volume_peak_exhale=min(volume_peak_exhale,volFlow_rate)
 			if(volFlow_rate < -5):
 			    time_elapsed_exhale_flow = time_elapsed_exhale
			packet_exhalation = ('C@' + str(round(indiff,2)) + ',' + str(round(volFlow_rate,2)) + ',' + str(int(volume)) + ',' + str(int((TITOT)*100))+'#')
			if(sending_time > 0.02):
			    try:
				ser.write(packet_exhalation)
			    except:
				print('BT Error sending in Exhalation conditional')
			    send_last_time = time()
			sending_time =time() - send_last_time 
                        start_time=current_time
                    #    print(start_time)
                    if(int(peep_val_send) >=  peep_val - 1 and int(peep_val_send) <= peep_val+1):
                        lock = 1
                    elif(int(peep_val_send) > peep_val+1 or int(peep_val_send) < peep_val -1):
                        lock = 0
		    if(EHold == '1'):
			GPIO.output(33, GPIO.HIGH)
			print("Entering expiratory hold")
			instant_time = time()
			ser.write("ACK67")
		        sending_time = 0.2
		        send_last_time=0
			while(time() - instant_time < EH_time):
			    indiff= self.ABP_pressure()
			    print(time() - instant_time)
			    volFlow_rate=self.rate()
			    if(sending_time > 0.02):
			        packet_exhalation = ('C@' + str(round(indiff,2)) + ',' + str(round(volFlow_rate,2)) + ',' + str(int(volume)) + ',' + str(int((TITOT)*100))+'#')
			        try:
				    ser.write(packet_exhalation)
			        except:
				    print('BT error send Inhalation')
			        send_last_time= time()
			    sending_time=time() - send_last_time
			peep_val_send = indiff
			GPIO.output(33, GPIO.LOW)
		    EHold = '0'
#		    flow_for_peep = 0.25 * (min(exp_flow_array))
 #                   print(flow_for_peep)
  #                  value1 = find_nearest(exp_flow_array , flow_for_peep)
   #                 peep_array_index = exp_flow_array.index(value1)
#                    peep_val_send = exp_press_array[peep_array_index]
                    flag=0
		    inhale_loop = "1"
		    loop = 1
		    exhale_time_last = time()
		    time_elapsed_exhale = time_elapsed_exhale + 0.5
#		    peep_val_send = temp_peep
#		    print(time_elapsed_inhale /( time_elapsed_inhale + time_elapsed_exhale))
		    TITOT=(time_elapsed_inhale/(time_elapsed_inhale + time_elapsed_exhale + TOT_last))
#		    if(len(exp_press_array) > 10):
#		        peep_val_send =  exp_press_array[len(exp_press_array) - 3]
		    if(int(peep_val_send) <= peep_val and patient_status == 1 and toggle_switch == '1' and lock == 0):
			peep_factor = peep_factor - (peep_val - peep_val_send)*0.1
		    if(int(peep_val_send) > peep_val and patient_status == 1 and toggle_switch == '1' and lock == 0) :
			peep_factor = peep_factor + (peep_val_send - peep_val)*0.1
		    if(int(peep_val_send) > peep_val+2):
			peep_count = peep_count + 1
			if(leak_percentage > 20 and trigger == '0'):
			    ser.write("ACK39")
			if(peep_count >=3 and peep_set == 0):
			    buzzer(4,1)
			    peep_set = 1
			    try:
			        ser.write('ACK08')
			    except:
			        print('unable to send')
		    else:
			if(peep_set == 1):
			    buzzer(4,0)
			    peep_set = 0
			peep_count = 0
			ser.write("ACK49")
			try:
			    ser.write('ACK58')
			except:
			    print('unable to send')
		    MVe_array.append(volume)
		    FiO2 = self.FiO2()
                    self.power()
		    print('FiO2 is')
		    print(FiO2)
		    if(toggle_switch == '1'):
		        time_elapsed_exhale = time_elapsed_exhale + 0.4
			time_elapsed_exhale_flow = time_elapsed_exhale_flow + 0.4
		    else:
			time_elapsed_exhale = time_elapsed_exhale + 0.1
			time_elapsed_exhale_flow = time_elapsed_exhale_flow + 0.1
		    if(len(MVe_array)>=5):
		        MVe= (MVe_array[1] + MVe_array[2] + MVe_array[3] + MVe_array[4])/4
			MVe = MVe * BPM
		        MVe_array.pop(0)
		        try:
			    leak_percentage = ((MVi - MVe)/MVi)*100
			    if(leak_percentage <= -1):
			        leak_percentage = 0
		        except:
			    leak_percentage = 0
#		        print('MVe is')
#		        print(MVe)
		   # packet_end_exhalation = 'ABC'
		        packet_end_exhalation = ('D@' + str(round(peep_val_send,2)) + ',' + str(int(BPM)) + ',' + str(FiO2) + ',' + str(round(volume_peak_exhale,2)*-1)  + "," + str(round((MVe/1000),2)) + "," + str(leak_percentage) + "," + str(trigflow_comp - trigflow) + "," + str(volume)  + "," + str(time_elapsed_exhale_flow) + '#')
#			print('string issssssssssssssssssssssssssssssssssssss')
			print(packet_end_exhalation)
		        try:
	                    ser.write(packet_end_exhalation)
		        except:
			    print('BT send error in end of exhalation')
	#	    try:
	#	        with open('/home/pi/AgVa_5.0/data.csv', mode='a') as data:
   	#	            data = csv.writer(data, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
	#	            now = datetime.datetime.now()
	#	            data.writerow([str(now),str(self.measure_temp()),str(Pmean),str(volume),str(volume_peak_inhale),str(trigger),str(BPM),str(MVi),str(ratio), str(volume_peak_exhale),str(MVe)])
	#	    except:
	#		print('data file open error')
        except KeyboardInterrupt:
            return

            return
#--------------------------------------------------------

    def VCV(self):
        global peep_first,compliance_flag,volume_flag,lock,peep_factor,IH_time, EH_time, IHold, EHold,exp_flow_array,exp_press_array,peep,peep_open,time_elapsed_exhale_flow,patient_trigger_flow,rise_time, rise_flag,inhale_array,motor_factor,pressure_low_count,leak_comp_flag,running_avg,trigflow_comp, comp_flag,volume_comp,inhale_loop,pump_pressure_array,P_plat_array,compliance,compliance_array,toggle_switch,peep_open,loop,peep,peep_pwm,P_plat,P_plat_high,exhale_time_last, TOT_last,TITOT,P_plat_low,ABP_flag,pressure_count,peep_count,volume_count, SDP_flag,patient_set,flow_plat,ratio_set,patient_status,pump_pressure,pump_pressure_low, pump_pressure_high,time_error_set,thread_mode_status,time_error, time_error_last,prev_mode,P_plat_value,peep_hole, peep_val,data,ratio,RR_time,indiff, RR, BPM, MVi_array, MVi,volume, volume_peak_inhale, trigger, max_value, volume_peak_exhale, MVe_array, MVe,clock_t2
        thread_mode_status = True
	time_elapsed_exhale_flow = 0
	pressure_low_count = 0
	lock = 0
	volume_flag = 0
	patient_trigger_flow = 0
	rise_time = 0
	peep_factor = 2.0
	rise_flag = '0'
	trigflow_comp = 0
	del MVi_array[:]
	del MVe_array[:]
	del exp_press_array[:]
	del exp_flow_array[:]
	running_avg = 0.0
	inhale_array = []
	motor_factor = 0.5
	comp_flag = '0'
	volume_count = 0
	volume_comp = 0
	exhale_time_last = time()
	TOT_last = time()
	peep_count = 0
	ratio_set = 0
	loop = 1
	inhale_loop = "0"
	pressure_count = 0
	GPIO.output(35, GPIO.LOW)
	VTi_volume = 0
	leak_percentage = 0
	volume_set = 0
	flag = 0
	compliance_flag = 0
	patient_set = 0
	peep_set = 0
	del compliance_array[:]
	sol.ChangeDutyCycle(100)
	peep_val_send = 0
	temp_peep = 0
	time_error_set = 0
        start_time=time() - 5000
	time_error_last = time()
	sending_time = 0.2
        try:
            while True:
		prev_mode = 26
		print('we are in VCV mode')
		data= self.read_data()
		hold_check()
	#	print(data)
		if(data == 1):
		    print("breaking the cuircuit in hereeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee")
		    break;
#		print(inhale_time)
#		print(BPM)
#		print(PIP)
#		print(VTi_max)
#		print(peep)
#		print(P_plat)
#		print(trigflow)
		indiff= self.ABP_pressure()
		exp_press_array.append(indiff)
		clock_t2 = time()
		volFlow_rate = self.rate()
	#	indiff= self.ABP_pressure()
		packet_exhalation = ('C@' + str(round(indiff,2)) + ',' + str(round(volFlow_rate,2)) + ',' + str(int(volume)) + ',' + str(int((TITOT)*100))+'#')
		if(sending_time > 0.02):
		    try:
			ser.write(packet_exhalation)
		    except:
			print('BT Error sending in Exhalation conditional')
		    send_last_time = time()
		sending_time =time() - send_last_time 
#		print("THE TIME TAKEN IS")
#		print(cuurent_time - start_time)
                flow=self.rate()
	        if(indiff <= peep_val+1 and comp_flag == '1' and leak_comp_flag == '1'):
		    trigflow_comp = trigflow + running_avg + 3
		    comp_flag = '0'
		    running_avg = 0
#                flow=self.rate()
#		flow = flow - peep_hole[int(peep_val)]
                current_time=time()
		if(leak_comp_flag == '0'):
		    trigflow_comp = trigflow
            #    volume=0
		TOT_last = current_time - exhale_time_last
	#	peep_val_send = indiff
                if(indiff <= peep_val+2 and flow >= -10):
                    sol.ChangeDutyCycle(100)
	        if(comp_flag == '0' and flow > 1):
		    running_avg = (running_avg + flow)/2
		time_error = time() - time_error_last
		SDP_flag = 0
		loop = 1
#		print("THE TIME TAKEN IS")
#		print(current_time - start_time)
                if(flow >= trigflow  or (current_time-start_time)>cycle_time):
		    sol.ChangeDutyCycle(100)
                    motor_1.ChangeDutyCycle(100)
		    if(VTi_max <= 200):
			motor_1.ChangeDutyCycle(pump_pressure)
		    RR_time=current_time-start_time
		    pump_pressure_now = pump_pressure
		    patient_trigger_flow = flow
		    GPIO.output(33, GPIO.LOW)
		    inhale_loop = "1"
		    del inhale_array[:]
		    volume = 0
		    leak_comp_flag = leak_comp()
		    comp_flag = '1'
		    GPIO.output(35, GPIO.LOW)
	#	    GPIO.output(35,GPIO.HIGH)
 		    peep_val_send = temp_peep
 		    if(len(exp_press_array) > 10):
 			peep_val_send = exp_press_array[len(exp_press_array) - 4]
 		    del exp_press_array[:]
		   # GPIO.output(19,GPIO.HIGH)
# 		    sol.ChangeDutyCycle(100)
#                     motor_1.ChangeDutyCycle(100)
# 		    if(P_plat <= 22):
# 			motor_1.ChangeDutyCycle(80)
#		    if(P_plat <= 17):
#			motor_1.ChangeDutyCycle(65)
#			pump_pressure_now = pump_pressure /2
 #                   RR_time=current_time-start_time
		    if(time_error < 0.7):
			time_error_set = 1
#			try:
#			    ser.write('ACK24')
#			except:
#			    print('unable to send data')
		    if(time_error > 0.7):
			time_error_set = 0
#			try:
#			    ser.write('ACK24')
#			except:
#			    print('unable to send data')
		    if(float(ratio) <= 0.25):
			ratio_set = 1
#			try:
#			    ser.write('ACK12')
#			except:
#			    print('unable to send data')
		    if(float(ratio) >= 0.75):
			ratio_set = 1
#			try:
#			    ser.write('ACK13')
#			except:
#			    print('unable to send data')
		    if(float(ratio) >0.25 and float(ratio) < 0.75):
			ratio_set = 0
#			try:
#			    ser.write('ACK22')
#			except:
#			    print('unable to send data')
		    if(RR_time > cycle_time):
			trigger='0'
		    else:
			trigger='1'
                    #motor_2.ChangeDutyCycle(pump_pressure)
                    indiff= self.ABP_pressure()
                  #  print(indiff)
                    time_elapsed_inhale=0
                    indiff = 0
		    volume=self.Flow()
#		    print('the value of volume is')
#		    print(volume)
		    if(volume == -9999):
			SDP_flag = 1
			print('well hrre you gooooooooooooooooooooooooooooooooo')
		    Pmean=0
		    del Pmean_array[:]
		    clock_t2= time()
		    volume_flag = 0
		    volume_reach_time = 0
		    volume=0
		    volume_peak_inhale=0
		    sending_time = 0.2
		    send_last_time=0
		    VTi_volume=0
		    flow_flag = 0
		    volFlow_rate = self.rate()
		    pump_flag = 0
		    pump_pressure_now = pump_pressure/2
		    peak_insp_pressure = 0
		    volume_comp = 0
		    peak_flow = volFlow_rate
		    rise_flag = '0'
                    while((( trigger == '0' or volume_comp <= VTi_max) or (volume_comp >= VTi_max and volFlow_rate > peak_flow * 0.20 and trigger == '1')) and indiff < PIP and time_elapsed_inhale <= inhale_time):
                    #    motor_1.ChangeDutyCycle(pump_pressure)
                        #motor_2.ChangeDutyCycle(pump_pressure)
			GPIO.output(37, GPIO.HIGH)
                        t3 = time()
#			print("inhale loop")
                        indiff= self.ABP_pressure()
                        time_elapsed_inhale= t3-current_time
  #                      volume_reach_time = time_elapsed_inhale
#			print(volume)
#			print(VTi_max)
#			print(indiff)
#			print(PIP)
			Pmean_array.append(indiff)
			if(indiff >= P_plat * 0.8 and rise_flag == '0'):
				rise_time = time_elapsed_inhale
				rise_flag = '1'
			if(SDP_flag == 0):
			    volume=self.Flow()
			    volume_comp = volume
			    volume_comp = volume_comp - ((volume_comp*leak_percentage)/100)
#			    print('hello its me')
			    volFlow_rate=self.rate()
			if(volFlow_rate > peak_flow):
			    peak_flow = volFlow_rate
# 			if(volFlow_rate <  peak_flow * 0.25 and time_elapsed_inhale >= 0.5 and trigger == '1'):
# 			    break 
			if(sending_time > 0.02):
			    packet_inhalation =('A@' + str(round(indiff,2)) + ','+str(round(volFlow_rate,2)) + ',' + str(int(volume)) + ',' + str(trigger) + '#')
			    try:
				ser.write(packet_inhalation)
			    except:
				print('BT error send Inhalation')
#			    print('packet size isssssssssss')
#			    print(len(packet_inhalation))
#			    print('packet conatins ')
#			    print(packet_inhalation)
			    send_last_time= time()
			sending_time=t3 - send_last_time
#			print('inhale_time is')
			if((indiff >= P_plat*motor_factor or time_elapsed_inhale >= 0.5) and flow_flag == 0): #(time_elapsed_inhale > 0.3 and flow_flag == 0) or indiff >= 10): #or indiff >= P_plat - 4): #or volFlow_rate >= flow_plat - 10):
			    flow_flag = 1
			    motor_1.ChangeDutyCycle(pump_pressure)
#			    print('CMHO reacged')
#			    print(indiff)
# 			if(P_plat <= 17 and (indiff >= P_plat/2 or time_elapsed_inhale >= 0.2) and flow_flag == 0): #(time_elapsed_inhale > 0.3 and flow_flag == 0) or indiff >= 10): #or indiff >= P_plat - 4): #or volFlow_rate >= flow_plat - 10):
		        if(volume_comp >= VTi_max and volume_flag == 0 and trigger == '0'):
		    	    volume_reach_time = time_elapsed_inhale
		    	    GPIO.output(33, GPIO.HIGH)
		    	    volume_flag = 1
# 			if(flow_flag == 1 and volFlow_rate <= flow_plat):
# 			    motor_1.ChangeDutyCycle(pump_pressure)
			if(volFlow_rate >= flow_plat and flow_flag ==1):
			    flow_flag = 2
			if(flow_flag == 3):
			    if(volFlow_rate <= flow_plat and pump_pressure_now <= 98):
				pump_pressure_now = pump_pressure_now + 1
			        motor_1.ChangeDutyCycle(pump_pressure_now)
			    if(volFlow_rate >= flow_plat and pump_pressure_now >= 2): #and pump_pressure_now >( pump_pressure /2)):
			        pump_pressure_now = pump_pressure_now - 1
			        motor_1.ChangeDutyCycle(pump_pressure_now)
# 			if(pump_pressure_now <= pump_pressure and volFlow_rate <= flow_plat):
# 			    if(pump_flag == 0):
# 			        pump_pressure_now  = pump_pressure_now + 1
# 			    else:
# 				pump_pressure_now = pump_pressure_now + 0.5
# 	#		    print('the pump_pressure now is')
# 	#		    print(pump_pressure_now)
# 	#		    print('pump pressure is')
# 	#		    print(pump_pressure)
# 			    motor_1.ChangeDutyCycle(pump_pressure_now)
# 			elif(volFlow_rate > flow_plat and pump_pressure_now >( pump_pressure /2)):
# 			    pump_pressure_now = pump_pressure_now - 0.5
# 			    motor_1.ChangeDutyCycle(pump_pressure_now)
# 			    pump_flag = 1
#			print(inhale_time)
			inhale_array.append(indiff)
			VTi_volume = max(volume,VTi_volume)
			peak_insp_pressure = max(peak_insp_pressure, indiff)
			volume_peak_inhale=max(volume_peak_inhale,volFlow_rate)
		    if(IHold == '1'):
			motor_1.ChangeDutyCycle(int(pump_pressure*0.9))
			GPIO.output(33, GPIO.HIGH)
			print("entering inspiratory hold")
			print(IH_time)
			ser.write("ACK66")
			instant_time = time()
		        sending_time = 0.2
		        send_last_time=0
			while(time() - instant_time < IH_time):
			    print(sending_time)
			    indiff= self.ABP_pressure()
			    volFlow_rate=self.rate()
			    if(sending_time > 0.02):
			        packet_inhalation =('A@' + str(round(indiff,2)) + ','+str(round(volFlow_rate,2)) + ',' + str(int(volume)) + ',' + str(trigger) + '#')
			        try:
				    ser.write(packet_inhalation)
			        except:
				    print('BT error send Inhalation')
			        send_last_time= time()
			    sending_time=time() - send_last_time
		    IHold = '0'
		    GPIO.output(33, GPIO.LOW)
		    if(indiff >= (PIP+2)):
			pressure_count = pressure_count +1
			if(pressure_count >= 3):
		            try:
		  	        ser.write('ACK07')
			    except:
		      	        print('unable to send')
		    else:
			pressure_count = 0
			try:
			    ser.write('ACK57')
			except:
			    print('unable to send')
		    if(indiff <= (P_plat)*0.5):
			pressure_low_count = pressure_low_count +1
			if(pressure_low_count >= 3):
		            try:
		  	        ser.write('ACK62')
			    except:
		      	        print('unable to send')
		    else:
			pressure_low_count = 0
			try:
			    ser.write('ACK72')
			except:
			    print('unable to send')
		    GPIO.output(37, GPIO.LOW)
		    MVi_array.append(VTi_volume)
		    VTi_volume = VTi_volume - ((VTi_volume*leak_percentage)/200)
                    flag=1
		    loop = 0
		    if(VTi_volume < (VTi_max*0.8) and indiff >= PIP and patient_status == 1):
		        ser.write("ACK61")
		    else:
			ser.write("ACK71")
		    if(VTi_volume < (VTi_max*0.8) and pump_pressure >= 95 and patient_status == 1):
			ser.write("ACK60")
		    else:
			ser.write("ACK70")
		    toggle_switch = str(toggle())
		    if(EHold == '1'):
			toggle_switch ='1'

	#	    print("the toffle switch in here issssssssssssssssssssssssssss " + toggle_switch)
		#    motor_1.ChangeDutyCycle(peep)
		    sleep(0.01)
		   # GPIO.output(35, GPIO.HIGH) # low level triggered relay
		    if(toggle_switch == "0"):
#			print("got in the first statement")
		        motor_1.ChangeDutyCycle(peep_open)
			sol.ChangeDutyCycle(0)
		    if(toggle_switch == "1"):
#			print("got in the 2nd statement")
	#		GPIO.output(33, GPIO.HIGH)
		        motor_1.ChangeDutyCycle(peep)
			sol.ChangeDutyCycle(0)
	#		sleep(0.01)
	#		GPIO.output(21, GPIO.HIGH)
		    inhale_array_length = len(inhale_array)
		    if(inhale_array_length % 2 != 0):
			inhale_array.append(0)
			inhale_array_length = inhale_array_length + 1
		    inhale_first_half = inhale_array[0:inhale_array_length/2]
		    inhale_second_half = inhale_array[inhale_array_length/2:]
		    first_max = max(inhale_first_half)
		    second_max = max(inhale_second_half)
		    if(first_max - P_plat > 0.5 and motor_factor > 0.2):
		        motor_factor = motor_factor - 0.01
			if(first_max - P_plat > 2):
			    motor_factor = motor_factor - 0.01
		    if(P_plat - first_max > 0.5 and motor_factor <= 1.0):
			motor_factor = motor_factor + 0.01
			if(P_plat - first_max > 2):
			    motor_factor = motor_factor + 0.01
		    if(VTi_volume < (VTi_max * 0.8)):
			volume_count = volume_count + 1
			if(volume_count >= 3):
			    buzzer(7,1)
			    try:
			        ser.write('ACK36')
			    except:
			        print('unable to send')
		    else:
			buzzer(7,0)
			volume_count = 0
			try:
			    ser.write('ACK46')
			except:
			    print('unable to send')
		    try:
		        Pmean= sum(Pmean_array)/len(Pmean_array)
		    except:
			print("zero exception error")
		    try:
		        if(compliance_flag == 0):
		            compliance_array.append(VTi_volume/(peak_insp_pressure - peep_val))
			    if(len(compliance_array) == 2):
			        compliance = (sum(compliance_array)/len(compliance_array)) * 0.75
			        pressure = (VTi_max/compliance) + peep_val
			        if(pressure > PIP):
				    pressure  = PIP - 2
	        	        P_plat_value=find_nearest(P_plat_array,pressure)
	        	        index= P_plat_array.index(P_plat_value)
				inst_pump_pressure = pump_pressure_array[index]
				if(inst_pump_pressure > pump_pressure):
	        	            pump_pressure = pump_pressure_array[index]  
			        compliance_flag = 1
		    except:
			print("error")
#		    print('Pmean is')
#		    print('Here it comessssssssss')
#		    print(int(VTi_volume))
#		    print(VTi_max)
#		    print(peak_insp_pressure)
#		    print(patient_status)
                    RR.append(RR_time)
#		    print(pump_pressure_high)
		    indiff = int(indiff)
		    if((VTi_volume > VTi_max*1.1 or (volume_reach_time < inhale_time*0.8 and trigger == '0' and VTi_volume >= VTi_max))  and ABP_flag == 0 and SDP_flag == 0 and peak_insp_pressure >= peep_val+5 and patient_status == 1 and pump_pressure >= 5 and pump_pressure <= 99):
			pump_pressure = pump_pressure - 1
			P_plat = indiff
#			print('-1')
		    elif((VTi_volume < VTi_max or (volume_reach_time >= inhale_time*0.8 and trigger == '0'))  and ABP_flag == 0 and SDP_flag == 0 and indiff < PIP and patient_status ==1 and pump_pressure >= 4 and pump_pressure <= 95):
			pump_pressure = pump_pressure + 1
# 			if((VTi_max - VTi_volume) > 100 and leak_percentage < 70 and VTi_max > 100):
# 			    if(((VTi_max - VTi_volume)/3 ) > 5):
# 				pump_pressure = pump_pressure + 5
# 			    else:
# 			        pump_pressure = pump_pressure + ((VTi_max - VTi_volume)/30)
# 			elif(VTi_max > 100):
# 			    pump_pressure = pump_pressure + 1
# 			if(VTi_max <= 100):
# 			    pump_pressure = pump_pressure + 5
			P_plat = indiff
		    if(pump_pressure > 95):
			pump_pressure = 95
		    if(patient_status == 1 and peep_val_send < peep_val and peep <= peep_first + 10 and peep >= 4 and  peep < 90 and toggle_switch == '1'):
			peep = peep + 1
		    elif(patient_status == 1 and peep_val_send > peep_val and peep >= peep_first - 10 and peep > 5 and peep < 92 and toggle_switch == '1'):
			peep = peep - 1
#			print("reducing the peeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeep")
#		    print("Diagnostics")
#		    print(VTi_max)
#		    print(VTi_volume)
#	#	    MVi_array.append(VTi_volume)
#		    print(peak_insp_pressure)
#		    print(patient_status)
#		    print(P_plat)
#		    print(pump_pressure)
		if(flag == 1):
	   	    patient_set = 0
		    if((leak_percentage > 90 or indiff <= 5)  and patient_set == 0 ):
			buzzer(5,1)
			patient_set = 1
			patient_status = 0
		        print('disconnection')
#			trigger = '0'
			try:
			    ser.write('ACK05')
			except:
			    print('not able to send')
		       # GPIO.output(22, GPIO.HIGH)
		    if((leak_percentage <= 90 or indiff > 5) and patient_set == 0):
			buzzer(5,0)
			patient_set = 1
			patient_status = 1
#			trigger = '1'
			try:
			    ser.write('ACK06')
			except:
			    print('not able to send')
                    if(len(RR) >= 5):
                        BPM = ( RR[1] + RR[2] + RR[3] + RR[4]) /4
                        BPM = ceil(60/BPM)
			if(BPM > 70):
			    try:
				ser.write('ACK09')
			    except:
				print('unable to send')
			else:
			    try:
				ser.write('ACK59')
			    except:
				print('unable to send')
 #                       print('BPM is')
  #                      print(BPM)
                        RR.pop(0)
		    if(len(MVi_array) >= 5):
		        MVi= (MVi_array[1] + MVi_array[2] + MVi_array[3] + MVi_array[4])/4
			MVi = MVi * BPM
#		        print('MVi value is')
#		        print(MVi)
		        MVi_array.pop(0)
		        packet_end_inhalation = ('B@' + str(round(peak_insp_pressure,2)) + "," + str(int(VTi_volume)) + "," + str(round(volume_peak_inhale,2)) + "," + str(round(Pmean,2)) + "," + str(round((MVi/1000),2)) + "," + str(round(patient_trigger_flow,2)) + "," + str(round(time_elapsed_inhale,2)) + "," + str(round(indiff,2)) + "," + str('0') + "," + str(round(rise_time,2)) + '#')
#		        print('packet length is')
#		        print(packet_end_inhalation)
		        try:
		            ser.write(packet_end_inhalation)
		        except:
			    print('BT Errror in end of inhalation loop')
                if(flag==1):
		    time_error_last = time()
		    time_elap = 0
		    q= time()
		    del exp_press_array[:]
		    del exp_flow_array[:]
		    inhale_loop = "0"
		    volume = 0
		    sol_flag = 0
		    sending_time = 200
		    send_last_time = 0
		    volume_peak_exhale = 0
		    exhale_break = '0'
		    while(time_elap < 0.1 or (toggle_switch == "1" and time_elap < 0.3)):
			w=time()
			time_elap=w-q
#			exp_flow = self.rate()
#                        exp_flow_array.append(exp_flow)
                        indiff = self.ABP_pressure()
			exp_press_array.append(indiff)
			peak_insp_pressure = int(peak_insp_pressure)
			if(peak_insp_pressure <= 0):
			    peak_insp_pressure = 1
			ratio = 1.0
                        if(ratio > 0.8 and time_elap >= 0.2 and toggle_switch == "1" and exhale_break == '0'):
                            GPIO.output(35, GPIO.LOW)
                            sleep(0.05)
#                            motor_1.ChangeDutyCycle(peep)
			    exhale_break = '1'
                        elif(ratio >= 0.15 and ratio <= 0.4 and time_elap >= 0.4 and P_plat < 28 and toggle_switch == "1" and exhale_break == '0'):
                            GPIO.output(35, GPIO.LOW)
                            sleep(0.05)
 #                           motor_1.ChangeDutyCycle(peep)
			    exhale_break = '1'
                        elif(ratio >= 0.15 and ratio <= 0.4 and time_elap >= 0.4 and P_plat >=28 and toggle_switch == "1" and exhale_break == '0'):
                            GPIO.output(35, GPIO.LOW)
                            sleep(0.05)
  #                          motor_1.ChangeDutyCycle(peep)
			    exhale_break = '1'
                        elif(ratio > 0.4 and ratio <= 0.8 and time_elap >= 0.3 and toggle_switch == "1" and exhale_break == '0'):
                            GPIO.output(35, GPIO.LOW)
                            sleep(0.05)
   #                         motor_1.ChangeDutyCycle(peep)
			    exhale_break = '1'
                        elif(ratio <0.15 and ratio >=0.00 and time_elap >= 0.4 and toggle_switch == "1" and exhale_break == '0'):
                            GPIO.output(35, GPIO.LOW)
                            sleep(0.05)
    #                        motor_1.ChangeDutyCycle(peep)
			    exhale_break = '1'
			diff = self.SDP_pressure()
			if(SDP_flag == 0):
			    volFlow_rate = self.rate()
			    volume=self.Flow()
			if(diff == 700 or diff == -700 or SDP_flag == 1):
			    try:
				f = open("/home/pi/AgVa_5.0/mode.txt","w")
				f.write(str('11'))
				f.close()
				try:
				    ser.write('ACK30')
				except:
				    print('unable to send data')
			    except:
				print('unable to write mode')
			packet_exhalation= ('C@' + str(round(indiff,2)) + ',' + str(round(volFlow_rate,2)) +',' + str(int(volume)) + ',' + str(int((TITOT)*100)) + '#')
			if(sending_time > 0.02):
			    try:
				ser.write(packet_exhalation)
			    except:
				print('BT exhalation error')
			    send_last_time = time()
			sending_time = w- send_last_time
# 		        if(toggle_switch == "1" and indiff <= peep_val and sol_flag == 0 ):
#                             sol.ChangeDutyCycle(100)
# 			    holding_now = time()
# 			    sol_flag = 1
# 			    holding_time = 0.3
# 			    peep_check = peep_val - trigflow - 1
# 			    if(peep_check < 0):
# 			        peep_check = 0
# 			    while( time() - holding_now <= holding_time and time_elap <0.3):
# 				time_elap = time() - q
# 				GPIO.output(33, GPIO.LOW)
# 				indiff = self.ABP_pressure()
# 				if(indiff <= peep_check):
# 				    break;
# 			        volFlow_rate = self.rate()
# 			        volume=self.Flow()
# 				packet_exhalation= ('C@' + str(round(indiff,2)) + ',' + str(round(0,2)) +',' + str(int(volume)) + ',' + str(int((TITOT)*100)) + '#')
# 				if(1 == 1):
# 			    	    try:
# 					ser.write(packet_exhalation)
# 			    	    except:
# 					print('BT exhalation error')
# 			    	    send_last_time = time()
# 				sending_time = w- send_last_time
# 				sleep(0.02)
			sending_time = w- send_last_time
#			print('packet exhalation size  is')
#			print((packet_exhalation))
			volume_peak_exhale= min(volume_peak_exhale, volFlow_rate)
#			print('packet_exhal;ation conatins is')
		    temp_peep =self.ABP_pressure() 
# 		    if(toggle_switch == "1" and indiff <= peep_val-peep_factor and sol_flag == 0):
#                         sol.ChangeDutyCycle(100)
# 			holding_now = time()
# 			sol_flag = 1
# 			holding_time = 0.3
# 			sending_time = 2.0
# 			send_last_time = 0
# 			while( time() - holding_now <= holding_time):
# 			    indiff = self.ABP_pressure()
# 			    volFlow_rate = self.rate()
# 			    volume=self.Flow()
# 			    packet_exhalation= ('C@' + str(round(indiff,2)) + ',' + str(round(0,2)) +',' + str(int(volume)) + ',' + str(int((TITOT)*100)) + '#')
# 			    if(1 == 1):
# 			        try:
# 				    ser.write(packet_exhalation)
# 			    	except:
# 				    print('BT exhalation error')
# 			    	send_last_time = time()
# 			    sending_time = time()- send_last_time
# 			    sleep(0.02)
		    #
		    print("peep pwm is-------------------------- " + str(peep))
                    new_time=time()
                    time_elapsed_exhale=0.0
                    diff=self.SDP_pressure()
		    start_time=current_time
		    sending_time=200
		    time_elapsed_exhale_flow = 0
#		    sol_flag  = 0
		    send_last_time = 0
                    while((time_elapsed_exhale<(cycle_time-inhale_time-0.3) and (diff <= 0 or indiff > peep_val* 1.5)) or (SDP_flag ==1 and time_elapsed_exhale<(cycle_time-inhale_time-0.5))):
                        time_elapsed_exhale=time()-new_time
#			exp_flow = self.rate()
 #                       exp_flow_array.append(exp_flow)
			indiff = self.ABP_pressure()
			exp_press_array.append(indiff)
			if((toggle_switch == "1"  and sol_flag == 0 and ((indiff <= peep_val+2 and volFlow_rate >= -10) or (indiff <= peep_val-2))) or (time_elapsed_exhale>(cycle_time-inhale_time-0.3))):
			    peep_check = peep_val - trigflow - 1
			    if(peep_check < 0):
			        peep_check = 0
			    if(volFlow_rate >= trigflow):
			        break;
			    sol.ChangeDutyCycle(100)
			    sol_flag  = 1
			    holding_now = time()
			    holding_time = 0.3
# 			    peep_check = trigflow - peep_val - 2.0
# 			    if(peep_check < 0):
# 			        peep_check = 0
			    while( time() - holding_now <= holding_time and (time_elapsed_exhale<(cycle_time-inhale_time-0.3))):
				time_elapsed_exhale = time() - new_time
				GPIO.output(33, GPIO.LOW)
				indiff = self.ABP_pressure()
				if(volFlow_rate >= trigflow):
				    break;
			        volFlow_rate = self.rate()
			        volume=self.Flow()
				packet_exhalation= ('C@' + str(round(indiff,2)) + ',' + str(round(0,2)) +',' + str(int(volume)) + ',' + str(int((TITOT)*100)) + '#')
				if(1 == 1):
			    	    try:
					ser.write(packet_exhalation)
			    	    except:
					print('BT exhalation error')
			    	    send_last_time = time()
				sending_time = time()- send_last_time
				sleep(0.02)
	#		    sleep(0.3)
			if(SDP_flag == 0):
           		    volume=self.Flow()
			    diff = self.SDP_pressure()
			    volFlow_rate=self.rate()
			volume_peak_exhale=min(volume_peak_exhale,volFlow_rate)
 			if(volFlow_rate < -5):
 			    time_elapsed_exhale_flow = time_elapsed_exhale
			packet_exhalation = ('C@' + str(round(indiff,2)) + ',' + str(round(volFlow_rate,2)) + ',' + str(int(volume)) + ',' + str(int((TITOT)*100))+'#')
			if(sending_time > 0.02):
			    try:
				ser.write(packet_exhalation)
			    except:
				print('BT Error sending in Exhalation conditional')
			    send_last_time = time()
			sending_time =time() - send_last_time 
                        start_time=current_time
                    #    print(start_time)
#		    flow_for_peep = 0.25 * (min(exp_flow_array))
              #      peep_val_send = 
 #                   value1 = find_nearest(exp_flow_array , flow_for_peep)
  #                  peep_array_index = exp_flow_array.index(value1)
#                    peep_val_send = exp_press_array[peep_array_index]
                    if(int(peep_val_send) >=  peep_val - 1 and int(peep_val_send) <= peep_val+1):
                        lock = 1
                    elif(int(peep_val_send) > peep_val+1 or int(peep_val_send) < peep_val -1):
                        lock = 0
		    if(EHold == '1'):
			GPIO.output(33, GPIO.HIGH)
			print("Entering expiratory hold")
			ser.write("ACK67")
			instant_time = time()
		        sending_time = 0.2
		        send_last_time=0
			while(time() - instant_time < EH_time):
			    indiff= self.ABP_pressure()
			    print(time() - instant_time)
			    volFlow_rate=self.rate()
			    if(sending_time > 0.02):
			        packet_exhalation = ('C@' + str(round(indiff,2)) + ',' + str(round(volFlow_rate,2)) + ',' + str(int(volume)) + ',' + str(int((TITOT)*100))+'#')
			        try:
				    ser.write(packet_exhalation)
			        except:
				    print('BT error send Inhalation')
			        send_last_time= time()
			    sending_time=time() - send_last_time
			peep_val_send = indiff
			GPIO.output(33, GPIO.LOW)
		    EHold = '0'
                    flag=0
		    loop = 1
		    inhale_loop = "1"
		    exhale_time_last = time()
		    time_elapsed_exhale = time_elapsed_exhale + 0.5
#		    print("length is ::::::::::::::::::::::::::::::: " + str(len(exp_press_array)))
#		    peep_val_send = exp_press_array[len(exp_press_array)-2]
		    TITOT=(time_elapsed_inhale/(time_elapsed_inhale + time_elapsed_exhale + TOT_last))
#		    if(len(exp_press_array) < 10):
#		        peep_val_send = temp_peep
		    if(int(peep_val_send) <= peep_val and patient_status == 1 and toggle_switch == '1' and lock == 0):
			peep_factor = peep_factor - (peep_val - peep_val_send)*0.1
		    if(int(peep_val_send) > peep_val and patient_status == 1 and toggle_switch == '1' and lock == 0):
			peep_factor = peep_factor + (peep_val_send - peep_val)*0.1
		    if(int(peep_val_send) > peep_val+2):
			peep_count = peep_count + 1
			if(leak_percentage > 20 and trigger == '0'):
			    ser.write("ACK39")
			if(peep_count >=3 and peep_set == 0):
			    buzzer(4,1)
			    peep_set = 1
			    try:
			        ser.write('ACK08')
			    except:
			        print('unable to send')
		    else:
			if(peep_set == 1):
			    buzzer(4,0)
			    peep_set = 0
			ser.write("ACK49")
			peep_count = 0
			try:
			    ser.write('ACK58')
			except:
			    print('unable to send')
		    MVe_array.append(volume)
		    FiO2 = self.FiO2()
                    self.power()
#		    print('FiO2 is')
		    hold_check()
		    if(toggle_switch == '1'):
		        time_elapsed_exhale = time_elapsed_exhale + 0.4
			time_elapsed_exhale_flow = time_elapsed_exhale_flow + 0.4
		    else:
			time_elapsed_exhale = time_elapsed_exhale + 0.1
			time_elapsed_exhale_flow = time_elapsed_exhale_flow + 0.1
		    if(len(MVe_array)>=5):
		        MVe= (MVe_array[1] + MVe_array[2] + MVe_array[3] + MVe_array[4])/4
			MVe = MVe * BPM
		        MVe_array.pop(0)
		        try:
			    leak_percentage = ((MVi - MVe)/MVi)*100
			    if(leak_percentage <= -1):
			        leak_percentage = 0
		        except:
			    leak_percentage = 0
#		        print('MVe is')
#		        print(MVe)
		   # packet_end_exhalation = 'ABC'
		        packet_end_exhalation = ('D@' + str(round(peep_val_send,2)) + ',' + str(int(BPM)) + ',' + str(FiO2) + ',' + str(round(volume_peak_exhale,2)*-1)  + "," + str(round((MVe/1000),2)) + "," + str(leak_percentage) + "," + str(trigflow_comp - trigflow) + "," + str(volume)  + "," + str(time_elapsed_exhale_flow) +  '#')
#			print('string issssssssssssssssssssssssssssssssssssss')
#			print(packet_end_exhalation)
		        try:
	                    ser.write(packet_end_exhalation)
		        except:
			    print('BT send error in end of exhalation')
#		    try:
#		        with open('/home/pi/AgVa_5.0/data.csv', mode='a') as data:
 #  		            data = csv.writer(data, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
#		            now = datetime.datetime.now()
#		            data.writerow([str(now),str(self.measure_temp()),str(Pmean),str(volume),str(volume_peak_inhale),str(trigger),str(BPM),str(MVi),str(ratio), str(volume_peak_exhale),str(MVe)])
#		    except:
#			print('data file open error')
        except KeyboardInterrupt:
            return

    # MODE SELECTION FUNCTION
    def mode_selection(self,mode):
        global calib_flag,thread_mode_status,pump_pressure,init_flow_int
	P_plat_value=find_nearest(P_plat_array,P_plat)
	print('Plat is')
	print(P_plat_value)
#	print(len(pump_pressure_array))
	if(calib_flag == 0):
            try:
	        init_pressure = str(abs(int(self.ABP_pressure())))
		init_flow = 0.0
		for i in range(0,100,1):
	            init_flow = ((self.rate()/2.0)+init_flow)/2.0
		    sleep(0.002)
		    print(init_flow)
		init_flow_int = init_flow
		print("and in here")
		print(init_flow_int)
		init_flow = str(abs(int(init_flow)))
		print(init_flow)
	        zeroes = ""
	        for i in range(0,2-len(init_pressure),1):
		    zeroes = zeroes + "0"
	        init_pressure = zeroes+init_pressure
	        zeroes = ""
	        for i in range(0,3-len(init_flow),1):
		    zeroes = zeroes + "0"
	        init_flow = zeroes + init_flow
		print("FACTOR IN HERE IS")
		print(init_flow)
	        f = open("/home/pi/AgVa_5.0/init_calib.py","w")
	        print("CALIBRATION READINGS ARE")
	        print(str(init_pressure)+str(init_flow))
	        f.write(str(init_pressure)+str(init_flow))
	        f.close()
		calib_flag = 1
            except IOError:
	        print("UNABLE TO CALIBRATE")
#	print(len(P_plat_array))
	index = P_plat_array.index(P_plat_value)
	pump_pressure =pump_pressure_array[index]
	print(index)
	print(pump_pressure)
	try:
	    exists=os.path.isfile('/home/pi/AgVa_5.0/data.csv')
	except:
	    print('file read error')
	if(exists):
	    print('file exists')
	    try:
		with open('/home/pi/AgVa_5.0/data.csv',mode='a') as data:
		    data=csv.writer(data,delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
		    data.writerow(['Initialization or change in Mode'])
	    except:
		print('file opening error')
	else:
	    try:
		with open('/home/pi/AgVa_5.0/data.csv', mode='w') as data:
	            data = csv.writer(data, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
	            data.writerow(['Time','Core Temperature','Pmean','volume','volume_peak_inhale','trigger','BPM','MVi','Ti/Tot','volume_peak_exhale','Mve'])
	    except:
		print('file opening error')
      #  while not thread_mode_status:

        #    thread_mode_status=False
        print("INSIDE MODE")
#	    while True : 
	#	for dc in range(10,101,10):
	#	    motor_1.ChangeDutyCycle(dc)
	#	    sleep(2)
	#	    print(self.rate())
#		motor_1.ChangeDutyCycle(air)
 #     	        print(self.rate())
#		sleep(0.1)
#		print('product')
#		print(self.rate()*clock)
#	    clock_t2 = time()
        if mode==22:
            self.VC_IMV()
        elif mode==12:
            self.PC_IMV()
        elif mode == 21:
            self.VC_CMV()
        elif mode == 11:
            self.PC_CMV()
	elif mode == 13:
	    self.PSV()
	elif mode == 14:
	    self.Spont()
	elif mode == 31:
	    self.NIV_CPAP()
	elif mode == 32:
	    self.NIV_BIPAP()
	elif mode == 23:
	    self.PRVC()
	elif mode == 35:
	    self.NIV_AVAPS()
	elif mode == 34:
	    self.NIV_APAP()
	elif mode == 33:
	    self.NIV_APRV()
	elif mode == 24:
	    self.AIVENT()
	elif mode == 25:
	    self.ACV()
	elif mode == 26:
	    self.VCV()

    #-------------------------------------------------







#    def VC_CMV(self):

while(True):
    v = Ventilator()
    thread_mode_status = False
    try:
	f = open('/home/pi/AgVa_5.0/mode.txt',"r")
	data = f.readline()
	a = int(data[0:2])
    except:
	print('error reading mode file')
    v.mode_selection(a)
sleep(1)
print('end')
motor_1.stop()
#motor_2.stop()
GPIO.cleanup()
