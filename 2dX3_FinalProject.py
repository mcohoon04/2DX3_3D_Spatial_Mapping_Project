# Student name: Marcus Cohoon
# Student number: 400297985
# MacID: cohoom1

import serial
import math

s = serial.Serial('COM6',115200)

s.open
s.reset_output_buffer()
s.reset_input_buffer()

f = open("2dx3points.xyz", "w") 
step = 0
x = 0 # initial x-displacement (mm)
increment = 250 # x-displacement steps(mm)
num_inc = int(input("enter num:"))
count=0
while(count<num_inc):
    raw = s.readline()
    data = raw.decode("utf-8") # Decodes byte input from UART into string 
    data = data[0:-2] # Removes carriage return and newline from string
    if (data.isdigit() == True):
        angle = (step/512)*2*math.pi # Obtain angle based on motor rotation
        r = int(data)
        y = r*math.cos(angle) # Calculate y
        z = r*math.sin(angle) # Calcualte z
        print(y)
        print(z)
        f.write('{} {} {}\n'.format(x,y,z)) # Write data to .xyz file
        step = step+32
    if (step == 512): #reset number of steps after a full rotation is completed and increament x and count
        step = 0
        x = x + increment
        count=count+1
    print(data)
f.close() #close file when done so data saves
