from cmath import cos, sin
from os import truncate
from turtle import color
import math
import serial
import numpy
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

#   Asking user if previous data should be erased for the case where measurements should not be added together for a complete reconstruction
eraseFlag = input("Do you want to erase previous graph data?(Y/N)")
#   If previous data should be erased, the file is opened in write mode, overwriting previous content
if(eraseFlag == 'Y'):      
    #   Opened textfile in write mode
    f = open('graphdata.txt','w')                                            
    truncate                                                                  
    f.close() 

#   Prepared to receive data from microcontroller
s = serial.Serial('COM4', baudrate = 115200, timeout = 4)    
#   Printing which port was opened (Port4)          
print("Opening: " + s.name)

#   Reset the buffers of the UART port
s.reset_output_buffer()
s.reset_input_buffer()

#   Readying 3d graph
fig = plt.figure(figsize=(10,10))                                              
ax = fig.add_subplot(111, projection='3d')     
#   Switched the x and z labels to make plot work better                                 
ax.set_xlabel('z')                                                              
ax.set_ylabel('y')                                                              
ax.set_zlabel('x')

#   Flag to start getting data
s.write('s'.encode())                                                         
  
count = 0
z = 0
#   Initialized array to hold measurements for 1 rotation
vals = [0]*64                                                                      
#   Opened textfile in append mode
plotData = open('graphdata.txt','a')                                                   
while (count != 64):
    #   Decoded and data was entered into list
    vals[count] = float(s.readline().decode())                                     
    print(vals[count], count + 1)
    #   Converted data to string then wrote into textfile
    plotData.write(str(vals[count]))                                                      
    plotData.write('\n')                                                               
    count += 1
#   Closed textfile    
plotData.close()

#   Opened textfile in read mode
data = open("graphdata.txt","r")        
#   Turned textfile data into dataStr as a string                                        
strData = data.read()           
#   Seperated data at line breaks                                                
dataList = strData.split("\n")       
#   Created for loop to to store each measurement as a string in the list                                           
for i in range(0, len(dataList), 1):                                            
    dataList[i] = str(dataList[i])                                              
data.close()
#   Removed last line break
dataList.pop(len(dataList) - 1)                                                   
#   Converted to float type
Data = [float(i) for i in dataList]
#   Stored the number of measurements                                            
numLines = len(Data)                                                            

#   Asking user if data should be plotted
plotFlag = input("Do you want to plot the data?(Y/N)")
if(plotFlag == 'Y'):

    count = 0
    #   Created while loop to iterate for all measurements
    while(count != numLines):
        #   Checking if the iteration is on the next set of measurements, if so z is incriminented by 0.1
        if(count % 64 == 0 and count >= 64):
            z += 0.1
        #   Created line segments for x, y, and z
        x_line = [(Data[count - 1])*cos((count - 1) * 0.09817477), (Data[count]) * cos(count * 0.09817477)]
        y_line = [(Data[count-1])*sin((count-1)*0.09817477),(Data[count])*sin(count*0.09817477)]
        z_line = [z,z]
        #   Plotting points in cartesian
        ax.scatter( (Data[count])*cos(count*0.09817477) ,(Data[count])*sin(count*0.09817477),z, c = "black", s = 1)
        ax.plot(x_line ,y_line,z_line, color = 'black')
        #   Incriminating for each iteration
        count += 1

    count = 0
    z = 0
    while(count < len(Data) - 64):                                                 
        #   Created line segments for x and y
        x_line = [(Data[count])*cos((count)*0.09817477),(Data[count+64])*cos(count*0.09817477)]
        y_line = [(Data[count])*sin((count)*0.09817477),(Data[count+64])*sin(count*0.09817477)]

        #   Checking if the iteration is on the next set of measurements, if so z is incriminanted by 0.1
        if(count % 64 == 0 and count >= 64):
            z += 0.1
        #   Created line segment for z 
        z_line = [z, z + 0.1]
        ax.plot(x_line ,y_line,z_line, color = 'black')
        count += 1
    
    #   Displaying created 3D graph
    plt.show()

#   Printing which port was closed (Port4) 
print("Closing: " + s.name)
s.close()
