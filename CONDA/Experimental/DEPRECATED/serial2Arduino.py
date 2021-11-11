# -*- coding: utf-8 -*-
"""
Created on Mon Mar  8 21:14:55 2021

@author: enriq
"""

robots = dict(ram01 = "COM9", 
              ram02 = "COM11", 
              ram03 = "COM14", 
              ram04 = "COM16", 
              ram05 = "COM17")

import time
import serial

RAM01 = serial.Serial(robots['ram01'],9600)
RAM02 = serial.Serial(robots['ram02'],9600)
RAM03 = serial.Serial(robots['ram03'],9600)
RAM04 = serial.Serial(robots['ram04'],9600)
RAM05 = serial.Serial(robots['ram05'],9600)

cad = "@W100D100I#"

print('RAM01')
RAM01.write(cad.encode('ascii'))
time.sleep(5)
print('RAM02')
RAM02.write(cad.encode('ascii'))
time.sleep(5)
print('RAM03')
RAM03.write(cad.encode('ascii'))
time.sleep(5)
print('RAM04')
RAM04.write(cad.encode('ascii'))
time.sleep(5)
print('RAM05')
RAM05.write(cad.encode('ascii'))

# a1 = ""
# a2 = ""
# try:
#     while True:
#         a1 = RAM01.read_until()
#         # a2 = RAM02.read_until()
#         a1 = a1[0:len(a1)-2]
#         # a2 = a2[0:len(a2)-2]
#         print(a1)
#         # print(a2)
# except KeyboardInterrupt:
#     pass

time.sleep(5)

cad = ""
cad = "@W0D0I#"
RAM01.write(cad.encode('ascii'))
RAM02.write(cad.encode('ascii'))
RAM03.write(cad.encode('ascii'))
RAM04.write(cad.encode('ascii'))
RAM05.write(cad.encode('ascii'))

RAM01.close()
RAM02.close()
RAM03.close()
RAM04.close()
RAM05.close()
