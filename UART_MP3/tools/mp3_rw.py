#!/usr/bin/python
import os, pty, serial, binascii


# name file test.mp3
# read mp3 as binary
f = open("rick.mp3", "rb")
first16bytes = f.read(87323) # right now it's just 16 bytes
hexadec = binascii.hexlify(first16bytes)
decim = int(hexadec,16)
binary = bin(decim)[2:].zfill(8)


# print s_name
ser = serial.Serial(
    port = "/dev/ttyUSB0", # this will be /dev/tty something for the microchip
    baudrate = 1000000, #change number
    parity = serial.PARITY_NONE, #or can do 'N'
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS, #or can just do 8
    xonxoff=serial.XOFF,
    rtscts=False,
    dsrdtr=False,
    timeout=1) #might need to adjust timeout


# To Write to the device
ser.write(first16bytes)
#print first16bytes

# To read from the virtual serial port port - for test
print ser
