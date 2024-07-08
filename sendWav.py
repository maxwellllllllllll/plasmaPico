import serial
import math
import pandas as pd

def buildTransferBlock(dataBytes: bytearray):
    length = len(dataBytes)
    #print(length)
    out = bytearray(length + 7)


    out[0] = 0x55                                              # Head
    out[1] = 0x3C                                              # Sync
    out[2] = 0x01                                              # Block Type (Data Block (TODO: Change name))
    out[3] = length.to_bytes(2, 'little')[0]                   # Data Length First Byte
    out[4] = length.to_bytes(2, 'little')[1]                   # Data Length Second Byte

    # Set data
    for i in range(0, length): #remove the -1
        out[i+5] = dataBytes[i]                                # Data
    
    # Compute checksum
    cs = 0
    for i in range(5, length + 7 - 2):
        cs += out[i]
    cs &= 0xFF

    out[length + 7 - 2] = cs                                   # Checksum
    out[length + 7 - 1] = 0x55                                 # Trailer

    return out

def makeList(length):
    longList = []
    for i in range(0,length):
        longList.append(i)
    
    return longList

def serialInit():
    print("Initializing serial connection")

    ser = serial.Serial('COM3', 9600, timeout=1, bytesize=8, parity='N', stopbits=1)

    print(ser.name)

    return ser

def buildTestDataBlock():
    blockLength = 10 # 10,000
    dataBytes = bytearray()

    j = 50
    flip = False
    for i in range(blockLength):
        #print(j)
        dataBytes.append(52) #math.floor(j)

        if flip == False:
            j = 150
        elif flip == True:
            j = 50
        
        if i % 2 == 0:
            flip = True
        else:
            flip = False
    
    return dataBytes


returnList = []


ser = serialInit()

listVals = [0x55, 0x3C, 0x01, 0x02, 0x02, 0x01, 0x03, 0x55] # Head, Sync, Type, Length, [data], cs, trailer

byteVals = bytearray(listVals)

dataBytes = buildTestDataBlock()

transferBlock = buildTransferBlock(dataBytes)

while True:
    if input("trig? "):
        ser.write(transferBlock) 
        print("written")
        for i in range(10):
            print(ser.readline())

# for i in range(20):
#     line = ser.readline()
#     #returnList.append(line)
#     print(line)
    

df = pd.DataFrame(returnList)

csvData = df.to_csv('csvFile', index=False)

#print(df)

ser.close()
print("here")