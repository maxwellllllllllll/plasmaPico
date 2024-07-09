import serial
import math
import pandas as pd

dataLength = 10000 # 10,000

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
    dataBytes = bytearray()

    j = 50
    flip = False
    for i in range(dataLength):
        #print(j)
        dataBytes.append(math.floor(j))

        if flip == False:
            j = 150
        elif flip == True:
            j = 50
        
        if i % 2 == 0:
            flip = True
        else:
            flip = False
    
    return dataBytes


def buildSineDataBlock():
    dataBytes = bytearray()

    j = 50
    flip = False
    for i in range(dataLength):
        dataBytes.append(math.floor(j))

        if flip == False:
            j += 0.1
        elif flip == True:
            j -= 0.1
        
        if j >= 200:
            flip = True
        elif j <= 1:
            flip = False
    
    return dataBytes


def recieveDataBlock(blockLength):
    returnList = []

    while True:
        read = ser.readline()
        print(read)
        if read != b'':
            returnList.append(read)
            break

    print("building")
    for i in range(blockLength): # redo me with new data block trailer system once implimented
        returnList.append(ser.readline())
    
    df = pd.DataFrame(returnList)

    df.to_csv('returnBlock.csv')

    return df


ser = serialInit()

listVals = [0x55, 0x3C, 0x01, 0x02, 0x02, 0x01, 0x03, 0x55] # Head, Sync, Type, Length, [data], cs, trailer

byteVals = bytearray(listVals)


while True:

    while True:
        inp = input("trig? ")
        if inp == 'y':
            dataBytes = buildTestDataBlock()
            break
            
        
        elif inp == 's':
            dataBytes = buildSineDataBlock()
            break

    transferBlock = buildTransferBlock(dataBytes)
    ser.write(transferBlock) 
    print("written")

    df = recieveDataBlock(dataLength + 7)
        
    print(df)