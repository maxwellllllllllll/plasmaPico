import serial

def buildTransferBlock(dataBytes: bytearray):
    length = len(dataBytes)
    out = bytearray(length + 6)

    out[0] = 0x55               # Head
    out[1] = 0x3C               # Sync
    out[2] = 0x01               # Block Type
    out[3] = length             # Data Length

    # Set data
    for i in range(0, length):
        out[i+4] = dataBytes[i]
    
    # Compute checksum
    cs = 0
    for i in range(2, length + 6 - 2):
        cs += out[i]
    cs &= 0xFF

    out[length + 6 - 2] = cs    # Checksum
    out[length + 6 - 1] = 0x55  # Trailer

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
    blockLength = 255 # 10,000
    dataBytes = bytearray()

    j = 0
    flip = False
    for i in range(blockLength):
        dataBytes.append(j)

        if flip == False:
            j += 1
        elif flip == True:
            j -= 1
        
        if j == 255:
            flip = True
        elif j == 0:
            flip = False
    
    return dataBytes


ser = serialInit()

listVals = [0x55, 0x3C, 0x01, 0x02, 0x02, 0x01, 0x03, 0x55] # Head, Sync, Type, Length, [data], cs, trailer

byteVals = bytearray(listVals)

packet = bytearray()

packet.append(0x55)

dataBytes = buildTestDataBlock()

transferBlock = buildTransferBlock(dataBytes)

ser.write(transferBlock)

print(ser.readline())
print(ser.readline())


ser.close()
print("here")