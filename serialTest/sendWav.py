import serial

def buildDataBlock(mybytes: bytes):
    length = len(mybytes)
    out = bytearray(length + 6)

    out[0] = 0x55             # Head
    out[1] = 0x3C             # Sync
    out[2] = 0x01             # Block Type
    out[3] = length           # Data Length

    # Set data
    for i in range(0, length):
        out[i+4] = mybytes[i]
    
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


ser = serialInit()

listVals = [0x55, 0x3C, 0x01, 0x07, 0x02, 0x02, 0x55] #

byteVals = bytearray(listVals)

packet = bytearray()

packet.append(0x55)

ser.write(byteVals)

print(ser.readline())
print(ser.readline())
print(ser.readline())
print(ser.readline())
print(ser.readline())
