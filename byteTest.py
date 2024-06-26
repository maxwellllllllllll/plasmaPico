import serial

ser = serial.Serial('COM3', 9600, timeout=1, bytesize=8, parity='N', stopbits=1)

print(ser.readline())
print(ser.readline())
print(ser.readline())