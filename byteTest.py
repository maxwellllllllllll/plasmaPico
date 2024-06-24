blockLength = 10000 # 10,000
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

for k in range(len(dataBytes)):
    print(dataBytes[k])