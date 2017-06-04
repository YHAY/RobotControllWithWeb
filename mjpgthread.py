import urllib
import re
import numpy as np
import cv2

#stream = urllib.urlopen('http://localhost:8080/snapshot?topic=/image_view/output')
stream = urllib.urlopen('http://localhost:8080/stream?topic=/image_view/output')
bytes = bytes()

rdbuffer = bytearray(40000)

while True:
    bytes += stream.read(1024)
    print type(bytes)
    a = bytes.find(b'\xff\xd8')
    b = bytes.find(b'\xff\xd9')
    if a != -1 and b != -1:
        jpg = bytes[a:b+2]
        bytes = bytes[b+2:]
        rdbuffer = jpg
        i = cv2.imdecode(np.fromstring(rdbuffer, dtype=np.uint8), cv2.IMREAD_COLOR)
        cv2.imshow('i', i)
        if cv2.waitKey(1) == 27:
            exit(0)

