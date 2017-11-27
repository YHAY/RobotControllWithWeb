from bottle import route, run, get, post, response, static_file, request
import cv2
import urllib
import numpy as np
import time
import threading
from controlCommand import *
import os

sem = threading._Semaphore()
stream = urllib.urlopen('http://localhost:8080/stream?topic=/image_view/output')
bytes = bytes()
rdbuffer = np.array([])

def thread_inc():
    global stream, rdbuffer, bytes
    print "aaa"
    while True:
        bytes += stream.read(1024)
        a = bytes.find(b'\xff\xd8')
        b = bytes.find(b'\xff\xd9')
        if a != -1 and b != -1:
            sem.acquire()
            rdbuffer = np.copy(bytes[a:b+2])
            bytes = bytes[b+2:]
            sem.release()
            time.sleep(0.001)###

def forceToKill():
	appName = 'python'
	killApp = 'killall -9 ' + appName
	os.system(killApp)

@get('/stream')
def do_stream():
    global rdbuffer
    response.set_header('Content-Type', 'multipart/x-mixed-replace; boundary=--MjpgBound')
    while True:
        sem.acquire()
        jpegdata = str(rdbuffer)
        string = "--MjpgBound\r\n"
        string += "Content-Type: image/jpeg\r\n"
        string += "Content-length: " + str(len(jpegdata))+"\r\n\r\n"
        string += jpegdata
        string += "\r\n\r\n\r\n"
        sem.release()
        yield string

@post('/motor')
def control_ros():
  Connect()#???
  command=request.forms.get('command')
  print command, '****'
  if command == "GO"     : forward()
  elif command == "LEFT" : left()
  elif command == "RIGHT" : right()
  elif command == "STOP" :  stop()
  elif command == "BACK" : backward()
  return ''
  #disConnect()#???

@route('/')
def do_route():
  return static_file("index.html", root=".") #WebClient

th = threading.Thread(target = thread_inc)
th.start()

run(host='localhost', port=8888, server='paste')
#run(host='localhost', port=8888, server='paste' debug=True)
th.join()

