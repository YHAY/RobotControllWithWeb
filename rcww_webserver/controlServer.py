# TCP communication adapted from https://wiki.python.org/moin/TcpCommunication#Client

#why repeat the connection and the disconnection?

import bson
import json
import time
import socket
import pprint
BUFFER_SIZE = 4096
print("BSON-ROSBridge Testclient")


def encode_message(msg):
  return json.dumps(msg)

def decode_message(msg):
  return json.loads(msg)
  
#forward command message
forwards = {
  "linear" : {"x" : 0.08}
}
#backward command message
backwards = {
  "linear" : {"x" : -0.08}
}
#leftward command message
leftwards = {
  "angular" : {"z" : 0.08}
}
#rightward command message
rightwards = {
  "angular" : {"z" : -0.08}
}
stops = {
  "linear" : {
       "x" : 0.0,
       "y" : 0.0,
       "z" : 0.0
    },
    "angular" : {
        "x" : 0.0,
        "y" : 0.0,
        "z" : 0.0
    }
}
#publish forward
pub_msg_forward = {
  "op": "publish",
  "topic": "/cmd_vel_mux/input/teleop",
  "msg": forwards
}
#publish backward
pub_msg_back = {
  "op": "publish",
  "topic": "/cmd_vel_mux/input/teleop",
  "msg": backwards
}
#publish leftward
pub_msg_left = {
  "op": "publish",
  "topic": "/cmd_vel_mux/input/teleop",
  "msg": leftwards
}
#publish rightward
pub_msg_right = {
  "op": "publish",
  "topic": "/cmd_vel_mux/input/teleop",
  "msg": rightwards
}
#publish stop
pub_msg_stop = {
  "op": "publish",
  "topic": "/cmd_vel_mux/input/teleop",
  "msg": stops
}

# TCP communication
def Connect():
    global s
    TCP_IP = '127.0.0.1' #global
    TCP_PORT = 9090 #global
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((TCP_IP, TCP_PORT))

def forward():
    global s
    print("Sending JSON data")
    s.send(encode_message(pub_msg_forward))
    time.sleep(1)  # Add delay to cause message to be sent in separate tcp packets

def backward():
    global s
    print("Sending JSON data")
    s.send(encode_message(pub_msg_back))
    time.sleep(1)  # Add delay to cause message to be sent in separate tcp packets

def left():
    global s
    print("Sending JSON data")
    s.send(encode_message(pub_msg_left))
    time.sleep(1)  # Add delay to cause message to be sent in separate tcp packets


def right():
    global s
    print("Sending JSON data")
    s.send(encode_message(pub_msg_right))
    time.sleep(1)  # Add delay to cause message to be sent in separate tcp packets

def stop():
    global s
    print("stop")
    s.send(encode_message(pub_msg_stop))
    time.sleep(1)  # Add delay to cause message to be sent in separate tcp packets

def disConnect():
    global s
    s.close()

if __name__ == "__main__":
      global s
      connect()
      left()
      stop()
      forward()
      disConnect()

