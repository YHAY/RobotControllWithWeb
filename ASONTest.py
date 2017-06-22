# TCP communication adapted from https://wiki.python.org/moin/TcpCommunication#Client

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
pub_msg_right = {
  "op": "publish",
  "topic": "/cmd_vel_mux/input/teleop",
  "msg": stops
}

# TCP communication
def connect():
    global s
    TCP_IP = '127.0.0.1'
    TCP_PORT = 9090
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((TCP_IP, TCP_PORT))

def forward():
    print("Sending JSON data")
    s.send(encode_message(pub_msg_forward))
    time.sleep(1)  # Add delay to cause message to be sent in separate tcp packets

def backward():
    print("Sending JSON data")
    s.send(encode_message(pub_msg_back))
    time.sleep(1)  # Add delay to cause message to be sent in separate tcp packets

def left():
    print("Sending JSON data")
    s.send(encode_message(pub_msg_left))
    time.sleep(1)  # Add delay to cause message to be sent in separate tcp packets


def right():
    print("Sending JSON data")
    s.send(encode_message(pub_msg_right))
    time.sleep(1)  # Add delay to cause message to be sent in separate tcp packets

def stop():
    print("stop")
    s.send(encode_message(pub_msg_right))
    time.sleep(1)  # Add delay to cause message to be sent in separate tcp packets

def disConnect():
    s.close()

if __name__ == "__main__":
      global s
      connect()
      left()
      stop()
      forward()
      disConnect()

