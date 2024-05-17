import sys
import zmq

port = "62026"

context = zmq.Context()
socket = context.socket(zmq.SUB)

socket.connect ("tcp://localhost:%s" % port)

socket.setsockopt(zmq.SUBSCRIBE, "")

for update_nbr in range(5):
    string = socket.recv()
    topic,messagedata = string.split()
    print topic, messagedata

