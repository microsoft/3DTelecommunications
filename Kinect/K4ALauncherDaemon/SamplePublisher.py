import zmq
import datetime
import time

context = zmq.Context()
socket = context.socket(zmq.PUB)
socket.bind("tcp://*:62026")

while True:
    print "Sending."
    socket.send("%d %s" % (1,"HelloWorld!"))
    time.sleep(1)