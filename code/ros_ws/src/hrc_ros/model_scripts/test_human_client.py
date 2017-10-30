from websocket import create_connection
import json

ws = create_connection("ws://localhost:7070")


print "Sending the state request"
while 1:
    state = raw_input("State: ")
    ws.send(state)
    print "Sent"
    ## RECEIVE
    print "Receiving..."
    result =  ws.recv()
    print "Received:  '%s'" % result

# ws.close()
