from websocket import create_connection
import json

ws = create_connection("ws://localhost:7070")


print "Sending the state request"
while 1:
    state = raw_input("State:")
    ws.send("-1," + state) # first informing about the current state to calculate the reward
    print "Sent"
    ## RECEIVE
    print "Receiving..."
    result =  ws.recv()
    print "Received:  '%s'" % result

    obs = raw_input("Observation:") # then informing about the observation to belief update and select an action.
    ws.send(obs + ",-1")
    print "Sent"
    ## RECEIVE
    print "Receiving..."
    result =  ws.recv()
    print "Received:  '%s'" % result
    # repeat the process

# ws.close()
