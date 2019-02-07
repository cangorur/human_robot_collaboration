from websocket import create_connection
import json

ws = create_connection("ws://localhost:7070")


print "Sending the state request"
while 1:
    obs = raw_input("Observation:")
    ws.send(obs + "," + "-1")
    print "Observation is sent"
    state = raw_input("Real State:")
    ws.send("-1" + "," + state)
