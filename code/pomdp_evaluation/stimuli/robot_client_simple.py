from websocket import create_connection
import json

ws = create_connection("ws://localhost:8080")


print "Sending the state request"
while 1:
    action = raw_input("Robot Action \n0=idle | 1=grasp | 2=cancel | 3=point | 4=planning :")
    ws.send(action + "," + "-1")
    print "Action is sent ... \n"

