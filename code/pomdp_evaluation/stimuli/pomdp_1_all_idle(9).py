from websocket import create_connection
import json
import time 


ws = create_connection("ws://localhost:7070")

obs = "9" # IDLE
state = "0"                     # always TaskHuman
for i in range(0,25):
     
    ws.send(obs + "," + "-1")
    print "Observation is sent"
    time.sleep(0.5)    
    ws.send("-1" + "," + state)
    time.sleep(1)  

ws.close()
