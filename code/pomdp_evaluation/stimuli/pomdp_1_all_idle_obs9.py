from websocket import create_connection
import json
import time 
import sys



ws = create_connection("ws://localhost:7070")

print(" \nSending observations ... ")

obs = "9" # IDLE
state = "0"                     # always TaskHuman
for i in range(0,50):
     
    ws.send(obs + "," + "-1")
    #print "Observation is sent"
    sys.stdout.write('.')
    sys.stdout.flush()
    time.sleep(0.5)    
    ws.send("-1" + "," + state)
    time.sleep(1)  

print("\n")
ws.close()
