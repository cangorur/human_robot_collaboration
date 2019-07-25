import websocket
import thread
import time

def on_message(ws, message):
    print(message)
    # TODO split the message: "observation, -1" or "-1, state"
    msg_array = message.split(",")
    observation = msg_array[0]
    state = msg_array[1]

    if observation != "-1": # observation information is provided.
        action = state_to_action(observation)
        #state = "-1"
    else: # state information is provided
        action = "-1"
        #TODO: this is state message

    time.sleep(1)
    message = action + "," + state + "," + "0,0"
    ws_client.send(action)
    print("[REACTIVE ROBOT] Message Sent: ", message)
    ## RECEIVE
    result =  ws_client.recv()
    print "[REACTIVE ROBOT] Received:  '%s'" % result

def on_error(ws, error):
    print("[REACTIVE ROBOT]", error)

def on_close(ws):
    print("[REACTIVE ROBOT] ### Websocket closed ###")


def on_open(ws):
    def run(*args):
        for i in range(3):
            time.sleep(1)
            ws.send("Hello %d" % i)
        time.sleep(1)
        ws.close()
        print("thread terminating...")
    thread.start_new_thread(run, ())


def state_to_action(observation):
    if observation == "0" or observation == "1" or observation == "4" or observation == "5":
        action = "0"
    elif observation == "2":
        action = "1"
    elif observation == "3":
        action = "2"
    else:
        print("[REACTIVE ROBOT] Wrong observation reading is received !. Skipping ... ")
    return action

if __name__ == "__main__":
    websocket.enableTrace(True)
    ws = websocket.WebSocketApp("localhost:7070",
                              on_message = on_message,
                              on_error = on_error,
                              on_close = on_close)
    ws.on_open = on_open
    ws.run_forever()

    #ws_client = websocket.create_connection("ws://localhost:8080")
