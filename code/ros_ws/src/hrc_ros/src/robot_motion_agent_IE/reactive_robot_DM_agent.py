#!/usr/bin/env python

from tornado import escape
from tornado import gen
from tornado import httpserver
from tornado import httpclient
from tornado import httputil
from tornado import ioloop
from tornado import websocket
import tornado.ioloop
import time
import rospy
import signal
import sys
from websocket import create_connection

class EchoWebSocket(websocket.WebSocketHandler):
    def open(self):
        print "Websocket Opened"
        #websocket.enableTrace(True)
        connection_up = 0
        #rospy.loginfo("[REACTIVE ROBOT] Trying to connect to 8080 port ...")
        print "[REACTIVE ROBOT] Trying to connect to 8080 port ..."
        try:
            self.client = create_connection("ws://localhost:8080")
            connection_up = 1
            #rospy.loginfo("[REACTIVE ROBOT] Client to 8080 is initiated !")
            print "[REACTIVE ROBOT] Client to 8080 is initiated !"
        except:
            print "[REACTIVE ROBOT] Server is not up yet. Dying ..."
            connection_up = 0
            sys.exit()

    def on_message(self, message):
        #rospy.loginfo("[REACTIVE ROBOT] Msg received from observation agent: ", message)
        print "[REACTIVE ROBOT] Msg received from observation agent: ", message
        self.write_message(u"You said: %s" % message)
        msg_array = message.split(",")
        observation = msg_array[0]
        state = msg_array[1]

        if observation != "-1": # observation information is provided.
            action = self.state_to_action(observation)
            #time.sleep(1.5)
            #state = "-1"
        else: # state information is provided
            action = "-1"
            #TODO: this is state message

        message = action + "," + state + "," + "0,0"
        self.client.send(message)
        ## RECEIVE
        result =  self.client.recv()
        #rospy.loginfo("[REACTIVE ROBOT] Message Sent: ", message)
        print "[REACTIVE ROBOT] Message Sent: ", message
        #print("[REACTIVE ROBOT] Received:  '%s'" % result)
        self.client.close()

    def on_close(self):
        #rospy.loginfo("[REACTIVE ROBOT] Websocket closed")
        print "[REACTIVE ROBOT] Websocket closed"


    def state_to_action(self, observation):
        action = "-1"
        if observation == "0" or observation == "4":
            action = "0"
        elif observation == "1" or observation == "5":
            sys.exit()
        elif observation == "2":
            action = "1"
        elif observation == "3":
            action = "2"
        else:
            #rospy.logwarn("[REACTIVE ROBOT] Wrong observation reading is received !. Skipping ... ")
            print "[REACTIVE ROBOT] Wrong observation reading is received !. Skipping ... "
        return action


if __name__ == "__main__":
    rospy.init_node('reactive_robot_DM_agent')

    application = tornado.web.Application([(r"/", EchoWebSocket),])
    http_server = tornado.httpserver.HTTPServer(application)
    http_server.listen(7070)
    tornado.ioloop.IOLoop.instance().start()

    #rospy.loginfo("Reactive robot decision_making agent is ready!")
    rospy.spin()
