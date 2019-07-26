from tornado import websocket
import tornado.ioloop
import time

class EchoWebSocket(websocket.WebSocketHandler):
    def open(self):
        print "Websocket Opened"

    def on_message(self, message):
        self.write_message(u"You said: %s" % message)
        print("[TEST SERVER]: Message received: %s" % message)

    def on_close(self):
        print "Websocket closed"

application = tornado.web.Application([(r"/", EchoWebSocket),])

if __name__ == "__main__":
    application.listen(8080)
    tornado.ioloop.IOLoop.instance().start()
