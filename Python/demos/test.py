from klampt.glprogram import *

class MySubwindow(GLRealtimeProgram):
    def initWindow(self):
        GLRealtimeProgram.initWindow(self)
        glutCloseFunc(self.close)
    def close(self):
        print "Asked for close"

class MyViewer(GLRealtimeProgram):
    def initialize(self):
        window = MySubwindow("other window")
        window.width = 300
        window.height = 300
        window.initWindow()

viewer = MyViewer("test")
viewer.run()

print "I'm done"
