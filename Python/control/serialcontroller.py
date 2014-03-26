"""An adaptor between python controllers and the Klamp't serial controller
interface (SerialController).
"""
import asyncore,socket
import json
import time

headerlen = 4

def packStrlen(s):
    l = len(s)
    assert(l <= 0xffffffff)
    bytes = [None]*4
    bytes[0] = chr(l&0xff)
    bytes[1] = chr((l>>8)&0xff)
    bytes[2] = chr((l>>16)&0xff)
    bytes[3] = chr((l>>24)&0xff)
    return ''.join(bytes)

def unpackStrlen(s):
    assert len(s)==headerlen
    return (ord(s[3])<<24)|(ord(s[2])<<16)|(ord(s[1])<<8)|ord(s[0])

class JsonClient(asyncore.dispatcher):
    """A publisher of JSON messages in the Klamp't simple serial interface.
    Sends variable-length messages such that the first 4 bytes are
    the length of the message (in binary) and the remainder is the payload.

    Subclasses should override onMessage, which accepts with arbitrary
    Python objects that can be serialized by the json module.
    Subclasses should use sendMessage to send a message.
    """
    def __init__(self, addr):
        asyncore.dispatcher.__init__(self)
        self.create_socket(socket.AF_INET, socket.SOCK_STREAM)
        self.connect( addr )
        self.buffer = ""

    def handle_connect(self):
        """Called on socket connect.  May be overridden."""
        pass

    def handle_close(self):
        """Called on socket close.  May be overridden."""
        self.close()

    def handle_read(self):
        """Called on read.  Do not override; override onMessage instead."""
        lenstr = self.recv(headerlen)
        msglen = unpackStrlen(lenstr)
        msg = self.recv(msglen)
        self.onMessage(json.loads(msg))

    def writable(self):
        """Called to determine whether there's any data left to be sent.
        Do not override."""
        return (len(self.buffer) > 0)

    def handle_write(self):
        """Called to send data when available.  Do not override."""
        sent = self.send(self.buffer)
        self.buffer = self.buffer[sent:]

    def onMessage(self,msg):
        """Override this to handle an incoming message"""
        pass
    
    def sendMessage(self,msg):
        """Call this to send an outgoing message"""
        smsg = json.dumps(msg)
        self.buffer = self.buffer + packStrlen(smsg) + smsg

class ControllerClient(JsonClient):
    """A client that relays Python BaseController object to a
    SerialController.
    The interface simply translates messages back and forth using the standard
    BaseController messages."""
    
    def __init__(self,addr,controller):
        """Sends the output of a controller to a SerialController.
        controller is assumed to follow the control.BaseController interface.
        """
        JsonClient.__init__(self,addr)
        self.controller = controller
    def handle_connect(self):
        self.controller.signal('enter')
        return
    def onMessage(self,msg):
        #print "receiving message",msg
        res = self.controller.output_and_advance(**msg)
        if res==None: return
        #print "sending message",res
        self.sendMessage(res)

if __name__ == "__main__":
    import sys
    import trajectory_controller

    host = 'localhost'
    port = 3456

    if len(sys.argv)==1:
        print "Usage: %s [linear_path_file]\n"%(sys.argv[0],)
        print "By default connects to localhost:3456"
        exit()
        
    #by default, runs a trajectory controller
    pathfn = sys.argv[1]
    pycontroller = trajectory_controller.make(None,pathfn)
    s = ControllerClient((host,port),pycontroller)
    asyncore.loop()
