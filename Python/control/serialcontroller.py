"""A socket that publishes variable-length messages such that the first 4
bytes are the length of the message (in binary) and the remainder is the payload """
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
    def __init__(self, addr):
        asyncore.dispatcher.__init__(self)
        self.create_socket(socket.AF_INET, socket.SOCK_STREAM)
        self.connect( addr )
        self.buffer = ""

    def handle_connect(self):
        pass

    def handle_close(self):
        self.close()

    def handle_read(self):
        lenstr = self.recv(headerlen)
        msglen = unpackStrlen(lenstr)
        msg = self.recv(msglen)
        self.onMessage(json.loads(msg))

    def writable(self):
        return (len(self.buffer) > 0)

    def handle_write(self):
        sent = self.send(self.buffer)
        self.buffer = self.buffer[sent:]

    def onMessage(self,msg):
        """Call this to parse an incoming message"""
        pass
    
    def sendMessage(self,msg):
        """Call this to send an outgoing message"""
        smsg = json.dumps(msg)
        self.buffer = self.buffer + packStrlen(smsg) + smsg

class ControllerClient(JsonClient):
    """A client that connects a Python BaseController object to a SerialController.
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
        print "By default connects to 127.0.0.1:3456"
        exit()
        
    #by default, runs a trajectory controller
    pathfn = sys.argv[1]
    pycontroller = trajectory_controller.make(None,pathfn)
    s = ControllerClient((host,port),pycontroller)
    asyncore.loop()
