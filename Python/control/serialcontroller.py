"""An adaptor between python controllers and the Klamp't serial controller
interface (SerialController).
"""
import asyncore,socket
import errno
import json
import time
import controller

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

def writeSocket(socket,msg):
    totalsent = 0
    while totalsent < len(msg):
        sent = socket.send(msg[totalsent:])
        if sent == 0:
            raise IOError("socket connection broken")
        totalsent = totalsent + sent
    return

def readSocket(socket,length):
    chunk = socket.recv(length)
    msg = chunk
    while len(msg) < length:
        chunk = socket.recv(length-len(msg))
        if chunk == '':
            raise IOError("socket connection broken")
        msg = msg + chunk
    return msg

class JsonClient(asyncore.dispatcher):
    """A client that transmits JSON messages in the Klamp't simple serial
    interface. Sends/receives variable-length messages such that the first
    4 bytes are the length of the message (in binary) and the remainder is
    the payload.

    Subclasses should override onMessage, which accepts with arbitrary
    Python objects that can be serialized by the json module.
    Subclasses should use sendMessage to send a message.

    To run, call asyncore.loop().
    """
    def __init__(self, addr):
        if isinstance(addr,socket.socket):
            asyncore.dispatcher.__init__(self,s)
        else:
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
        lenstr = self.read(headerlen)
        msglen = unpackStrlen(lenstr)
        msg = self.read(msglen)
        try:
            output = json.loads(msg)
        except ValueError:
            print "Error parsing JSON object from message '"+msg+"'"
            return
        self.onMessage(output)

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
        #print "JSON message:",smsg
        self.buffer = self.buffer + packStrlen(smsg) + smsg
        #print "buffer now:",self.buffer

    def read(self,length):
        chunk = self.recv(length)
        msg = chunk
        while len(msg) < length:
            chunk = self.recv(length-len(msg))
            if chunk == '':
                raise IOError("socket connection broken")
            msg = msg + chunk
        return msg

    def recv(self, buffer_size):
        """Fix for windows sockets throwing EAGAIN crashing asyncore"""
        while True:
            try:
                data = self.socket.recv(buffer_size)
                if not data:
                    # a closed connection is indicated by signaling
                    # a read condition, and having recv() return 0.
                    print "Socket closed..."
                    self.handle_close()
                    return ''
                else:
                    return data
            except socket.error, why:
                # winsock sometimes throws ENOTCONN
                if why.args[0] in (errno.EAGAIN, errno.EWOULDBLOCK):
                    #print "EGAIN or EWOULDBLOCK returned... spin waiting"
                    time.sleep(0.001)
                    continue
                elif why.args[0] == errno.ENOTCONN:
                    self.handle_close()
                    return ''
                else:
                    raise

class ControllerClient(JsonClient):
    """A client that relays Python BaseController object to a
    SerialController.
    The interface simply translates messages back and forth using the standard
    BaseController messages.

    To run, pass it an address and a control.BaseController interface.
    Then, call asyncore.loop().
    """
    
    def __init__(self,addr,controller):
        """Sends the output of a controller to a SerialController.
        controller is assumed to follow the control.BaseController interface.
        """
        self.connecting = True
        JsonClient.__init__(self,addr)
        self.controller = controller
    def handle_connect(self):
        print "Handle connect"
        JsonClient.handle_connect(self)
    def handle_expt(self):
        self.close()
    def handle_error(self):
        JsonClient.handle_error(self)
        if self.connecting:
            print
            print "(Did you forget to start up a Klamp't controller server?)"
        else:
            print
            print "(Did the Klamp't controller server shut down?)"
    def handle_connect(self):
        self.connecting = False;
        self.controller.signal('enter')
        return
    def onMessage(self,msg):
        #print "receiving message",msg
        try:
            res = self.controller.output_and_advance(**msg)
            if res==None: return
        except Exception as e:
            print "Exception",e,"on read"
            return
        try:
            #print "sending message",res
            self.sendMessage(res)
        except IOError as e:
            print "Exception",e,"on send"
            return


class SerialController(controller.BaseController):
    """A controller that maintains a server to write/read messages.
    It simply translates messages back and forth to a client via the serial
    interface.
    """
    
    def __init__(self,addr=('localhost',3456)):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.bind( addr )
        self.sock.listen(1)
        print "SerialController: Listening on port",addr[1]
        self.clientsock = None

    def accept(self):
        """Get a new connection, if there isn't one"""
        if self.clientsock == None:
            pair = self.sock.accept()
            if pair != None:
                sock, addr = pair
                print 'SerialController: Incoming connection from %s' % repr(addr)
                self.clientsock = sock
        return
    
    def output(self,**inputs):
        self.accept()
        if self.clientsock == None:
            return None
        #Convert inputs to JSON message
        smsg = json.dumps(inputs)
        msg = packStrlen(smsg) + smsg
        try:
            writeSocket(self.clientsock,msg)
            #Read response from serial client
            lenstr = readSocket(self.clientsock,headerlen)
            msglen = unpackStrlen(lenstr)
            msg = readSocket(self.clientsock,msglen)
        except IOError:
            print "SerialController: Error writing or reading socket..."
            self.clientsock.close()
            self.clientsock = None
            return None
        try:
            output = json.loads(msg)
            return output
        except ValueError:
            #didn't parse properly
            print "Couldn't read Python object from JSON message '"+msg+"'"
            return None


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

def make(robot):
    return SerialController()
