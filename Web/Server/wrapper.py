import time
import kviz
import emb
import log
import sys

class StdoutCatcher:
    def write(self, str):
        log.CaptureStdout(str)
class StderrCatcher:
    def write(self, str):
        log.CaptureStderr(str)
sys.stdout = StdoutCatcher()
sys.stderr = StderrCatcher()


wrapper_frame_precomputed = False
wrapper_JSON_message_count = 0 
wrapper_jString = None

def wrapper_compute_JSON():
	global wrapper_JSON_message_count,wrapper_jString
	if wrapper_JSON_message_count==0:
		starttime=time.time();
		wrapper_jString=kviz._getInitialJSON()
		secs = time.time()- starttime
		msecs = secs * 1000  # millisecs
		#print "Getting the scene in JSON format took: " + "{:.2f}".format(msecs) + " ms"
	else:   
		starttime=time.time();
		wrapper_jString=kviz._getUpdateJSON()
		secs = time.time() - starttime
		msecs = secs * 1000  # millisecs
		#print "Getting the transforms in JSON format took: " + "{:.2f}".format(msecs) + " ms"

def wrapper_send_JSON():
	global wrapper_JSON_message_count,wrapper_jString
	starttime=time.time(); 
	emb.send(wrapper_jString)
	secs = time.time()- starttime
	msecs = secs * 1000  # millisecs
	#print "sending JSON took: " + "{:.2f}".format(msecs) + " ms"
	wrapper_JSON_message_count+=1

def wrapper_advance_internal():
	try:
		starttime = time.time()
		boilerplate_advance()
		secs = time.time() - starttime
		msecs = secs * 1000  # millisecs
		#print "Advancing took: " + "{:.2f}".format(msecs) + " ms"
	except Exception as e:
		print "Exception in advance code",
		raise

def wrapper_start():
	global wrapper_JSON_message_count,wrapper_frame_precomputed
	if wrapper_frame_precomputed != False:
		#had an old frame -- need to send this before reset
		wrapper_send_JSON()

	kviz._reset()
	try:
		boilerplate_start()
	except Exception as e:
		print "Exception in init code"
		raise
	wrapper_JSON_message_count = 0
	wrapper_frame_precomputed = False

	#send the initial scene
	wrapper_compute_JSON()
	wrapper_send_JSON()
	
def wrapper_advance():
	global wrapper_frame_precomputed

	if wrapper_frame_precomputed == False:
		wrapper_advance_internal()
		wrapper_compute_JSON()
	
	wrapper_send_JSON()

	wrapper_advance_internal()  #we'll precompute the next frame to speed things up
	wrapper_compute_JSON()		
	wrapper_frame_precomputed = True

def wrapper_keypress(key):
	#print "got key: " + str(key)
	try:
		boilerplate_keypress(key)
	except Exception as e:
		print "Exception in init code"
		raise


