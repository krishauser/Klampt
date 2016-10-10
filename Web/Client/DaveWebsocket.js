
function Network(URI, newSceneArrivedCallback, consoleTextArrivedCallback, consoleErrorArrivedCallback)
{
   console.log("creating new network object for websocket use");		

   this.URI = URI;
   this.websocket = null;
   this.intervalId = null;
   this.newSceneArrivedCallback=newSceneArrivedCallback;
   this.consoleTextArrivedCallback=consoleTextArrivedCallback;
   this.consoleErrorArrivedCallback=consoleErrorArrivedCallback;
   
   //this.editor = editor;
   this.disconnectionAsked = false;
   
   this.connect();
   updateSocketState();		
}

Network.prototype.connect = function()
{
   console.log("Attempting to setup connection to: " + this.URI);

   this.disconnectionAsked = false;

   try
   {
      if (this.websocket)
      {
         if (this.connected())
         {
            this.websocket.close();
         }
         delete this.websocket;
      }
	
      if (typeof MozWebSocket === 'function')
      {
         WebSocket = MozWebSocket;
      }
		
      this.websocket = new WebSocket(this.URI, ['binary','base64']);
      
      this.websocket.onopen = function(evt)
      {
         console.log("websocket callback onopen");
         updateSocketState(this.websocket);
      }.bind(this);
		
      this.websocket.onclose = function(evt)
      {
         console.log("websocket callback onclose");
               
         updateSocketState(this.websocket);
         if (!this.disconnectionAsked)
         {
            //setTimeout(this.connect.bind(this), 500);
         }
         delete this.websocket;
      }.bind(this);	
		
      this.websocket.onmessage = function(evt) //this is where the webpage receives data from remote
      {
         //console.log("websocket callback onmessage");		
                 
	 if(evt.data instanceof ArrayBuffer)
         {			
            console.log("  got an ArrayBuffer");
            console.log("    data length: " + evt.data.byteLength);				
         }
         else if(evt.data instanceof Blob)
         { 
            console.log("looks like its a Blob!");              
     
            console.log("filename: " + evt.data.name);            
            editor.loader.loadFile(evt.data);
         }
         else if(evt.data instanceof String)
         {
            console.log("its a string!");
         }
	
         else
         {        
            //console.log("  message is text");
				var message=evt.data;
            //console.log("raw message is: " + message);
            var slicedMessage=message.slice(1);
            if(message[0]=='S') 
               newSceneArrivedCallback(slicedMessage);
            else if(message[0]=='C')
               consoleTextArrivedCallback(slicedMessage);      
            else if(message[0]=='E') //console error
               consoleTextArrivedCallback(slicedMessage);      
            else 
              console.log("websocket callback onmessage got invalid message "+message);
         }
				
      }.bind(this);
		
      this.websocket.onerror = function(evt)
      {
         console.warn("Websocket error:", evt.data);
      };
   }
   catch(exception)
   {
      alert("Websocket fatal error, maybe your browser can't use websockets. You can look at the javascript console for more details on the error.");
      console.error("Websocket fatal error", exception);
   }
}

Network.prototype.connected = function()
{
	if (this.websocket && this.websocket.readyState == 1)
	{
		return true;
	}
	return false;
};

Network.prototype.reconnect = function()
{
	if (this.connected())
	{
		this.disconnect();
	}
	this.connect();
}

Network.prototype.disconnect = function()
{
	this.disconnectionAsked = true;
	if (this.connected())
	{
		this.websocket.close();
		updateSocketState(this.websocket);
	}
}

Network.prototype.send = function(message)
{
	if (this.connected())
	{
		this.websocket.send(message);
	}
};

Network.prototype.checkSocket = function()
{
	if (this.websocket)
	{
		var stateStr;
		switch (this.websocket.readyState)
		{
		case 0:
			stateStr = "CONNECTING";
			break;
		case 1:
			stateStr = "OPEN";
			break;
		case 2:
			stateStr = "CLOSING";
			break;
		case 3:
			stateStr = "CLOSED";
			break;
		default:
			stateStr = "UNKNOW";
			break;
		}
      //$("#socketState").text(" (" + stateStr + ")"); 
		console.log("Websocket state : " + this.websocket.readyState + " (" + stateStr + ")");
	}
	else
	{
		console.log("Websocket is not initialised");
	}
}

function sendMessage(value)
{

	if (network && network.connected())
	{
		network.send(value);
		console.log("Message sent :", '"'+value+'"');
	}
	else
	console.log("Not connected to remote, so no message sent");
}

function isConnected()
{
	return network && network.connected()
}

//note: this doesn't actually pause the code... you need to provide callbacks for things to change on
//connection success / failure
function waitForConnection(msecs,callback,failcallback) {
	if(network == null || network.websocket == null || msecs < 0) {
		if(failcallback != null) {
			failcallback();
		}
		return;
	}
	updateSocketState(network.websocket);
    if (network.websocket.readyState === 1) {
        if(callback != null){
            callback();
        }
        return;
    }

    setTimeout(
        function () {
            console.log("wait for connection...");
            updateSocketState(network.websocket);
            waitForConnection(msecs-50, callback, failcallback);
        }, 50); // wait 50 miliseconds for the connection...
}

function waitForDisconnection(msecs,callback,failcallback) {
	if(network == null || network.websocket == null || msecs < 0) {
		if(callback != null) {
			callback();
		}
		return;
	}
	updateSocketState(network.websocket);
    if (network.websocket.readyState == 3) {
        if(callback != null){
            callback();
        }
        return;
    }

    setTimeout(
        function () {
            console.log("wait for disconnection...");
            updateSocketState(network.websocket);
            waitForDisconnection(msecs-50, callback, failcallback);
        }, 50); // wait 50 milisecond for the connection...
}


function updateSocketState(websocket)
{
   console.log("in updateSocketState");		

	if (websocket != null)
	{
		var stateStr;
		switch (websocket.readyState)
		{
		case 0:
			stateStr = "CONNECTING";
			break;
		case 1:
			stateStr = "OPEN";
			break;
		case 2:
			stateStr = "CLOSING";
			break;
		case 3:
			stateStr = "CLOSED";
			break;
		default:
			stateStr = "UNKNOW";
			break;
		}
		//$("#socketState").text(" (" + stateStr + ")"); 
		
		console.log("  socket state changed: " + websocket.readyState + " (" + stateStr + ")");
	}
	else
	{
      console.log("  websocket is null. closed");		
		//document.querySelector("#socketState").innerText = "3 (CLOSED)";
	}
}



