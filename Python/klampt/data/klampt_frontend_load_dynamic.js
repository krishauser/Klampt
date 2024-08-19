  function addScript(id,url,callback) {
    var js, fjs = document.getElementsByTagName('script')[0];
    if (document.getElementById(id)) {return;}
    js = document.createElement('script');
    js.id = id;
    js.src = url;
    js.onreadystatechange = callback;
    js.onload = callback;
    js.onerror = function() { 
      c = document.getElementById("canvas");
      msg = document.createElement("h3");
      msg.innerHTML = "Error loading Javascript libraries from "+url+", you will need internet access to view this animation";
      c.appendChild(msg);
    }
    fjs.parentNode.insertBefore(js, fjs);
  }
  addScript("three_js_load","https://motion.cs.illinois.edu/klampt/three.min.js",function() {
    addScript("klampt_load","https://motion.cs.illinois.edu/klampt/KlamptFrontend.js",null);
  });