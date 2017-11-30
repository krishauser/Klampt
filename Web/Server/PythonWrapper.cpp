// http://motion.pratt.duke.edu/klampt/tutorial_simulation.html

#include "PythonWrapper.h"
#include <unistd.h> //to get sleep function TODO: not cross-platform
#include <iostream>
#include <fstream>
#include <string> 
#include <node.h> // from python
#include <compile.h> // from python
using namespace std;

#include "websocket.h"

string boilerplate_directory;
bool python_initialized = false;
bool boilerplate_loaded = false;
bool student_code_preload = false;
bool student_code_loaded = false;
bool precomputed_response = false;
string stdout_buffer;
string stderr_buffer;
string response_buffer;
bool pause_io = false;

/*
// http://faq.cprogramming.com/cgi-bin/smartfaq.cgi?answer=1045689663&id=1043284385
std::string IntToString ( int number )
{
  std::ostringstream oss;

  // Works just like cout
  oss<< number;

  // Return the underlying string
  return oss.str();
}
*/

///Clears the stdout and stderr buffers
void ClearStreams()
{
  stdout_buffer.clear();
  stderr_buffer.clear();
  response_buffer.clear();
}

///Flushes the stdout and stderr buffers to the client
void FlushStreams()
{
  if(!response_buffer.empty()) {
    websocket_send("S"+response_buffer);
    response_buffer.clear();
  }
  if(!stdout_buffer.empty()) {
    //printf("[python stdout flush] %s\n",stdout_buffer.c_str());
    websocket_send("C"+stdout_buffer);
    stdout_buffer.clear();
  }
  if(!stderr_buffer.empty()) {
    //printf("[python stderr flush] %s\n",stderr_buffer.c_str());
    websocket_send("E"+stderr_buffer);
    stderr_buffer.clear();
  }
}

////////////////////////////////////////////////////////
// Allow python program to call C function 
// https://docs.python.org/2/extending/embedding.html
////////////////////////////////////////////////////////

static PyObject* emb_send(PyObject *self, PyObject *args)
{
  PyObject *a;
	
  if (!PyArg_UnpackTuple(args, "func", 1, 1, &a)) 
    return NULL;
	      
  if(PyString_Check(a)) //verify data type 
  {
    char *data=PyString_AsString(a);
    //printf("from python got data: %s\n",data);
    if(pause_io) {
      response_buffer += data;
    }
    else {
      websocket_send("S"+string(data));
    }
  }

  Py_INCREF(Py_None);
  return Py_None;
}

static PyMethodDef EmbMethods[] = {
   {"send", emb_send, METH_VARARGS,"send info from the py process to C++ process"},
   {NULL, NULL, 0, NULL}
};

//////////////////////////////////////////////////
// Capture Python Stdout + Stderror
// http://www.ragestorm.net/tutorial?id=21#9
//////////////////////////////////////////////////

PyObject* log_CaptureStdout(PyObject* self, PyObject* args)
{
  PyObject *a;

  if (!PyArg_UnpackTuple(args, "func", 1, 1, &a))
    return NULL;      

  if(PyString_Check(a))
  {
    char *data=PyString_AsString(a);
    stdout_buffer += data;
    if(!pause_io) {
      if(stdout_buffer[stdout_buffer.length()-1] == '\n') {
       //printf("[python stdout] %s",stdout_buffer.c_str());
       websocket_send("C"+stdout_buffer);
       stdout_buffer.clear();
      }
    }
  }

  Py_INCREF(Py_None);
  return Py_None;
}

PyObject* log_CaptureStderr(PyObject* self, PyObject* args)
{
  PyObject *a;

  if (!PyArg_UnpackTuple(args, "func", 1, 1, &a)) 
    return NULL;
      
  if(PyString_Check(a))
  {
    char *data=PyString_AsString(a);
    stderr_buffer += data;
    if(!pause_io) {
      if(stderr_buffer[stderr_buffer.length()-1] == '\n') {
       //printf("[python stderr] %s",stderr_buffer.c_str());
       websocket_send("E"+stderr_buffer);
       stderr_buffer.clear();
      }
    }
  }

  Py_INCREF(Py_None);
  return Py_None;
}

static PyMethodDef logMethods[] = {
 {"CaptureStdout", log_CaptureStdout, METH_VARARGS, "Logs stdout"},
 {"CaptureStderr", log_CaptureStderr, METH_VARARGS, "Logs stderr"},
 {NULL, NULL, 0, NULL}
};

bool mval=false;

// from http://stackoverflow.com/questions/2912520/read-file-contents-into-a-string-in-c
std::string load_file(std::string filename)
{
  std::ifstream ifs(filename.c_str());

  if(ifs.good()==false)
  {
    //printf("  file %s doesn't exist!\n",filename.c_str());
    return std::string("");
  }

  std::string content( (std::istreambuf_iterator<char>(ifs) ),
                (std::istreambuf_iterator<char>()   ) );
  //std::cout << content;
  return content;
}

//// Code to run a string and have the exception information have a filename rather than just <string>


// Copied from pythonrun.c
static PyObject *run_node(struct _node *n, const char *filename, PyObject *globals, PyObject *locals) {
   PyCodeObject *co;
   PyObject *v;
   co = PyNode_Compile(n, filename);
   PyNode_Free(n);
   if (co == NULL)
      return NULL;
   v = PyEval_EvalCode(co, globals, locals);
   Py_DECREF(co);
   return v;
}

// This is missing from python: a PyRun_String that also takes a filename, to show in backtraces
static PyObject* MyPyRun_StringFileName(const char *str, const char* filename, int start, PyObject *globals, PyObject *locals) {
   struct _node* n = PyParser_SimpleParseString(str, start);
   if (!n) return 0;
   return run_node( n, filename, globals, locals);
}

void initialize_python_interpreter()
{
  assert(!python_initialized);
  //Py_SetProgramName("KlamptWebPython");  /* optional but recommended */
  Py_Initialize();
  Py_InitModule("emb", EmbMethods); //setup embedded methods
  Py_InitModule("log", logMethods); //setup stdio capture  
  
  python_initialized = true;
}

bool run_boiler_plate(const string& which)
{
  string boilerplate=which+"/boilerplate"+".py";

  printf("  running boilerplate code %s\n",which.c_str()); 
  std::string boiler_plate=load_file(boilerplate);

  if(boiler_plate.size()==0) //okay now lets try to find the file in a different place
  {
    boiler_plate=load_file("./Web/Client/Scenarios/"+boilerplate);
  }

  if(boiler_plate.size()!=0)
  {
    printf("   found the boiler plate!\n");
    int res = PyRun_SimpleString(boiler_plate.c_str());
    if(res < 0 || PyErr_Occurred()) {
      printf("  error while running boiler plate %s?\n",which.c_str());
      PyErr_Clear();
      return false;
    }
  }
  else {
    printf("   We weren't able to properly load the boiler plate %s\n",which.c_str());
    return false;
  }
  std::string wrapper=load_file("Web/Server/wrapper.py");
  if(wrapper.size()!=0)
  {
    printf("   found the boiler plate wrapper!\n");
    int res = PyRun_SimpleString(wrapper.c_str());
    if(res < 0 || PyErr_Occurred()) {
      printf("  error while running wrapper.py?\n");
      PyErr_Clear();
      return false;
    }
  }
  else {
    printf("   We weren't able to properly load the wrapper\n");
    return false;
  }
  return true;
}

void shutdown_python_interpreter()
{
  if(python_initialized) {
    printf("Shutting down Python interpreter\n");
    Py_Finalize();
  }
}

void handleIncomingMessage(string message)
{
  //printf("received incoming message!\n"); 

  if(message.size()>=1) //TODO, actually have prefix to route message
  {
    char routing=message[0];
    message.erase(0, 1); //remove routing prefix

    if(routing=='E') //user event
    {
      if(precomputed_response) {
        //output previous code
        FlushStreams();
        precomputed_response = false;
        pause_io = false;
      }
      std::string pycode;
      pycode+="wrapper_event('"+message+"')\n";
      int res = PyRun_SimpleString(pycode.c_str());
      FlushStreams();
    }
    else if(routing=='S') //set item event
    {
      if(precomputed_response) {
        //output previous code
        FlushStreams();
        precomputed_response = false;
        pause_io = false;
      }
      size_t i=message.find(',');
      if(i == std::string::npos) {
       printf("Error parsing set item message, no comma found\n");
       return;
      }
      std::string item = message.substr(0,i);
      std::string value = message.substr(i+1,message.length()-i-1);
      //convert JSON simple values to Python ones
      if(value == "true") value="True";
      else if(value == "false") value="False";
      std::string pycode;
      pycode+="wrapper_setitem('" + item + "',"+value+")\n";
      int res = PyRun_SimpleString(pycode.c_str());
      FlushStreams();
    }
    else if(routing=='A')
    {
      //printf("  user would like to advance frame\n");
      if(!boilerplate_loaded || !student_code_loaded) {
        printf("  Code is not updated, returning.\n");
      }
      else {
        if(precomputed_response) {
         //output previous code
         FlushStreams();
         precomputed_response = false;
         pause_io = false;
        }
        else {
         precomputed_response = false;
         pause_io = false;
         int res = PyRun_SimpleString("wrapper_advance()\n");
         if(res < 0 || PyErr_Occurred()) {
           printf("  An exception occurred while running client code\n");
           PyErr_Clear();
           return;
         }
        }
      }
    }
    else if(routing=='R')
    {
      //printf("  user would like to run continuously\n");
      if(!boilerplate_loaded || !student_code_loaded) {
        printf("  Code is not updated, returning.\n");
      }
      else {
        if(!precomputed_response) {
         pause_io = true;
         int res = PyRun_SimpleString("wrapper_advance()\n");
         if(res < 0 || PyErr_Occurred()) {
           printf("  An exception occurred while running client code\n");
           PyErr_Clear();
           FlushStreams();
           return;
         }
        }
        //output previous code
        FlushStreams();
        pause_io = true;
        precomputed_response = true;
        int res = PyRun_SimpleString("wrapper_advance()\n");
        if(res < 0 || PyErr_Occurred()) {
          printf("  An exception occurred while running client code\n");
          PyErr_Clear();
          return;
        }
      }
    }
    else if(routing=='C')
    {  
      pause_io = false;
      precomputed_response = false;
      ClearStreams();

      printf("  user would like to add some student code\n");
      if(!boilerplate_loaded) {
       printf("Boilerplate failed to load, not proceeding.\n");
       return;
      }

      PyObject* stub_module;
      PyObject* main_module = PyImport_AddModule("__main__");
      if(main_module == NULL) {
       printf("Uh... couldn't add __main__ module?\n");
       return;
      }
      if(student_code_preload) {
       PyRun_SimpleString("del sys.modules['stub']");
      }

      stub_module = PyImport_ImportModule("stub");
      if(stub_module == NULL) {
       printf("Uh... couldn't load stub.py?\n");
       return;
      }
      PyObject_SetAttrString(main_module, "stub", stub_module);
      PyObject* stub_dict = PyModule_GetDict(stub_module);
      if(!student_code_preload) {
       //some simple sandboxing
       //PyRun_SimpleString("print stub.__builtins__.keys()");
       //ARGH: traceback uses linecache which uses open
       //PyRun_SimpleString("del stub.__builtins__['open']");
       PyRun_SimpleString("del stub.__builtins__['raw_input']");
       PyRun_SimpleString("import sys");
       PyRun_SimpleString("sys.path.append('Web/Server')");
       //TEMP: allow os in imports.
       //PyRun_SimpleString("sys.modules['os']=None");
       //TEMP: allow sys in imports.
       //PyRun_SimpleString("sys.modules['sys']=None");
       student_code_preload = true;
      }
      PyRun_SimpleString((string("__DIR__ = '")+boilerplate_directory+string("/'")).c_str());
      PyRun_SimpleString(string("__KLAMPT_DIR__ = ''").c_str());
      PyRun_SimpleString((string("stub.__DIR__ = '")+boilerplate_directory+string("/'")).c_str());
      PyRun_SimpleString(string("stub.__KLAMPT_DIR__ = ''").c_str());

      /*
      PyObject *key, *value;
      Py_ssize_t pos = 0;
      printf("Stub code: before values\n");
      while (PyDict_Next(stub_dict, &pos, &key, &value)) {
        PyObject_Print(key,stdout,Py_PRINT_RAW);
        printf("\n");
      }
      */

      PyObject* res = MyPyRun_StringFileName(message.c_str(),"client_code",Py_file_input,stub_dict,stub_dict);
      /*
      printf("Stub code: after values\n");
      pos = 0;
      while (PyDict_Next(stub_dict, &pos, &key, &value)) {
        PyObject_Print(key,stdout,Py_PRINT_RAW);
        printf("\n");
      }
      */

      if(!res) {
       printf("  An exception occurred while running client code.\n");
       Py_XDECREF(stub_dict);
       PyErr_Print();
       PyErr_Clear();
       return;
      }
      if(PyErr_Occurred()) {
       printf("  An exception occurred while running client code\n");
       PyErr_Print();
       PyErr_Clear();
       return;
      }
      Py_XDECREF(res);
      Py_XDECREF(stub_dict);
      //PyRun_SimpleString(message.c_str());

      if(!student_code_preload) {
        //TEMP: delete access to sys from code
        PyRun_SimpleString("sys.modules['sys']=None");
      }

      printf("Running boilerplate_start()...\n");
      int res2 = PyRun_SimpleString("wrapper_start()\n");
      if(res2 >= 0 && !PyErr_Occurred()) {
       printf("  Loaded.\n");
       student_code_loaded = true;
      }
      else {
       printf("  Error occurred while running boilerplate_start()\n");
       PyErr_Print();
       PyErr_Clear();
      }
    }
    else if(routing=='B')
    {
      if(boilerplate_loaded) fprintf(stderr,"  Boilerplate was loaded twice, erroring out...\n");
      assert(!boilerplate_loaded);
      if(run_boiler_plate(message)) {
        boilerplate_directory = string("Web/Client/Scenarios/")+message;
        boilerplate_loaded = true;
        student_code_loaded = false;
        student_code_preload = false;
      }
      else {
        boilerplate_loaded = false;
        student_code_loaded = false;
        student_code_preload = false;
      }
    }
  }
}
