// http://motion.pratt.duke.edu/klampt/tutorial_simulation.html

#include "PythonWrapper.h"
#include <unistd.h> //to get sleep function TODO: not cross-platform
#include <iostream>
#include <fstream>
#include <string> 
using namespace std;

#include "websocket.h"

bool boilerplate_loaded = false;
bool student_code_loaded = false;

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
      websocket_send("S"+string(data));
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
      printf("[python stdout] %s\n",data);
      websocket_send("C"+string(data));
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
      printf("[python stderr] %s\n",data);
      websocket_send("E"+string(data));
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
                       (std::istreambuf_iterator<char>()    ) );
  //std::cout << content;
  return content;
}

void initialize_python_interpreter()
{
  printf("Initializating python interpreter\n");
  //Py_SetProgramName("KlamptWebPython");  /* optional but recommended */
  Py_Initialize();
  
  Py_InitModule("emb", EmbMethods); //setup embedded methods
  Py_InitModule("log", logMethods); //setup stdio capture  
}

bool run_boiler_plate(const string& which)
{
   string boilerplate="boilerplate_"+which+".py";

   printf("  running boilerplate code %s\n",which.c_str()); //TODO, allow client to specify boiler plate
   std::string boiler_plate=load_file(boilerplate);

   if(boiler_plate.size()==0) //okay now lets try to find the file in a different place
   {
      boiler_plate=load_file("./Web/Client/Scenarios/"+boilerplate);
   }

   if(boiler_plate.size()!=0)
   {
      printf("    found the boiler plate!\n");
      int res = PyRun_SimpleString(boiler_plate.c_str());
      if(res < 0 || PyErr_Occurred()) {
        printf("   error while running boiler plate %s?\n",which.c_str());
        PyErr_Clear();
        return false;
      }
   }
   else {
      printf("    We weren't able to properly load the boiler plate %s\n",which.c_str());
      return false;
   }
   std::string wrapper=load_file("Web/Server/wrapper.py");
   if(wrapper.size()!=0)
   {
      printf("    found the boiler plate wrapper!\n");
      int res = PyRun_SimpleString(wrapper.c_str());
      if(res < 0 || PyErr_Occurred()) {
        printf("   error while running wrapper.py?\n");
        PyErr_Clear();
        return false;
      }
   }
   else {
      printf("    We weren't able to properly load the wrapper\n");
      return false;
   }
   return true;
}

void shutdown_python_interpreter()
{
   printf("Shutting down Python interpreter\n");
   Py_Finalize();
}

void handleIncomingMessage(string message)
{
   printf("received incoming message!\n"); 

   if(message.size()>=1) //TODO, actually have prefix to route message
   {
      char routing=message[0];
      message.erase(0, 1); //remove routing prefix

      if(routing=='A')
      {
         printf("  user would like to advance frame\n");
         if(!boilerplate_loaded || !student_code_loaded) {
           printf("  Code is not updated, returning.\n");
         }
         else {
           PyRun_SimpleString("wrapper_advance()\n");
         }
      }
      if(routing=='C')
      {  
        printf("  user would like to add some student code\n");
        student_code_loaded = false;

        if(!boilerplate_loaded) {
          printf("Boilerplate failed to load, not proceeding.\n");
          return;
        }

        PyObject* stub_module = PyImport_ImportModule("stub");\
        if(stub_module == NULL) {
          printf("Uh... couldn't load stub.py?\n");
          return;
        }
        PyObject* main_module = PyImport_AddModule("__main__");
        if(main_module == NULL) {
          printf("Uh... couldn't add __main__ module?\n");
          return;
        }
        PyObject_SetAttrString(main_module, "stub", stub_module);
        PyObject* stub_dict = PyModule_GetDict(stub_module);
        PyObject *key, *value;
        Py_ssize_t pos = 0;

        /*
        printf("Before values\n");
        while (PyDict_Next(stub_dict, &pos, &key, &value)) {
            PyObject_Print(key,stdout,Py_PRINT_RAW);
            printf("\n");
        }
        */
        PyObject* res = PyRun_String(message.c_str(),Py_file_input,stub_dict,stub_dict);
        /*
        printf("After values\n");
        pos = 0;
        while (PyDict_Next(stub_dict, &pos, &key, &value)) {
            PyObject_Print(key,stdout,Py_PRINT_RAW);
            printf("\n");
        }
        */
        if(!res) {
          printf("   Error running submitted code.\n");
          Py_XDECREF(stub_dict);
          return;
        }
        if(PyErr_Occurred()) {
          printf("  An exception occurred while running client code\n");
          PyErr_Clear();
          return;
        }
        Py_XDECREF(res);
        Py_XDECREF(stub_dict);
        //PyRun_SimpleString(message.c_str());

        printf("Running boilerplate_start()...\n");
        int res2 = PyRun_SimpleString("wrapper_start()\n");
        if(res2 >= 0 && !PyErr_Occurred()) {
          printf("   Loaded.\n");
          student_code_loaded = true;
        }
        else {
          printf("  Error occurred while running boilerplate_start()\n");
          PyErr_Clear();
        }
      }
      if(routing=='B')
      {
         if(run_boiler_plate(message)) {
           boilerplate_loaded = true;
           student_code_loaded = false;
         }
         else {
            boilerplate_loaded = false;
            student_code_loaded = false;
         }
      }
   }
}
