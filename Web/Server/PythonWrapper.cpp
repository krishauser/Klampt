// http://motion.pratt.duke.edu/klampt/tutorial_simulation.html
#include <QtCore/QDebug>
#include "PythonWrapper.h"
#include <unistd.h> //to get sleep function TODO: not cross-platform
#include <iostream>
#include <fstream>


PythonWrapper *currentServer; //this breaks thread safe-ness. will need to fork to seperate processes

PythonWrapper::~PythonWrapper() {
}

PythonWrapper::PythonWrapper(int argc, char **argv)
{
   this->argc=argc;
   this->argv=argv;
}


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
      currentServer->send("S"+QString(data));
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

void PythonWrapper::send(QString message)
{
   emit sendData(message);
}

PyObject* log_CaptureStdout(PyObject* self, PyObject* args)
{
   PyObject *a;

   if (!PyArg_UnpackTuple(args, "func", 1, 1, &a))
      return NULL;        

   if(PyString_Check(a))
   {
      char *data=PyString_AsString(a);
      printf("from stdout length: %ld text: %s\n",strlen(data),data);
      currentServer->send("C"+QString(data));
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
      printf("from stderr length: %ld text: %s\n",strlen(data),data);
      currentServer->send("E"+QString(data));
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
     printf("  file %s doesn't exist!\n",filename.c_str());
     return std::string("");
  }

  std::string content( (std::istreambuf_iterator<char>(ifs) ),
                       (std::istreambuf_iterator<char>()    ) );
  //std::cout << content;
  return content;
}

void PythonWrapper::initialize()
{
  printf("Initializating python interpreter\n");
  currentServer=this; 
  //Py_SetProgramName("KlamptWebPython");  /* optional but recommended */
  Py_Initialize();
  
  Py_InitModule("emb", EmbMethods); //setup embedded methods
  Py_InitModule("log", logMethods); //setup stdio capture

  printf("running boilerplate code\n");
  std::string boiler_plate=load_file("boilerplate1.py");

  if(boiler_plate.size()==0)
  {
      boiler_plate=load_file("./Web/Server/boilerplate1.py");
  }
  if(boiler_plate.size()!=0)
  {
     printf("  found the boiler plate!\n");
     PyRun_SimpleString(boiler_plate.c_str());
  }
  else
     printf("  We weren't able to properly load the boiler plate\n");
}

void PythonWrapper::incomingMessage(QString message)
{
   printf("received incoming message!\n"); 

   if(message.size()>1) //TODO, actually have prefix to route message
      PyRun_SimpleString(message.toStdString().c_str());
   
   PyRun_SimpleString("boilerplate_advance()\n");

}
