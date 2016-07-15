#ifndef DJZ_PYTHON_WRAPPER
#define DJZ_PYTHON_WRAPPER

#include <string>
using namespace std;

#include <Python.h>

void initialize_python_interpreter();
void shutdown_python_interpreter();

void handleIncomingMessage(string message);


#endif
