#include <log4cxx/logger.h>
#include <KrisLibrary/Logger.h>
#include "LineReader.h"
#include <KrisLibrary/utils/ioutils.h>
#include <KrisLibrary/errors.h>
#include <sstream>

//reads a c-style token, getting rid of whitespace in front
//of line.  Reads alphanumeric characters, '#' and '_'
bool ReadCToken(istream& in,string& str)
{
  str.erase();
  int c;
  int mode=0;
  while((c=in.peek()) != EOF) {
    switch(mode) {
    case 0:
      if(isspace(c)) {  //stay in mode 0
      }
      else if(isalnum(c) || c=='_' || c=='#') {  //start reading token
	str += (char)c;
	mode = 1;
      }
      else {  //non-token character
	LOG4CXX_ERROR(KrisLibrary::logger(),"Started reading at a non-token character "<<c<<"\n");
	return false;
      }
      break;
    case 1:
      if(isalnum(c) || c=='_' || c=='#') {
	str += c;
      }
      else {
	LOG4CXX_INFO(KrisLibrary::logger(),"Finished reading at "<<c<<" string is "<<str<<"\n");
	return true;
      }
      break;
    }
    //take next character
    c=in.get();
  }
  if(in.eof()) {
    LOG4CXX_ERROR(KrisLibrary::logger(),"Encountered eof"<<"\n");
    if(mode == 1) return true;
    else return false;
  }
  LOG4CXX_ERROR(KrisLibrary::logger(),"Failed read"<<"\n");
  return false;
}

//same as above, returns rest of line
bool ReadTokenLine(istream& in,string& str,string& line)
{
  if(!in) return false;
  if(!ReadCToken(in,str)) return false;
  stringbuf buf;
  in.get(buf,'\n');
  if(!in) {
    LOG4CXX_ERROR(KrisLibrary::logger(),"ReadTokenLine(): failed to get args"<<"\n");
    LOG4CXX_ERROR(KrisLibrary::logger(),"str="<<str<<"\n");
    return false;
  }
  line = buf.str();
  char c;
  in.get(c);
  if(in.eof()) return true;
  else if(!in) {
    LOG4CXX_ERROR(KrisLibrary::logger(),"ReadTokenLine(): Failed reading trailing endline?"<<"\n");
    return false;
  }
  return true;
}



bool LineReader::Read()
{
  mode = 0;
  bool res=SimpleParser::Read();
  if(!res) return false;
  if(mode != 0) {
    LOG4CXX_INFO(KrisLibrary::logger(),"LineReader: did not end properly\n");
    return false;
  }
  return true;
}

SimpleParser::Result LineReader::InputToken(const string& word)
{
  string line;
  switch(mode) {
  case 0:
    if(word != "begin") {
      LOG4CXX_ERROR(KrisLibrary::logger(),"Couldn't read begin on line "<<lineno<<"\n");
      return Error;
    }
    else {
      mode = 1;
      return Continue;
    }
    break;
  case 1:  //begin name
    if(!ReadLine(line)) { 
      LOG4CXX_ERROR(KrisLibrary::logger(),"Couldn't read line after begin"<<"\n"); 
      return Error;
    }
    else {
      stringstream ss; ss.str(line);
      if(!Begin(word,ss)) {
	LOG4CXX_ERROR(KrisLibrary::logger(),"Couldn't do begin with args "<<line<<"\n");
	return Error;
      }
      else if(ss.fail()) {
	LOG4CXX_ERROR(KrisLibrary::logger(),"Begin() failed to read arguments correctly"<<"\n");
	return Error;
      }
      mode = 2;
      return Continue;
    }
    break;
  case 2:
    if(word == "end") {
      if(End()) {
	mode = 0;
	return Stop;
      }
      else {
	LOG4CXX_ERROR(KrisLibrary::logger(),"Failed End()"<<"\n");
	return Error;
      }
    }
    else {
      curitem = word;
      mode = 3;
      return Continue;
    }
    break;
  case 3:
    LOG4CXX_ERROR(KrisLibrary::logger(),"Expecting an = sign, instead got "<<word<<" on line "<<lineno<<"\n");
    return Error;
  default:
    LOG4CXX_ERROR(KrisLibrary::logger(),"Unknown mode: "<<mode<<"\n");
    return Error;
  }
  AssertNotReached();
  return Error;
}

SimpleParser::Result LineReader::InputPunct(const string& punct)
{
  if(mode == 3) {
    if(punct == "=") {
      string line;
      ReadLine(line);
      stringstream ss; ss.str(line);
      if(!Assign(curitem,ss)) {
	LOG4CXX_ERROR(KrisLibrary::logger(),"Assign() failed for item "<<curitem<<", arguments "<<ss.str()<<" on line "<<lineno<<"\n");
	return Error;
      }
      if(!ss.eof() && ss.fail()) {
	LOG4CXX_ERROR(KrisLibrary::logger(),"Assign() failed to read rhs "<<curitem<<" correctly"<<"\n");
	LOG4CXX_INFO(KrisLibrary::logger(),"Stringbuf "<<ss.str()<<"\n");
	LOG4CXX_INFO(KrisLibrary::logger(),"Position "<<ss.tellg()<<"\n");
	return Error;
      }
      mode = 2;
      return Continue;
    }
  }
  LOG4CXX_ERROR(KrisLibrary::logger(),"Error, reading punctuation characters: "<<punct<<" on line "<<lineno<<"\n");
  return Error;
}
