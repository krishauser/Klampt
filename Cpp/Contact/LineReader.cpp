#include "LineReader.h"
#include <KrisLibrary/utils/ioutils.h>
#include <KrisLibrary/errors.h>
#include <sstream>

namespace Klampt {

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
	cerr<<"Started reading at a non-token character "<<c<<endl;
	return false;
      }
      break;
    case 1:
      if(isalnum(c) || c=='_' || c=='#') {
	str += c;
      }
      else {
	cout<<"Finished reading at "<<c<<" string is "<<str<<endl;
	return true;
      }
      break;
    }
    //take next character
    c=in.get();
  }
  if(in.eof()) {
    cerr<<"Encountered eof"<<endl;
    if(mode == 1) return true;
    else return false;
  }
  cerr<<"Failed read"<<endl;
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
    cerr<<"ReadTokenLine(): failed to get args"<<endl;
    cerr<<"str="<<str<<endl;
    return false;
  }
  line = buf.str();
  char c;
  in.get(c);
  if(in.eof()) return true;
  else if(!in) {
    cerr<<"ReadTokenLine(): Failed reading trailing endline?"<<endl;
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
    printf("LineReader: did not end properly\n");
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
      cerr<<"Couldn't read begin on line "<<lineno<<endl;
      return Error;
    }
    else {
      mode = 1;
      return Continue;
    }
    break;
  case 1:  //begin name
    if(!ReadLine(line)) { 
      cerr<<"Couldn't read line after begin"<<endl; 
      return Error;
    }
    else {
      stringstream ss; ss.str(line);
      if(!Begin(word,ss)) {
	cerr<<"Couldn't do begin with args "<<line<<endl;
	return Error;
      }
      else if(ss.fail()) {
	cerr<<"Begin() failed to read arguments correctly"<<endl;
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
	cerr<<"Failed End()"<<endl;
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
    cerr<<"Expecting an = sign, instead got "<<word<<" on line "<<lineno<<endl;
    return Error;
  default:
    cerr<<"Unknown mode: "<<mode<<endl;
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
	cerr<<"Assign() failed for item "<<curitem<<", arguments "<<ss.str()<<" on line "<<lineno<<endl;
	return Error;
      }
      if(!ss.eof() && ss.fail()) {
	cerr<<"Assign() failed to read rhs "<<curitem<<" correctly"<<endl;
	cout<<"Stringbuf "<<ss.str()<<endl;
	cout<<"Position "<<ss.tellg()<<endl;
	return Error;
      }
      mode = 2;
      return Continue;
    }
  }
  cerr<<"Error, reading punctuation characters: "<<punct<<" on line "<<lineno<<endl;
  return Error;
}

} //namespace Klampt