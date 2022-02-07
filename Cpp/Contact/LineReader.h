#ifndef CONTACT_UTIL_LINE_READER_H
#define CONTACT_UTIL_LINE_READER_H

#include <KrisLibrary/utils/SimpleParser.h>
#include <string>

namespace Klampt {
  using namespace std;

/** This is a helper class to read text data of the form
 *
 * begin NAME [args]
 *    ITEM1 = [args]
 *    ITEM2 = [args]
 * end
 * where NAME and ITEM_ are single words, [args] are arbitrary
 * arguments up to the endline.
 *
 * Subclasses should fill in Begin(), Assign(), and End().  These
 * methods return true if the name and arguments are valid.
 */
class LineReader  : public SimpleParser
{
public:
  LineReader(istream& in) : SimpleParser(in),mode(0) {}
  virtual ~LineReader() {}
  bool Read();

  //overrides of SimpleParser
  virtual Result InputToken(const string& word);
  virtual Result InputPunct(const string& punct);
  virtual Result InputEndLine() { return Continue; }

  ///subclass should just fill these in
  virtual bool Begin(const string& name,stringstream& args)=0;
  virtual bool Assign(const string& item,stringstream& rhs)=0; 
  virtual bool End() { return true; }

  //0=nothing read
  //1=reading name of begin
  //2=reading items
  //3= read item name, waiting for assignment
  int mode;  
  string curitem;
};

} //namespace Klampt

#endif
