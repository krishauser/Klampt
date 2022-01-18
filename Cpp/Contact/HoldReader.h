#ifndef HOLD_READER_H
#define HOLD_READER_H

#include "Hold.h"
#include "LineReader.h"

namespace Klampt {

class HoldReader : public LineReader
{
public:
  HoldReader(istream& in);
  virtual bool Begin(const string& name,stringstream& args);
  virtual bool Assign(const string& item,stringstream& rhs);
  virtual bool End();

  Hold h;
};

} //namespace Klampt

#endif
