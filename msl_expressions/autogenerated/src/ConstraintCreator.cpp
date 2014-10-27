#include "ConstraintCreator.h"
#include <iostream>

#include  "Plans/constraints/WM161413992564408Constraints.h"

using namespace std;
using namespace alicaAutogenerated;

namespace alica
{

ConstraintCreator::ConstraintCreator()
{
}

ConstraintCreator::~ConstraintCreator()
{
}

shared_ptr<BasicConstraint> ConstraintCreator::createConstraint(long constraintConfId)
{
  switch (constraintConfId)
  {

    default:
      cerr << "ConstraintCreator: Unknown constraint requested: " << constraintConfId << endl;
      throw new exception();
      break;
  }
}

}
