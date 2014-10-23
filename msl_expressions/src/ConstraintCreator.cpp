using namespace std;

#include "ConstraintCreator.h"
#include <iostream>

#include  "WM161413992564408Constraints.h"

namespace alica {

ConstraintCreator::ConstraintCreator() {
}

ConstraintCreator::ConstraintCreator() {
}

shared_ptr<BasicConstraint> ConstraintCreator::createConstraint(
    long constraintConfId) {
  switch (constraintConfId) {

    case 1413992578046
    return make_shared<Constraint1413992578046>();
    break;

  default:
    cerr << "ConstraintCreator: Unknown constraint requested: "
        << constraintConfId << endl;
    throw new exception();
    break;
  }
}

}
