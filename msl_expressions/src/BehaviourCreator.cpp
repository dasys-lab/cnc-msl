using namespace std;

#include "BehaviourCreator.h"
#include "engine/BasicBehaviour.h"

#include  "Stop.h"

namespace alica {

BehaviourCreator::BehaviourCreator() {
}

BehaviourCreator::~BehaviourCreator() {
}

unique_ptr<BasicBehaviour> BehaviourCreator::createBehaviour(
    long behaviourConfId) {
  unique_ptr < alica::BasicBehaviour > beh;
  switch (behaviourConfId) {

    case 1413992604875
    return make_shared<Stop>();
    break;

  default:
    cerr << "BehaviourCreator: Unknown behaviour requested: " << behaviourConfId
        << endl;
    throw new exception();
    break;
  }
}

}
