using namespace std;

#include "BehaviourCreator.h"
#include "engine/BasicBehaviour.h"

#include  "Plans/Behaviours/DriveForward.h"

#include  "Plans/Behaviours/Stop.h"

namespace alica
{

BehaviourCreator::BehaviourCreator()
{
}

BehaviourCreator::~BehaviourCreator()
{
}

shared_ptr<BasicBehaviour> BehaviourCreator::createBehaviour(long behaviourConfId)
{
  switch (behaviourConfId)
  {

    case 1414427354149:
      return make_shared<DriveForward>();
      break;

    case 1413992626194:
      return make_shared<Stop>();
      break;

    default:
      cerr << "BehaviourCreator: Unknown behaviour requested: " << behaviourConfId << endl;
      throw new exception();
      break;
  }
}
}
