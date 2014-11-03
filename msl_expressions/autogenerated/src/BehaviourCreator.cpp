using namespace std;

#include "BehaviourCreator.h"
#include "engine/BasicBehaviour.h"

#include  "Plans/Behaviours/DriveForward.h"

#include  "Plans/Behaviours/Stop.h"

#include  "Plans/Behaviours/DribbleToPoint.h"

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

    case 1414752423981:

      return make_shared<DribbleToPoint>();
      break;

    default:
      cerr << "BehaviourCreator: Unknown behaviour requested: " << behaviourConfId << endl;
      throw new exception();
      break;
  }
}
}
