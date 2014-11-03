using namespace std;

#include "BehaviourCreator.h"
#include "engine/BasicBehaviour.h"

#include  "Plans/Behaviours/GetBall.h"

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

    case 1414828313541:

    case 1414840399972:

      return make_shared<GetBall>();
      break;

    case 1414840446941:

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
