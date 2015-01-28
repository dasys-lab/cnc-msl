#ifndef DomainBehaviour_H_
#define DomainBehaviour_H_

#include "engine/BasicCondition.h"
#include <MSLWorldModel.h>
#include <Situation.h>
namespace alica
{
class DomainCondition : public BasicCondition
{
public:
  DomainCondition();
  virtual ~DomainCondition();
  msl::MSLWorldModel* wm;
};
} /* namespace alica */

#endif /* DomainBehaviour_H_ */

