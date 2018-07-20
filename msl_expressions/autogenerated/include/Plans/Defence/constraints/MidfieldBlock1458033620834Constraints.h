#ifndef MidfieldBlockCONSTRAINT_H_
#define MidfieldBlock_H_
#include "engine/BasicConstraint.h"
#include <memory>

using namespace std;
using namespace alica;

namespace alica
{
    class ProblemDescriptor;
    class RunningPlan;
}

namespace alicaAutogenerated
{

    class Constraint1458033723845 : public BasicConstraint
    {
        void getConstraint(shared_ptr<ProblemDescriptor> c, shared_ptr<RunningPlan> rp);
    };

} /* namespace alica */

#endif /* MidfieldBlockCONSTRAINT_H_ */
