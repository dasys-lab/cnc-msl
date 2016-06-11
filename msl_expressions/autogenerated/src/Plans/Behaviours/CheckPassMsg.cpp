using namespace std;
#include "Plans/Behaviours/CheckPassMsg.h"

/*PROTECTED REGION ID(inccpp1457441479350) ENABLED START*/ //Add additional includes here
#include <WhiteBoard.h>
#include <MSLWorldModel.h>
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1457441479350) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    CheckPassMsg::CheckPassMsg() :
            DomainBehaviour("CheckPassMsg")
    {
        /*PROTECTED REGION ID(con1457441479350) ENABLED START*/ //Add additional options here#
        receivedMsg = false;
        /*PROTECTED REGION END*/
    }
    CheckPassMsg::~CheckPassMsg()
    {
        /*PROTECTED REGION ID(dcon1457441479350) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void CheckPassMsg::run(void* msg)
    {
        /*PROTECTED REGION ID(run1457441479350) ENABLED START*/ //Add additional options here
        auto passMsg = wm->whiteBoard->getPassMsg();

        if (!receivedMsg && passMsg != nullptr)
        {
            receivedMsg = true;

            this->setSuccess(true);
        }

        /*PROTECTED REGION END*/
    }
    void CheckPassMsg::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1457441479350) ENABLED START*/ //Add additional options here
        receivedMsg = false;
        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1457441479350) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
