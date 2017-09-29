#ifndef Duel_H_
#define Duel_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1450178699265) ENABLED START*/ //Add additional includes here
#include <msl_robot/robotmovement/MovementQuery.h>
namespace geometry
{
    class CNPointAllo;
}
/*PROTECTED REGION END*/
namespace alica
{
    class Duel : public DomainBehaviour
    {
    public:
        Duel();
        virtual ~Duel();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1450178699265) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1450178699265) ENABLED START*/ //Add additional protected methods here
        double wheelSpeed;
        double fieldLength;
        double fieldWidth;
        double translation;
        double robotRadius;
        double radiusToCheckOpp;
        double radiusToCheckOwn;
        int direction;
        int itcounter;
        unsigned long entryTime;
        signed long long freeTime;
        unsigned long duelMaxTime;
        bool friendlyBlocked;
        vector<bool> hadClosestOpp;
        bool pointLeftOfVec(const geometry::CNVecAllo &lineVector, const geometry::CNVecAllo &pointToCheck) const;
        msl::MovementQuery query;
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1450178699265) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* Duel_H_ */
