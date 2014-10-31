#ifndef CONTROLLER_H
#define CONTROLLER_H

namespace Controlling
{
class Controller {


public:
	Controller(int count);
	virtual int InitAllNodes() =0;

	virtual int IsFunctional() =0;
	virtual int ActualRPM(int which) =0;
	virtual	void SetDemandRPM(int which, int rpm) =0;
	virtual void EmergencyShutdown() =0;
	virtual void PrintStatus() =0;


	virtual int EnableAllNodes() =0;
	virtual int DisableAllNodes()=0;
	virtual int DisableNode(int which)=0;
	virtual int EnableNode(int which)=0;

//	virtual int Reset();
//	int SendCommand(int towho,unsigned char op,unsigned char* data, int datalen);
protected:
	int initialised;
	int controllerCount;
	
};
}

#endif
