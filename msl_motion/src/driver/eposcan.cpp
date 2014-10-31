#include "driver/eposcan.h"
#include <stdio.h>
#include "settings.h"
#include "gonzales.h"
#include "ros/ros.h"


extern controller_settings current_settings;

using namespace Controlling;


void EposCan::Trigger(int enabled)
{
	gettimeofday(&cur_triggertime,NULL);
	//epos->demandRPM = 0;
	lifeguardcounter +=TIMEDIFFMS(cur_triggertime,last_triggertime);
	
	//printf("ttl %ld / %ld -- %ld\n", lifeguardcounter,EPOS_LIFE_GUARD_TIME,TIMEDIFFMS(cur_triggertime,last_triggertime));
	//exit(1);
	if (lifeguardcounter >= current_settings.nodeGuardTime/2.0)  //send heartbeat
	{
		SendLifeGuard();
		
		ReadErrorRegister();
	}
	
	last_triggertime = cur_triggertime;
	
	SendSynch();
	
	if (enabled > 0)
	{
		for(int i=0; i < controllerCount; i++)
		{
			if (IsEnabled(i)<=0)
			{
				printf("Enabling %d\n",i+1);
				EnableNode(i+1);
			}
		}
		SendVelocity();
	}
	else if (enabled == 0)
	{
		for(int i=0; i<controllerCount; i++)
		{
			if(IsEnabled(i) > 0)
			{
				printf("Disabling %d\n",i+1);
				DisableNode(i+1);
			}
		}
	}
	else
	{
		SendVelocity();
	}
	//PrintStatus();
}


int EposCan::InitAllNodes()
{
	printf("INITING\n");
	int ret;
	status = initing;
	SendLifeGuard();
	ResetAllNodes();
	usleep(60000);
	ResetCommunication();
	usleep(10000);
	
	int allbooted=0;
	int counter = 0;
	
	while(allbooted ==0)
	{
		allbooted = 1;
		for(int i=0; i<controllerCount;i++)
		{
			allbooted &= ((epos+i)->nmtStatus != booting);
		}
		
		if(allbooted==0)
		{
			usleep(50000);
			SendLifeGuard();
		}
		
		if (counter++ > 100)
		{
			ROS_ERROR_THROTTLE(1,"Booting failed, retrying!");
			return -1;
		}
	}
	
	SendLifeGuard();
	
	if(ResetFaults()<=0)
	{
		ROS_ERROR_THROTTLE(1,"Could not reset fault register!");
		//fprintf(stderr,"Could not reset fault register!\n");
		return -1;
	}
	
	ret=DisableAllNodes();	
	
	if(ret<=0)
	{
		ROS_ERROR_THROTTLE(1,"Could not disable all nodes securely!");
		//fprintf(stderr,"Could not disable all nodes securely!\n");
		return -1;
	}
	
	for(int i=0; i<controllerCount; i++)
	{
		ret = InitNode(i+1);
		SendLifeGuard();
		
		if(ret<=0)
		{
			ROS_ERROR_THROTTLE(1,"Unable to initialise Node %d",i+1);
			//fprintf(stderr,"Unable to initialise Node %d\n",i+1);
			return -1;
		}
	}
	
	printf("Configuration successful, enabling...\n");
	
	if (EnableAllNodes()<=0)
	{
		ROS_ERROR_THROTTLE(1,"Unable to enable all nodes!");
		return -1;
	}
	
	status = running;
	printf("Enabled!\n");
	ROS_INFO_THROTTLE(2,"Epos init ok!");
	return 1;
}


int EposCan::InitNode(int nodeid)
{
	unsigned char buf[8];
	
	//RESET FAULTS:
	buf[0] = 0x2B; buf[1] = 0x40; buf[2] = 0x60;buf[3]=0x00; buf[4]=0x80; buf[5]=0x0;
	can->SendCanMsg(CAN_ID_SDO_WRITE|nodeid,buf,6);
	if(SdoDownloadConfirmation(nodeid,buf)<=0)	return 0;
	
	//SHUTDOWN OPTION
	buf[0] = 0x2B; buf[1] = 0x5B; buf[2] = 0x60;buf[3]=0x00; buf[4]=0x00; buf[5]=0x0;
	can->SendCanMsg(CAN_ID_SDO_WRITE|nodeid,buf,6);
	if(SdoDownloadConfirmation(nodeid,buf)<=0)	return 0;
	
	//DISABLE OPTION:
	buf[0] = 0x2B; buf[1] = 0x5C; buf[2] = 0x60;buf[3]=0x00; buf[4]=0x00; buf[5]=0x0;
	can->SendCanMsg(CAN_ID_SDO_WRITE|nodeid,buf,6);
	if(SdoDownloadConfirmation(nodeid,buf)<=0)	return 0;
	
	//Fault Reaction
	buf[0] = 0x2B; buf[1] = 0x5E; buf[2] = 0x60;buf[3]=0x00; buf[4]=0x00; buf[5]=0x0;
	can->SendCanMsg(CAN_ID_SDO_WRITE|nodeid,buf,6);
	if(SdoDownloadConfirmation(nodeid,buf)<=0)	return 0;
	
	//Connection Failure:
	buf[0] = 0x2B; buf[1] = 0x07; buf[2] = 0x60;buf[3]=0x00; buf[4]=0x02; buf[5]=0x0;
	can->SendCanMsg(CAN_ID_SDO_WRITE|nodeid,buf,6);
	if(SdoDownloadConfirmation(nodeid,buf)<=0)	return 0;
	
	//Node Guarding:
	printf("Node Guard Time: %dms\n",current_settings.nodeGuardTime);
	buf[0] = 0x2B; buf[1] = 0x0C; buf[2]=0x10; buf[3]=0x00; SHORT2BYTEPOS(current_settings.nodeGuardTime,buf,4);
	can->SendCanMsg(CAN_ID_SDO_WRITE|nodeid,buf,6);
	if(SdoDownloadConfirmation(nodeid,buf)<=0)	return 0;
	
	//Set Lifetime factor to 1, not used
	buf[0] = 0x2F; buf[1] = 0x0D; buf[2]=0x10; buf[3]=0x00; buf[4]=0x01; buf[5]=0x00;
	can->SendCanMsg(CAN_ID_SDO_WRITE|nodeid,buf,6);
	if(SdoDownloadConfirmation(nodeid,buf)<=0)	return 0;
	
	//MODE VELOCITY Profile:
	buf[0] = 0x2F; buf[1] = 0x60; buf[2]=0x60; buf[3]=0x00; buf[4]=0x03; buf[5]=0x0; //one byte to 6060 00 
	can->SendCanMsg(CAN_ID_SDO_WRITE|nodeid,buf,6);
	if(SdoDownloadConfirmation(nodeid,buf)<=0)	return 0;
	
	//MAX VELOCITY
	printf("MaxRPM: %d\n",current_settings.maxRPM); //With Gear
	buf[0] = 0x22; buf[1] = 0x7F; buf[2]=0x60; buf[3]=0x00; INT2BYTEPOS(current_settings.maxRPM,buf,4);
	can->SendCanMsg(CAN_ID_SDO_WRITE|nodeid,buf,8);
	if(SdoDownloadConfirmation(nodeid,buf)<=0)	return 0;
	
	//printf("MaxRPM: %d\n",current_settings.maxRPM); //Without gear
	buf[0] = 0x22; buf[1] = 0x10; buf[2]=0x64; buf[3]=0x04; INT2BYTEPOS(current_settings.maxRPM,buf,4);
	can->SendCanMsg(CAN_ID_SDO_WRITE|nodeid,buf,8);
	if(SdoDownloadConfirmation(nodeid,buf)<=0)	return 0;
	
	//MAX ACCEL:
	printf("MaxAccel: %d\n",current_settings.max_accel);
	buf[0] = 0x22; buf[1] = 0x83; buf[2]=0x60; buf[3]=0x00; INT2BYTEPOS(current_settings.max_accel,buf,4);
	can->SendCanMsg(CAN_ID_SDO_WRITE|nodeid,buf,8);
	if(SdoDownloadConfirmation(nodeid,buf)<=0)	return 0;
	
	//MAX DECCEL:
	printf("MaxDeccel: %d\n",current_settings.max_deccel);
	buf[0] = 0x22; buf[1] = 0x84; buf[2]=0x60; buf[3]=0x00; INT2BYTEPOS(current_settings.max_deccel,buf,4);
	can->SendCanMsg(CAN_ID_SDO_WRITE|nodeid,buf,8);
	if(SdoDownloadConfirmation(nodeid,buf)<=0)	return 0;
	
	//GEAR RATIO
	printf("Gear Ratio Numerator: %d\n",current_settings.gear_ratio_nominator);
	buf[0] = 0x22; buf[1] = 0x30; buf[2]=0x22; buf[3]=0x01; INT2BYTEPOS(current_settings.gear_ratio_nominator,buf,4);
	can->SendCanMsg(CAN_ID_SDO_WRITE|nodeid,buf,8);
	if(SdoDownloadConfirmation(nodeid,buf)<=0)	return 0;
	
	printf("Gear Ratio Denomerator: %d\n",current_settings.gear_ratio_denominator);
	buf[0] = 0x22; buf[1] = 0x30; buf[2]=0x22; buf[3]=0x02; INT2BYTEPOS(current_settings.gear_ratio_denominator,buf,4);
	can->SendCanMsg(CAN_ID_SDO_WRITE|nodeid,buf,8);
	if(SdoDownloadConfirmation(nodeid,buf)<=0)	return 0;
	
	//CURRENT and VELOCITY control params set by tuning tool
	
	//Current Limits:
	printf("Cont. Amp Limit: %d\n",current_settings.max_continous_amp);
	buf[0] = 0x2B; buf[1] = 0x10; buf[2]=0x64; buf[3]=0x01; SHORT2BYTEPOS(current_settings.max_continous_amp,buf,4);
	can->SendCanMsg(CAN_ID_SDO_WRITE|nodeid,buf,6);
	if(SdoDownloadConfirmation(nodeid,buf)<=0)	return 0;
	
	printf("Max Amp Limit: %d\n",current_settings.max_amp);
	buf[0] = 0x2B; buf[1] = 0x10; buf[2]=0x64; buf[3]=0x02; SHORT2BYTEPOS(current_settings.max_amp,buf,4);
	can->SendCanMsg(CAN_ID_SDO_WRITE|nodeid,buf,6);
	if(SdoDownloadConfirmation(nodeid,buf)<=0)	return 0;
	
	printf("Thermal Constant Winding: %d\n",current_settings.thermalConstantWinding);
	buf[0] = 0x2B; buf[1] = 0x10; buf[2]=0x64; buf[3]=0x05; SHORT2BYTEPOS(current_settings.thermalConstantWinding,buf,4);
	can->SendCanMsg(CAN_ID_SDO_WRITE|nodeid,buf,6);
	if(SdoDownloadConfirmation(nodeid,buf)<=0)	return 0;
	
	
	//--------------CONFIGURE PDO TxPDO 1: (main com channel)
	printf("Configuring PDOs on node %d...\n",nodeid);
	
	//Type to 0x01 (transmit after sync).
	buf[0] = 0x2F; buf[1] = 0x00; buf[2] = 0x18; buf[3] = 0x02; buf[4]=0x01; buf[5]=0x00;
	can->SendCanMsg(CAN_ID_SDO_WRITE|nodeid,buf,6);
	if(SdoDownloadConfirmation(nodeid,buf)<=0) return 0;
	
	//Clear Objects in TxPDO 1:
	buf[0] = 0x2F; buf[1] = 0x00; buf[2] = 0x1A; buf[3] = 0x00; buf[4]=0x00; buf[5]=0x00;
	can->SendCanMsg(CAN_ID_SDO_WRITE|nodeid,buf,6);
	if(SdoDownloadConfirmation(nodeid,buf)<=0) return 0;
	
	//write new objects to TxPDO 1:
	//first object is actual velocity!	
	buf[0] = 0x22; buf[1] = 0x00; buf[2] = 0x1A; buf[3] = 0x01; INT2BYTEPOS(EPOS_ADDR_VELOCITY_ACTUAL,buf,4);
	can->SendCanMsg(CAN_ID_SDO_WRITE|nodeid,buf,8);
	if(SdoDownloadConfirmation(nodeid,buf)<=0) return 0;
	
	//second object is actual current:	
	buf[0] = 0x22; buf[1] = 0x00; buf[2] = 0x1A; buf[3] = 0x02; INT2BYTEPOS(EPOS_ADDR_CURRENT_ACTUAL,buf,4);
	can->SendCanMsg(CAN_ID_SDO_WRITE|nodeid,buf,8);
	if(SdoDownloadConfirmation(nodeid,buf)<=0) return 0;
	
	//third object is status word:	
	buf[0] = 0x22; buf[1] = 0x00; buf[2] = 0x1A; buf[3] = 0x03; INT2BYTEPOS(EPOS_ADDR_STATUS,buf,4);
	can->SendCanMsg(CAN_ID_SDO_WRITE|nodeid,buf,8);
	if(SdoDownloadConfirmation(nodeid,buf)<=0) return 0;
	
	//activate:
	buf[0] = 0x2F; buf[1] = 0x00; buf[2] = 0x1A; buf[3] = 0x00; buf[4]=0x03; buf[5]=0x00;
	can->SendCanMsg(CAN_ID_SDO_WRITE|nodeid,buf,6);
	if(SdoDownloadConfirmation(nodeid,buf)<=0) return 0;
	
	//CONFIGURE PDO TxPDO 2: (event based status changes)
	//Type = 0xFF (transmit after change).
	//Type = 0xFD (transmit after RTR).
	
	/*
	buf[0] = 0x2F; buf[1] = 0x01; buf[2] = 0x18; buf[3] = 0x02; buf[4]=0x01; buf[5]=0x00;
	can->SendCanMsg(CAN_ID_SDO_WRITE|nodeid,buf,6);
	if(SdoDownloadConfirmation(nodeid,buf)<=0)	return 0;
	//time to 20ms (given as multiple of 100us)
	//buf[0] = 0x2B; buf[1] = 0x01; buf[2] = 0x18; buf[3] = 0x03; buf[4]=200; buf[5]=0x00;
	//can->SendCanMsg(CAN_ID_SDO_WRITE|nodeid,buf,6);
	//if(SdoDownloadConfirmation(nodeid,buf)<=0)	return 0;
	//Clear Objects:
	buf[0] = 0x2F; buf[1] = 0x01; buf[2] = 0x1A; buf[3] = 0x00; buf[4]=0x00; buf[5]=0x00;
	can->SendCanMsg(CAN_ID_SDO_WRITE|nodeid,buf,6);
	if(SdoDownloadConfirmation(nodeid,buf)<=0)	return 0;
	//write new objects to TxPDO 2:
	//first object is status word!	
	buf[0] = 0x22; buf[1] = 0x01; buf[2] = 0x1A; buf[3] = 0x01; INT2BYTEPOS(EPOS_ADDR_CURRENT_ACTUAL,buf,4);//INT2BYTEPOS(EPOS_ADDR_STATUS,buf,4);
	can->SendCanMsg(CAN_ID_SDO_WRITE|nodeid,buf,8);
	if(SdoDownloadConfirmation(nodeid,buf)<=0)	return 0;
	//activate;
	buf[0] = 0x2F; buf[1] = 0x01; buf[2] = 0x1A; buf[3] = 0x00; buf[4]=0x01; buf[5]=0x00;
	can->SendCanMsg(CAN_ID_SDO_WRITE|nodeid,buf,6);
	if(SdoDownloadConfirmation(nodeid,buf)<=0)	return 0;
	*/
	
	//CONFIGURE PDO RxPDO 1 for velocity setting:
	//Type to 0xFF (asynchronous).
	buf[0] = 0x2F; buf[1] = 0x00; buf[2] = 0x14; buf[3] = 0x02; buf[4]=0xFF; buf[5]=0x00;
	can->SendCanMsg(CAN_ID_SDO_WRITE|nodeid,buf,6);	
	if(SdoDownloadConfirmation(nodeid,buf)<=0)	return 0;
	
	//Clear Objects:
	buf[0] = 0x2F; buf[1] = 0x00; buf[2] = 0x16; buf[3] = 0x00; buf[4]=0x00; buf[5]=0x00;
	can->SendCanMsg(CAN_ID_SDO_WRITE|nodeid,buf,6);
	if(SdoDownloadConfirmation(nodeid,buf)<=0)	return 0;
	
	//write new objects to RxPDO 1:
	//first object is target velocity
	buf[0] = 0x22; buf[1] = 0x00; buf[2] = 0x16; buf[3] = 0x01; INT2BYTEPOS(EPOS_ADDR_TARGET_VELOCITY,buf,4);
	can->SendCanMsg(CAN_ID_SDO_WRITE|nodeid,buf,8);
	if(SdoDownloadConfirmation(nodeid,buf)<=0)	return 0;
	
	//second object is controlword
	buf[0] = 0x22; buf[1] = 0x00; buf[2] = 0x16; buf[3] = 0x02; INT2BYTEPOS(EPOS_ADDR_CONTROLWORD,buf,4);
	can->SendCanMsg(CAN_ID_SDO_WRITE|nodeid,buf,8);
	if(SdoDownloadConfirmation(nodeid,buf)<=0)	return 0;
	
	//activate:
	buf[0] = 0x2F; buf[1] = 0x00; buf[2] = 0x16; buf[3] = 0x00; buf[4]=0x02; buf[5]=0x00;
	can->SendCanMsg(CAN_ID_SDO_WRITE|nodeid,buf,6);
	if(SdoDownloadConfirmation(nodeid,buf)<=0)	return 0;
	
	// MISC Settings:
	// sensor supervision by hardware and software (0,0)
	// motor resistance measurement at first change (0),
	// somewhat better velocity estimation (1), rest reserved =
	buf[0] = 0x2B; buf[1] = 0x08; buf[2] = 0x20; buf[3] = 0x00; buf[4] = 0x08; buf[5]=0x01; //ROTATION CW
	can->SendCanMsg(CAN_ID_SDO_WRITE|nodeid,buf,6);
	if(SdoDownloadConfirmation(nodeid,buf)<=0)	return 0;
	
	return 1;
}


int EposCan::DisableAllNodes()
{
	int ret=1;
	//printf("Disabling all Nodes\n");
	unsigned char buf[6] = {0x2B,0x40,0x60,0x00,0x00,0x01}; //two byte to 6040 00
	for(unsigned char i=0; i<controllerCount; i++)
	{ 
		can->SendCanMsg(CAN_ID_SDO_WRITE|(i+1),buf,6);
		ret&=SdoDownloadConfirmation(i+1,buf);
	}
	return ret;
}


int EposCan::DisableNode(int which)
{
	int ret=1;
	unsigned char buf[6] = {0x2B,0x40,0x60,0x00,0x00,0x01}; //two byte to 6040 00
	can->SendCanMsg(CAN_ID_SDO_WRITE|which,buf,6);
	ret&=SdoDownloadConfirmation(which,buf);
	return ret;
}


int EposCan::EnableAllNodes()
{
	unsigned char buf[6] = {CAN_NMT_SET_OPERATIONAL,0x00,0x00,0x00,0x00,0x00};
	
	for(unsigned char i=0; i<controllerCount; i++)
	{
		buf[1] = i+1;
		can->SendCanMsg(CAN_ID_NMT,buf,2);
	}	
	SendLifeGuard();
	usleep(100000);
	int enabled = 1;
	
	for(int i=0; i<controllerCount; i++)
	{
		enabled &= (epos+i)->nmtStatus==operational;
		if(!enabled)
		{
			//fprintf(stderr,"Could not enable node %d\n",i+1);
			break;
		}
	}
	
	buf[0] = 0x2B; buf[1]=0x40; buf[2]=0x60; buf[3]=0x00; buf[4]=0x06; buf[5]=0x00; //shutdown
	
	for(unsigned char i=0; i<controllerCount; i++)
	{
		can->SendCanMsg(CAN_ID_SDO_WRITE|(i+1),buf,6);
		enabled&=SdoDownloadConfirmation(i+1,buf);
	}
	
	buf[0] = 0x2B; buf[1]=0x40; buf[2]=0x60; buf[3]=0x00; buf[4]=0x0F; buf[5]=0x00; //switchon
	
	for(unsigned char i=0; i<controllerCount; i++)
	{
		can->SendCanMsg(CAN_ID_SDO_WRITE|(i+1),buf,6);
		enabled&=SdoDownloadConfirmation(i+1,buf);
	}
	
	return enabled;
}


int EposCan::EnableNode(int which)
{
	unsigned char buf[6] = {CAN_NMT_SET_OPERATIONAL,0x00,0x00,0x00,0x00,0x00};
	buf[1] = which;
	can->SendCanMsg(CAN_ID_NMT,buf,2);
	
	SendLifeGuard();
	usleep(50000);
	
	int enabled;
	enabled = (epos+which-1)->nmtStatus==operational;
	
	if(!enabled)
	{
		ROS_ERROR_THROTTLE(1,"Could not enable node %d",which);
		//fprintf(stderr,"Could not enable node %d\n",which);
	}
	
	buf[0] = 0x2B; buf[1]=0x40; buf[2]=0x60; buf[3]=0x00; buf[4]=0x06; buf[5]=0x00; //shutdown
	can->SendCanMsg(CAN_ID_SDO_WRITE|which,buf,6);
	enabled&=SdoDownloadConfirmation(which,buf);
	
	buf[0] = 0x2B; buf[1]=0x40; buf[2]=0x60; buf[3]=0x00; buf[4]=0x0F; buf[5]=0x00; //switchon
	can->SendCanMsg(CAN_ID_SDO_WRITE|which,buf,6);
	enabled&=SdoDownloadConfirmation(which,buf);
	
	return enabled;
}


void EposCan::ReadErrorRegister()
{
	unsigned char buf[6] = {0x40,0x01,0x10,0x00,0x00,0x00}; //two byte to 6040 00
	for(unsigned char i=0; i<controllerCount; i++)
	{
		can->SendCanMsg(CAN_ID_SDO_READ|(i+1),buf,6);
	}
}


int EposCan::ResetFaults() {
	unsigned char buf[8];	
	//printf("Resetting faults!\n");
	for(unsigned char i=0; i < controllerCount; i++)
	{
		//RESET FAULTS:
		buf[0] = 0x2B; buf[1] = 0x40; buf[2] = 0x60;buf[3]=0x00; buf[4]=0x80; buf[5]=0x0;
		can->SendCanMsg(CAN_ID_SDO_WRITE|(i+1),buf,6);
		if(SdoDownloadConfirmation((i+1),buf)<=0)	return 0;
	}
	return 1;
}


void EposCan::EmergencyShutdown() // no confirmation, since event triggered!
{
	unsigned char buf[6] = {0x2B,0x40,0x60,0x00,0x00,0x01};
	for(unsigned char i=0; i<controllerCount; i++)
	{
		can->SendCanMsg(CAN_ID_SDO_WRITE|(i+1),buf,6);
	}
	//printf("SHUTDOWN SENT\n");
	ResetCommunication();
	
	//exit(1);
}


void EposCan::ResetCommunication()
{
	//printf("Resetting Communication\n");
	unsigned char buf[2] = {CAN_NMT_RESET_COMM,0x00};
	for(unsigned char i=0; i<controllerCount; i++)
	{
		buf[1] = i+1;
		(epos+i)->nmtStatus = booting;
		can->SendCanMsg(CAN_ID_NMT,buf,2);
	}
}


void EposCan::ResetAllNodes()
{
	//printf("Resetting Nodes\n");
	unsigned char buf[2] = {CAN_NMT_RESET_NODE,0x00};
	for(unsigned char i=0; i<controllerCount; i++)
	{
		buf[1] = i+1;
		(epos+i)->nmtStatus = booting;
		can->SendCanMsg(CAN_ID_NMT,buf,2);
	}
}


int EposCan::IsFunctional()
{
	EposStatus* cp = epos;
	timeval curtime;
	gettimeofday(&curtime,NULL);
	for(unsigned char i=0; i<controllerCount; i++)
	{
		if(cp->nmtStatus != operational)
		{
			//printf("Node %d is not operational!\n",i+1);
			return -1;
		}
		if(TIMEDIFFMS(curtime,cp->lastReceivedUpdate) > current_settings.communication_timeout)
		{
			ROS_ERROR_THROTTLE(1,"Node %d is not responding!",i+1);
			//printf("Node %d is not responding!\n",i+1);
			return -1;
		}
		else
		{
			//printf("Node %d last reponse is %ldms old\n",i+1,TIMEDIFFMS(curtime,cp->lastReceivedUpdate) );
		}
		if(HasStatusError(i))
		{
			ROS_ERROR_THROTTLE(1,"Node %d has Status Word Fault!",i+1);
			//printf("Node %d has Status Word Fault!\n",i+1);
			return -1;
		}
		cp++;
	}
	return 1;
}


int EposCan::ActualRPM(int which)
{
	if (which >= 0 && which < controllerCount)
	{
		//DO2012 HACK
		if (which == current_settings.newGearHack)
		{
			return (int)((18.0/23.0)*(epos+which)->actualRPM+0.5);
		}
		//END DO2012 HACK
		return (epos+which)->actualRPM;
	}
	return 0;
}


int EposCan::DemandRPM(int which)
{
	if (which >= 0 && which < controllerCount)
	{
		//DO2012 HACK
		if (which == current_settings.newGearHack)
		{
			return (int)((18.0/23.0)*(epos+which)->demandRPM+0.5);
		}
		//END DO2012 HACK
		return (epos+which)->demandRPM;
	}
	return 0;
}


int EposCan::ActualCurrent(int which)
{
	if (which >= 0 && which < controllerCount)
	{
		return (epos+which)->actualCurrent;
	}
	return 0;
}


void EposCan::SetDemandRPM(int which, int rpm)
{
	if (which >= 0 && which < controllerCount)
	{
		(epos+which)->demandRPM = rpm;
		//DO2012 HACK
		if (which == current_settings.newGearHack)
		{
			(epos+which)->demandRPM = (int)((23.0/18.0)*rpm+0.5);
		}
		//END DO2012 HACK
	}
}


void EposCan::PrintStatus()
{
	unsigned char i;
	for(i=0; i<controllerCount; i++)
	{
		printf("NODE %d\n", i+1);
		printf("\tNmt Status: %x\n",(epos+i)->nmtStatus);
		printf("\tActual RPM: %d\n",(epos+i)->actualRPM);
		printf("\tDemand RPM: %d\n",(epos+i)->demandRPM);
		printf("\tActual C  : %d mA\n",(epos+i)->actualCurrent);
		printf("\tStatus Word: %x\n",(epos+i)->statusword);
		printf("--------------\n");
	}
}


void EposCan::ProcessOdometry(unsigned char nodeid,unsigned char* data,int len)
{
	if (len!=8)
	{
		fprintf(stderr,"unexpected value length on TxPDO1 of node %d\n",nodeid);
		return;
	}
	unsigned char i = nodeid -1;
	int v = data[0]+(data[1]<<8)+(data[2]<<16)+(data[3]<<24);
	short c = data[4]+(data[5]<<8);
	short sw = data[6]+(data[7]<<8);
	(epos+i)->actualRPM = v;
	(epos+i)->actualCurrent = c;
	(epos+i)->statusword = sw;
	gonz_notify_odometry();
	//printf("V: %d rpm  C: %d mA\n",v,c);


}
void EposCan::Receive(unsigned int canid, unsigned char* data, int len)
{
	unsigned char nodeid = canid & CAN_NODE_ID_MASK;
	if (nodeid > 0 && nodeid <= controllerCount)
	{
		gettimeofday(&((epos+(nodeid-1))->lastReceivedUpdate),NULL);
	}
	switch(canid & CAN_MSG_ID_MASK)
	{
		case CAN_PDO_1_Tx:
			ProcessOdometry(nodeid,data,len);
			break;
		case CAN_ID_NMT_STATUS:
			//printf("Rcvd new Status for node %d\n",nodeid);
			ProcessNMTResponse(nodeid,data,len);
			break;
		case CAN_ID_ERROR:
			//printf("emergency message from node %d!\n",nodeid);
			HandleEposError(nodeid,(data[1]<<8)+data[0]);
			break;
		case CAN_ID_SDO_RESPONSE:
			ProcessSDOResponse(nodeid,data,len);
			break;
		default:
			fprintf(stderr,"Received unknown can msg id: %#x from node: %d\n",canid&CAN_MSG_ID_MASK,nodeid);
			/*
			for(int i =0; i<len;i++) {
			printf("%x ",data[i]);
			}
			exit(1);
			*/
	}
}


void EposCan::ProcessSDOResponse(unsigned char nodeid,unsigned char* buffer, int len)
{
	if (len < 4)
	{
		fprintf(stderr,"Malformed SDO Response from node %d\n",nodeid);
		return;
	}
	if ((buffer[0]&0xF0) != 0x40)
	{
		//fprintf(stderr,"Command specifier is not a read answer! %#x from node %d\n",buffer[0],nodeid);
		return;
	}
	unsigned short addr = buffer[1] + (buffer[2] << 8);
	switch (addr)
	{
		case 0x1001: // Error Register
			if (buffer[4] != 0x00) //there is an error
			{
				fprintf(stderr,"Node %d reported error %#x\n",nodeid,buffer[4]);
				status = failure;
				EmergencyShutdown();
				gonz_set_error_state(1,current_settings.errorRestTime);
			}
			break;
		default:
			fprintf(stderr,"Unknown address in SDO Response: %#x\n",addr);
	}
}


void EposCan::HandleEposError(unsigned char nodeid,unsigned int errorCode)
{
	switch(errorCode)
	{
		case 0x00: //no error, silently drop
			return;
		case 0x2310:
			ROS_ERROR_THROTTLE(1,"Error: Overcurrent / Motor Power Cable on node %d (code %#x)",nodeid,errorCode);
			//fprintf(stderr,"Error: Overcurrent / Motor Power Cable on node %d (code %#x)\n",nodeid,errorCode);
			break;
		case 0xFF01:
		case 0xFF04:
		case 0xFF08:
			ROS_ERROR_THROTTLE(1,"Error: Hall Sensor Cable on node %d (code %#x)",nodeid,errorCode);
			//fprintf(stderr,"Error: Hall Sensor Cable on node %d (code %#x)\n",nodeid,errorCode);
			break;
		case 0xFF0A: //Position Sensor not working
			ROS_ERROR_THROTTLE(1,"Error: Position Sensor Breach on node %d (code %#x)",nodeid,errorCode);
			//fprintf(stderr,"Error: Position Sensor Breach on node %d (code %#x)\n",nodeid,errorCode);
			break;
		case 0x8130: //Can Lifeguard Error
			ROS_ERROR_THROTTLE(1,"Error: Can Lifeguard on node %d (code %#x)",nodeid,errorCode);
			//fprintf(stderr,"Error: Can Lifeguard on node %d (code %#x)\n",nodeid,errorCode);
			break;
		case 0x8120:
			ROS_ERROR_THROTTLE(1,"Error: Can Passive Mode on node %d (code %#x)",nodeid,errorCode);
			//fprintf(stderr,"Error: Can Passive Mode on node %d (code %#x)\n",nodeid,errorCode);
			break;
		case 0x1000:
			ROS_ERROR_THROTTLE(1,"Error: Generic / Unknown on node %d (code %#x)",nodeid,errorCode);
			//fprintf(stderr,"Error: Generic / Unknown on node %d (code %#x)\n",nodeid,errorCode);
			break;
		case 0x3210:
			ROS_ERROR_THROTTLE(1,"Error: Overvoltage on node %d (code %#x)",nodeid,errorCode);
			//fprintf(stderr,"Error: Overvoltage on node %d (code %#x)\n",nodeid,errorCode);
			break;
		case 0x3220:
			ROS_ERROR_THROTTLE(1,"Error: Undervoltage on node %d (code %#x)",nodeid,errorCode);
			//fprintf(stderr,"Error: Undervoltage on node %d (code %#x)\n",nodeid,errorCode);
			break;
		case 0x4210:
			ROS_ERROR_THROTTLE(1,"Error: Overcurrent / Motor Power Cable on node %d (code %#x)",nodeid,errorCode);
			fprintf(stderr,"Error: Overtemperature on node %d (code %#x)\n",nodeid,errorCode);
			break;
		case 0x5113:
			ROS_ERROR_THROTTLE(1,"Error: Internal 5V supply too low on node %d (code %#x)",nodeid,errorCode);
			//fprintf(stderr,"Error: Internal 5V supply too low on node %d (code %#x)\n",nodeid,errorCode);
			break;
		case 0x5114:
			ROS_ERROR_THROTTLE(1,"Error: Internal supply voltage at power stage too low on node %d (code %#x)",nodeid,errorCode);
			//fprintf(stderr,"Error: Internal supply voltage at power stage too low on node %d (code %#x)\n",nodeid,errorCode);
			break;
		case 0x6100:
			ROS_ERROR_THROTTLE(1,"Error: Software failure on node %d (code %#x)",nodeid,errorCode);
			//fprintf(stderr,"Error: Software failure on node %d (code %#x)\n",nodeid,errorCode);
			break;
		case 0x6320:
			ROS_ERROR_THROTTLE(1,"Error: Parameter out of bounds on node %d (code %#x)",nodeid,errorCode);
			//fprintf(stderr,"Error: Parameter out of bounds on node %d (code %#x)\n",nodeid,errorCode);
			break;
		case 0x7320:
			ROS_ERROR_THROTTLE(1,"Error: Position Sensor Error on node %d (code %#x)",nodeid,errorCode);
			fprintf(stderr,"Error: Position Sensor Error on node %d (code %#x)",nodeid,errorCode);
			break;
		case 0x8110:
		case 0x8111:
		case 0x81FE:
		case 0x81FF:
			ROS_ERROR_THROTTLE(1,"Error: CAN Overrun (communication rate too high) on node %d (code %#x)",nodeid,errorCode);
			//fprintf(stderr,"Error: CAN Overrun (communication rate too high) on node %d (code %#x)\n",nodeid,errorCode);
			break;
		case 0x8150:
			ROS_ERROR_THROTTLE(1,"Error: CAN COB-ID collision (CAN IDs wrong?) on node %d (code %#x)",nodeid,errorCode);
			//fprintf(stderr,"Error: CAN COB-ID collision (CAN IDs wrong?) on node %d (code %#x)\n",nodeid,errorCode);
			break;
		case 0x81FD:
			ROS_ERROR_THROTTLE(1,"Error: CAN Bus Off on node %d (code %#x)",nodeid,errorCode);
			//fprintf(stderr,"Error: CAN Bus Off on node %d (code %#x)\n",nodeid,errorCode);
			break;
		case 0x8210:
			ROS_ERROR_THROTTLE(1,"Error: Received PDO too short on node %d (code %#x)",nodeid,errorCode);
			//fprintf(stderr,"Error: Received PDO too short on node %d (code %#x)\n",nodeid,errorCode);
			break;
		case 0xFF02:
			ROS_ERROR_THROTTLE(1,"Error: Position Index missing (check encoder cable) on node %d (code %#x)",nodeid,errorCode);
			//fprintf(stderr,"Error: Position Index missing (check encoder cable) on node %d (code %#x)\n",nodeid,errorCode);
			break;
		case 0xFF03:
			ROS_ERROR_THROTTLE(1,"Error: Encoder resolution wrong on node %d (code %#x)",nodeid,errorCode);
			//fprintf(stderr,"Error: Encoder resolution wrong on node %d (code %#x)\n",nodeid,errorCode);
			break;
		case 0xFF0B:
			ROS_ERROR_THROTTLE(1,"Error: System Overload on node %d (code %#x)",nodeid,errorCode);
			//fprintf(stderr,"Error: System Overload on node %d (code %#x)\n",nodeid,errorCode);
			break;
		default:
			ROS_ERROR_THROTTLE(1,"Error: Unknown %#x from node %d",nodeid,errorCode);
			//fprintf(stderr,"Error: Unknown %#x from node %d\n",errorCode,nodeid);
	}
	status = failure;
	EmergencyShutdown();
	gonz_set_error_state(1,current_settings.errorRestTime);
}


void EposCan::SendSynch()
{
	can->SendCanMsg(CAN_ID_SYNC,NULL,0);
}


int EposCan::SdoDownloadConfirmation(unsigned char nodeid, unsigned char* toconfirm)
{
	int ret;
	ret = can->WaitForCanMsg(CAN_ID_SDO_RESPONSE | nodeid,txbuf,&txbuflen,5000);
	if (!ret)
	{
		fprintf(stderr,"SDO Confirmation failed, got no answer till timeout\n");
		return 0;
	}
	if (txbuflen!=8)
	{
		//fprintf(stderr,"SDO Confirmation failed, length mismatch\n");
		return 0;
	}
	if (txbuf[0]!= CAN_CMD_DOWNLOAD_RESPONSE)
	{
		//fprintf(stderr,"SDO Confirmation failed, packet is not a response: %#x\n",txbuf[0]);
		return 0;
	}
	unsigned char i;
	for (i=1; i < 4; i++)
	{
		if (txbuf[i]!=toconfirm[i])
		{
			fprintf(stderr,"SDO Confirmation failed, adress does not match\n");
			return 0;
		}
	}
	return 1;
}


void EposCan::ProcessNMTResponse(unsigned char nodeid,unsigned char* buffer, int len)
{
	unsigned char status = 0x7F & buffer[0];
	unsigned char index = nodeid-1;
	//EposNMTStatus cur = (epos+index)->nmtStatus;
	switch (status)
	{
		case 0x00:
			//printf("Received Boot Msg from node %d\n",nodeid);
			(epos+index)->nmtStatus = booting;
			this->status = preinit;
			break;
		case 0x7F:
		    (epos+index)->nmtStatus = preoperational;
		    break;
		case 0x05:
		    (epos+index)->nmtStatus = operational;
		    break;
		case 0x04:
		    (epos+index)->nmtStatus = stopped;
		    break;
		default:
		    fprintf(stderr,"UNKNOWN NMT STATUS: %#x\n",status);
	}
	/*if (cur != (epos+index)->nmtStatus) {
		printf("NMT Status change of Node %d to %d\n",nodeid,(epos+index)->nmtStatus);
		
	}*/
}


void EposCan::SendLifeGuard()
{
	for(int i=0; i<controllerCount; i++)
	{
		can->SendCanRtr(CAN_ID_NMT_STATUS | (i+1));
	}
	lifeguardcounter=0;
}


void EposCan::SetVelocityDirect(int which, int rpm)
{
	unsigned char buf[6];
	buf[4] = 0xF; buf[5] = 0x0;
	(epos+which-1)->demandRPM = rpm;
	INT2BYTEPOS(rpm,buf,0);
	can->SendCanMsg(CAN_PDO_1_Rx|which,buf,6);
}


void EposCan::SendVelocity()
{
	unsigned char buf[6];
	buf[4] = 0xF; buf[5] = 0x0;
	for(unsigned char i=0; i<controllerCount; i++)
	{
		INT2BYTEPOS((epos+i)->demandRPM,buf,0);
		can->SendCanMsg(CAN_PDO_1_Rx|(i+1),buf,6);
	}

/*	unsigned char buf[8] = {0x22,0xFF,0x60,0x00,0x00,0x00,0x00,0x00};
	for(unsigned char i=0; i<controllerCount; i++) {
		INT2BYTEPOS((epos+i)->demandRPM,buf,4);
		can->SendCanMsg(CAN_ID_SDO_WRITE|(i+1),buf,8);
	}
	buf[0] = 0x2B; buf[1] = 0x40; buf[2] = 0x60; buf[3] = 0x00; buf[4]=0xF; buf[5]=0x0;
	for(unsigned char i=0; i<controllerCount; i++) {
		can->SendCanMsg(CAN_ID_SDO_WRITE|(i+1),buf,6);
	}
*/	
}


void EposCan::ResetData()
{
	EposStatus* e = epos;
	for(int i=0; i<controllerCount; i++)
	{
		e->nmtStatus		= unknown;
		e->statusword		= 0;
		e->actualRPM		= 0;
		e->actualCurrent	= 0;
		e->demandRPM		= 0;
		e++;
	}
}


int EposCan::IsEnabled(int node)
{
	//shoulb be: 0x0x1x11x
	char lstatus = (epos+node)->statusword & 0xFF;
	if (lstatus & 0x40) 
	{
		return 0;
	}
	return ((lstatus & 0x16) == 0x16);
}


int EposCan::HasStatusError(int node)
{
	//fault bits: 0x1000
	char lstatus = (epos+node)->statusword & 0x08;
	if (lstatus)
	{
		return 1;
	}
	return 0;
}

