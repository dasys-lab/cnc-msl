#include "CanHandler.h"

#include <msl_actuator_msgs/CanMsg.h>

namespace msl_bbb
{

CanHandler::CanHandler()
{
    usbCanConnection = new UsbCanConnection("can0");
    usbCanConnection->SetReceiver(this);

    // add id's for can devices
    receivers.push_back(Compass);
    receivers.push_back(ReKick);
    receivers.push_back(BallHandler);
}

CanHandler::~CanHandler()
{
    usbCanConnection->Stop();
}

void CanHandler::sendCanMsg(const msl_actuator_msgs::CanMsg &msg)
{
    std::cout << "CanHandler: Write data to CAN!" << std::endl;

    unsigned char *p = (unsigned char *)msg.data.data();
    usbCanConnection->SendExCanMsg(msg.id, p, msg.data.size());
}

void CanHandler::resetInterface()
{
    usbCanConnection->Stop();

    int e = system("sudo ifdown can0");
    sleep(1);
    usbCanConnection = new UsbCanConnection("can0");
    usbCanConnection->SetReceiver(this);
    e = system("sudo ifup can0");
    sleep(1);
    if (!e)
    {
        std::cerr << "error reseting!" << std::endl;
    }

    usbCanConnection->Start();
}

void CanHandler::receive(unsigned int canid, unsigned char *data, int len)
{
    bool found = false;
    unsigned int id = ((canid & 0xFF00) >> 8);
    for (unsigned int i = 0; i < receivers.size(); i++)
    {
        if (id == receivers[i])
        {
            found = true;

            msl_actuator_msgs::CanMsg cm;
            cm.id = id;

            for (int i = 0; i < len; i++)
            {
                cm.data.push_back(data[i]);
            }

            switch (id)
            {
            case Compass:
                break;
            case ReKick:
                std::cout << "CanHandler: sending rekick can msg" << std::endl;
                //onRosCanMsg418700403(cm);
                break;
            case BallHandler:
                break;
            default:
                break;
            }
        }
    }

    if (!found)
    {
        std::cerr << "CanHandler: Unkown CAN ID received: " << canid << std::endl;
    }
}

//void CanHandler::receive(unsigned int canid, unsigned char *data, int len)
//{
//    int found = 0;
//    unsigned int id = ((canid & 0xFF00) >> 8);
//    for (unsigned int i = 0; i < receivers.size(); i++)
//    {
//        // printf("receiver %u\n",receivers[i]);
//        // printf("id is : %u \n",id);
//        if (id == receivers[i])
//        {
//            found = 1;
//
//            CanMsg cm;
//            // 				cm.header = 0x0;
//            // 				cm.priority = (canid>>16) & 0xFF; //Priority
//            // 				cm.sender = (canid>>8) & 0xFF; //Sender
//            // 				cm.receiver = (canid)  & 0xFF;  //receiver
//            // 				cm.length = (len<<1);
//            cm.id = id;
//            for (int i = 0; i < len; i++)
//            {
//                cm.data.push_back(data[i]);
//            }
//
//            if (id == Compass)
//            {
//                compass.publish(cm);
//                // printf("get compass val from canbus!\n");
//            }
//            else if (id == ReKick)
//            {
//                rekick.publish(cm);
//                /*printf("get rekick val from canbus!\n");
//                for(unsigned int i=0; i<cm.data.size(); i++)
//                {
//                        printf("%u\n",cm.data[i]);
//                }*/
//            }
//            else if (id == BallHandler)
//            {
//                ballhandler.publish(cm);
//                // printf("get actuator val from canbus!\n");
//            }
//            else
//            {
//                fprintf(stderr, "Unkown canid received: 0x%x\n", canid);
//            }
//            break;
//        }
//    }
//    if (found <= 0)
//    {
//        fprintf(stderr, "Unkown canid received: 0x%x\n", canid);
//    }
//}
}
