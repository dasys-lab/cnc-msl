/*
 * CaceMultiCastThread.h
 *
 *  Created on: 23.06.2014
 *      Author: endy
 */

#ifndef MULTICASTTHREAD_H_
#define MULTICASTTHREAD_H_

#include "PracticalSocket.h"
#include <string>
#include <thread>

using namespace std;

//#define MAX_PACKETSIZE 8192
#define MAX_PACKETSIZE 100000

namespace multicast
{
template <class CommunicationClass> using t_multicastcallback = void (CommunicationClass::*)(char *, int);
// typedef void (*t_multicastcallback)(char*, int size);

template <class CommunicationClass> class MultiCastChannel
{
  public:
    static long traffic;
    MultiCastChannel(string address, unsigned short port, t_multicastcallback<CommunicationClass> callback, CommunicationClass *obj);
    ~MultiCastChannel();
    void publish(const char *bytes, int size);

  protected:
    static unsigned short sourcePort;
    bool running;
    std::thread *t;
    string address;
    unsigned short port;
    UDPSocket udpsocket;
    void call();
    char *recvArray;
    t_multicastcallback<CommunicationClass> callback;
    CommunicationClass *obj;
};

template <class CommunicationClass> unsigned short MultiCastChannel<CommunicationClass>::sourcePort = 30000;
template <class CommunicationClass> long MultiCastChannel<CommunicationClass>::traffic = 0;

template <class CommunicationClass>
inline MultiCastChannel<CommunicationClass>::MultiCastChannel(string address, unsigned short port, t_multicastcallback<CommunicationClass> callback,
                                                              CommunicationClass *obj)
    : udpsocket(port)
{
    running = true;

    this->obj = obj;
    this->callback = callback;
    this->address = address;
    this->port = port;

    recvArray = new char[MAX_PACKETSIZE];

    udpsocket.setMulticastTTL(1);

    udpsocket.joinGroup(address);
    t = new std::thread(&MultiCastChannel<CommunicationClass>::call, this);
}

template <class CommunicationClass> inline MultiCastChannel<CommunicationClass>::~MultiCastChannel()
{
    running = false;
    // udpsocket.leaveGroup(address);
    udpsocket.disconnect();
    t->join();
    // udpsocket.disconnect();
    delete t;
    delete[] recvArray;
}

template <class CommunicationClass> inline void MultiCastChannel<CommunicationClass>::publish(const char *bytes, int size)
{
    udpsocket.sendTo(bytes, size, address, port);
    traffic += size;
}

template <class CommunicationClass> void MultiCastChannel<CommunicationClass>::call()
{

    int numBytes = 0;
    while (true)
    {
        numBytes = udpsocket.recvFrom(recvArray, MAX_PACKETSIZE, address, port);
        if (!running)
            return;
        (obj->*callback)(recvArray, numBytes);
    }
}

} /* namespace cace */

#endif /* CACEMULTICASTTHREAD_H_ */
