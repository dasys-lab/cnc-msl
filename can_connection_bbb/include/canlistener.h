#ifndef CANLISTENER_H
#define CANLISTENER_H 1

class CanListener
{
  public:
    virtual void receive(unsigned int canid, unsigned char *data, int len) = 0;
};

#endif
