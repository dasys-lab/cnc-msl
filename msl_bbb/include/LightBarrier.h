#pragma once

#include "Communication.h"
#include "Worker.h"

#include <BlackADC.h>
#include <BlackDef.h>

namespace BlackLib
{
class BlackADC;
}

namespace msl_bbb
{
class LightBarrier : public Worker
{

  public:
    LightBarrier(BlackLib::adcName adc_P, Communication *comm);
    ~LightBarrier();

    void run(); /** < overwrites the workers virtual run method */

    bool checkLightBarrier();
    bool setTreshold(int th);

  private:
    Communication *comm;
    BlackLib::BlackADC *adc;
    int threshold;
};
}
