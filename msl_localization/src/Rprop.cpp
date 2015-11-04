#include "Rprop.h"


Rprop::Rprop() {
  gamma = 0.1;
  oldErrGrad = 0;
  gammaMax = 50.0;
  gammaMin = 0.000001;
  etap = 1.2;
  etam = 0.5;
}

Rprop::Rprop(double _gamma) {
  gamma = _gamma;
  oldErrGrad = 0;
  gammaMax = 50.0;
  gammaMin = 0.000001;
  etap = 1.2;
  etam = 0.5;
}


Rprop::Rprop(double _gammaMax, double _gammaMin) {
  gamma = 0.1;
  oldErrGrad = 0;
  gammaMax = _gammaMax;
  gammaMin = _gammaMin;
  etap = 1.2;
  etam = 0.5;
}


Rprop::Rprop(double _gammaMax, double _gammaMin, double _etap, double _etam) {
  gamma = 0.1;
  oldErrGrad = 0;
  gammaMax = _gammaMax;
  gammaMin = _gammaMin;
  etap = _etap;
  etam = _etam;
}

Rprop::~Rprop() {
}
  
double Rprop::getdW(double errorGrad) {
  if(errorGrad == 0) return 0;
  if(errorGrad*oldErrGrad>0) gamma = min(gamma*etap, gammaMax);
  else if(errorGrad*oldErrGrad<0) gamma = max(gamma*etam, gammaMin);
  
  oldErrGrad = errorGrad;
  
  return gamma * sign(errorGrad);
}

