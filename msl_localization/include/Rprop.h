#pragma once

class Rprop {

public:
  Rprop();
  Rprop(double _gamma);
  Rprop(double _gammaMax, double _gammaMin);
  Rprop(double _gammaMax, double _gammaMin, double _etap, double _etam);
  ~Rprop();
  
  double getdW(double errorGrad);
  
  void setGamma(double _gamma) {gamma = _gamma;}
  
private:

  double min(double a, double b) {return a<b ? a:b;}
  double max(double a, double b) {return a>b ? a:b;}
  double sign(double a) {return a>=0 ? 1:-1;}

  double oldErrGrad;
  double gamma;
  
  double gammaMax;
  double gammaMin;
  
  double etap;
  double etam;
  
};

