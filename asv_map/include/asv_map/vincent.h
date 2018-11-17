#ifndef VINCENT_H
#define VINCENT_H

#include <math.h>

class Vincent
{
  public:
    Vincent(double a, double b, double oLat, double oLon);
    ~Vincent();
    void direct(double alpha_1, double s, double &lat, double &lon);
    void inverse();

  private:
    // Projection defined variables
    const double a_;      // Major semi-axis
    double b_;      // Minor semi-axis
    double f_;      // Flattening
    double U_1_;
    double c_U_1_;
    double s_U_1_;
    double t_U_1_;
    double oLat_;   // Latitude of origin
    double oLon_;   // Longitude of origin
    double k1_;

    int maxIterations_ = 100;
    int N_ = 20;
    const double RAD2DEG = 180/M_PI;
    const double DEG2RAD = M_PI/180;

    // Point defined variables

};


#endif  // VINCENT_H
