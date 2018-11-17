
#include "asv_map/vincent.h"

#include <ros/ros.h>

#include <stdio.h>

Vincent::Vincent(double a, double b, double oLat, double oLon):
  a_(a),
  b_(b),
  oLat_(oLat),
  oLon_(oLon)
{
  f_ = (a_-b_)/a_;
  U_1_ = atan((1-f_)*tan(DEG2RAD*oLat_));
  c_U_1_ = cos(U_1_);
  s_U_1_ = sin(U_1_);
  t_U_1_ = tan(U_1_);
  k1_ = (a_*a_ - b_*b_)/(b_*b_);
  ROS_INFO("a:\t%f", a_);
  ROS_INFO("b:\t%f", b_);
  ROS_INFO("oLat:\t%f", oLat_);
  ROS_INFO("oLon:\t%f", oLon_);
  ROS_INFO("f:\t%f", f_);
  ROS_INFO("U1:\t%f", U_1_);
  ROS_INFO("sU1:\t%f", s_U_1_);
  ROS_INFO("cU1:\t%f", c_U_1_);
  ROS_INFO("tU1:\t%f", t_U_1_);
  ROS_INFO("k1:\t%f", k1_);
}

Vincent::~Vincent()
{

}


void Vincent::direct(double alpha_1, double s, double &lat, double &lon)
{
  const double c_alpha_1 = cos(alpha_1);
  const double s_alpha_1 = sin(alpha_1);
  const double sigma_1 = atan2(t_U_1_, c_alpha_1);      // 2
  const double s_alpha = c_U_1_*s_alpha_1;              // 3
  const double ss_alpha = s_alpha*s_alpha;
  const double cc_alpha = 1.0 - ss_alpha;               // 4
  const double uu = cc_alpha*k1_;                       // 5

  const double A = 1 + (uu/16384.0)*(4096.0 + uu*(-768.0+uu*(320.0-175.0*uu))); // 6
  const double B = (uu/1024.0)*(256.0 + uu*(-128.0 + uu*(74.0 - 47.0*uu)));     // 7

  int cnt = 0;
  const double sigma_init = s/(b_*A);
  double sigma = sigma_init;

  double sigma_m;
  double c_sigma_m;
  double cc_sigma_m;
  double ss_sigma_m;

//  ROS_INFO("Begin Iterations with init sigma %f.", sigma);

  while(cnt < maxIterations_)
  {
    cnt++;
    sigma_m = 2*sigma_1 + sigma;          // 8
    c_sigma_m = cos(sigma_m);
    cc_sigma_m = c_sigma_m*c_sigma_m;
    ss_sigma_m = sin(sigma_m)*sin(sigma_m);

    double delta_sigma = B*sin(sigma)*(c_sigma_m + 0.25*B*(cos(sigma)*(-1+2*cc_sigma_m) - (B/6)*c_sigma_m*(-3 + 4*ss_sigma_m)*(-3+4*cc_sigma_m)));    // 9

    double sigma_new = sigma_init + delta_sigma;  // 10
    if(std::abs(sigma_new - sigma) < std::pow(10, -N_))
    {
      sigma = sigma_new;
      break;
    }
    sigma = sigma_new;
  }

//  ROS_INFO("Found sigma %f after %d iterations.", sigma, cnt);

  const double s_sigma = sin(sigma);
  const double c_sigma = cos(sigma);

  double lat_num = s_U_1_*c_sigma + c_U_1_*s_sigma*c_alpha_1;
  double lat_pow = std::pow((s_U_1_*s_sigma - c_U_1_*c_sigma*c_alpha_1),2);
  double lat_denum = (1 - f_)*sqrt(ss_alpha + lat_pow);
  lat = RAD2DEG*atan(lat_num/lat_denum);    // 11
  ROS_INFO("Vincent LAT: %f", RAD2DEG*atan(lat_num/lat_denum));
  double lambda = atan((s_sigma*s_alpha_1)/(c_U_1_*c_sigma - s_U_1_*s_sigma*c_alpha_1));    //12
  double C = f_/16*cc_alpha*(4+f_*(4-3*cc_alpha));    //13
  double L = lambda - (1-C)*f_*s_alpha*(sigma + C*s_sigma*(c_sigma_m + C*c_sigma*(-1+2*cc_sigma_m))); //14
  ROS_INFO("Vincent LON: %f", RAD2DEG*L + oLon_);
  lon = RAD2DEG*L + oLon_;    //15

//  double alpha_2 = atan(s_alpha/(-s_U_1_*s_sigma + c_U_1_*c_sigma*c_alpha_1)); ??
/*  ROS_INFO("Origin:");
  ROS_INFO("Lat: %f, Lon %f", oLat_, oLon_);
  ROS_INFO("New Point:");
  ROS_INFO("Lat: %f, Lon: %f", lat, lon);*/
}
