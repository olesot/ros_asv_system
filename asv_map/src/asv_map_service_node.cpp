#include <ros/ros.h>

#include "asv_map/asv_map_service.h"

#define WGS_A 6378137.0
#define WGS_B 6356752.314245

// REPLACE THIS TO CONFIG
#define ORIGIN_LAT 58.475
#define ORIGIN_LON 5.84

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "asv_map_service_node");
  ros::NodeHandle nh;
  MapService mapService(nh,
                        WGS_A,
                        WGS_B,
                        ORIGIN_LAT,
                        ORIGIN_LON);
  ros::spin();
  return 0;
}


