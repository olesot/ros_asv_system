#ifndef MAP_SERVICE_H
#define MAP_SERVICE_H

#include <ros/ros.h>
#include <gdal/ogrsf_frmts.h>
#include <string>

#include "asv_msgs/Intersect.h"
#include "asv_map/vincent.h"



class MapService : public Vincent
{
  public:
    explicit MapService(ros::NodeHandle nh,
                        double a,
                        double b,
                        double oLat,
                        double oLon);
    ~MapService();
    bool intersects(asv_msgs::Intersect::Request  &req,
                    asv_msgs::Intersect::Response &res);
    void updatePoint(asv_msgs::Intersect::Request &req);

  private:
    ros::NodeHandle nh_;
    ros::ServiceServer service_;

    std::string path_;
    OGRLayer *layer_;
    OGRFeature *feat_;
    OGREnvelope extnt_;
    OGRSpatialReference *spRef_;
    OGRPoint *point_;
};


#endif  // MAP_SERVICE_H

