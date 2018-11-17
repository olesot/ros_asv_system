#include "asv_map/asv_map_service.h"

#include <ros/package.h>
#include <math.h>

MapService::MapService(ros::NodeHandle nh, double a, double b, double oLat, double oLon) :
  nh_(nh),
  Vincent(a, b, oLat, oLon)
{
  path_ = ros::package::getPath("asv_map_service");
  path_.append("/config/maps/polyHazards.shp");

  GDALAllRegister();
  GDALDataset *ds;
  ds = (GDALDataset*) GDALOpenEx(path_.c_str(), GDAL_OF_VECTOR, NULL, NULL, NULL);
  if( ds == NULL)
  {
    ROS_ERROR("Open map failed.");
    ros::shutdown();
  }

  layer_ = ds->GetLayer(0);
  if(layer_->GetExtent(0, &extnt_) == OGRERR_FAILURE)
  {
    ROS_ERROR("Error getting extent.");
    ros::shutdown();
  }
  spRef_ = layer_->GetSpatialRef();
  point_ = new OGRPoint(0,0);
  point_->assignSpatialReference(spRef_);

  service_ = nh_.advertiseService("intersect_map", &MapService::intersects, this);

}

MapService::~MapService()
{
  delete point_;
}

bool MapService::intersects(asv_msgs::Intersect::Request &req,
                       asv_msgs::Intersect::Response &res)
{
  double x,y;
  x = req.pos.x;
  y = req.pos.y;
  double distance = sqrt(x*x + y*y);
  // ENU TO NED swap x y
  double azimuth = atan2(x, y);
  ROS_INFO("dist: %f, az: %f", distance, azimuth);
  double lat,lon;
  direct(azimuth, distance, lat, lon);
  ROS_INFO("Lat: %.9f, Lon: %.9f", lat, lon);
  point_->setX(lon);
  point_->setY(lat);

  OGRFeature *feat;
  layer_->ResetReading();
  while((feat = layer_->GetNextFeature()) != NULL)
  {
    OGRGeometry *geom = feat->GetGeometryRef();
    if(point_->Intersects(geom))
    {
  //    ROS_INFO("InterSects at (%f, %f)!", point_->getX(), point_->getY());
      return true;
    }
  }
//  ROS_INFO("Do not intersect at (%f, %f)!", point_->getX(), point_->getY());
  return false;
}
