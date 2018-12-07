#include <ros/ros.h>
#include <ros/package.h>

#include <fstream>

#include "asv_ctrl_sb_mpc/hazardMap.h"


static const double DEG2RAD = M_PI/180.0f;
static const double RAD2DEG = M_PI/180.0f;

static const double Y2LAT = 1/111386.1;
static const double X2LON = 1/58347.8;
static const double XORIGIN = 5.84;
static const double YORIGIN = 58.475;

HazardMap::HazardMap()
{


  dir_ = ros::package::getPath("asv_ctrl_sb_mpc");
  dir_.append("/config/maps/shp/");


  GDALAllRegister();
  GDALDataset *ds;
  ds = (GDALDataset*) GDALOpenEx((dir_+"hazards").c_str(), GDAL_OF_VECTOR, NULL, NULL, NULL);
  if(ds == NULL)
  {
    ROS_ERROR("Open source Map, %s, failed.", (dir_+"hazard").c_str());
    ros::shutdown();
  }
  /*
  src_ = ds->GetLayer(0);
  srs_ = src_->GetSpatialRef();

  drv_ = GetGDALDriverManager()->GetDriverByName("ESRI Shapefile");
  if(drv_ == NULL)
  {
    ROS_ERROR("Error open GDAL Driver.");
    ros::shutdown();
  }
//  createLayer(clip_, "clip", srs_);
//  createLayer(area_, "area", srs_);

//  updateCloseArea(5.84,58.475);
//  updateCloseArea(5.85,58.5);
*/
  ROS_INFO("MAP Initialized");

}

HazardMap::~HazardMap()
{

}


void HazardMap::updateCloseArea(double x, double y)
{
  ROS_INFO("HERE");
  pos_.setX(x);
  pos_.setY(y);

/*
  int i = clip_->GetFeatureCount();

  {
    if(clip_->DeleteFeature(0) != OGRERR_NONE)
    {
      ROS_ERROR("Error deleting feature");
    }
  }


  OGRFeature *feat;
  feat = OGRFeature::CreateFeature(clip_->GetLayerDefn());
  OGRGeometry *geom;
  geom = pos_.Buffer(0.01, 4);
  feat->SetGeometry(geom);
  feat->SetFID(0);

  if( clip_->CreateFeature(feat) != OGRERR_NONE)
  {
    ROS_ERROR("Error creating feature");
  }
*/
}

void HazardMap::createLayer(OGRLayer *lyr, std::string ds_name, OGRSpatialReference *srs)
{
  std::string src_path;
  src_path.append(dir_);
  src_path.append(ds_name);
  std::ifstream ifile(src_path.c_str());

  if((bool)ifile)
  {
    ROS_INFO("DATASET EXISTS");
    drv_->Delete(src_path.c_str());
  }

  GDALDataset *ds;
  ds = drv_->Create(src_path.c_str(), 0, 0, 0, GDT_Unknown, NULL);
  if(ds == NULL)
  {
    ROS_ERROR("Error creating dataset");
  }
  lyr = ds->CreateLayer(ds_name.c_str(), srs, wkbPolygon, NULL);
}
