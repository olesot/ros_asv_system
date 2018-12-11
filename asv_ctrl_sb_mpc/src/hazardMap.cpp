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

HazardMap::HazardMap(double T, double DT)
{
  T_ = T;
  DT_ = DT;
  n_samp = floor(T_/DT_);

  dir_ = ros::package::getPath("asv_ctrl_sb_mpc");
  dir_.append("/config/maps/shp/");


  GDALAllRegister();
//  GDALDataset *ds;
  ds_src_ = (GDALDataset*) GDALOpenEx((dir_+"hazards").c_str(), GDAL_OF_VECTOR, NULL, NULL, NULL);
  if(ds_src_ == NULL)
  {
    ROS_ERROR("Open source Map, %s, failed.", (dir_+"hazard").c_str());
    ros::shutdown();
  }

  src_ = ds_src_->GetLayer(0);
  srs_ = src_->GetSpatialRef();

  drv_ = GetGDALDriverManager()->GetDriverByName("ESRI Shapefile");
  if(drv_ == NULL)
  {
    ROS_ERROR("Error open GDAL Driver.");
    ros::shutdown();
  }
  createLayer(ds_clip_, clip_, "clip", srs_);
  createLayer(ds_area_, area_, "area", srs_);

  ROS_INFO("After createlayer: %s", clip_->GetName());
/*
  OGRLineString *testLine;
  testLine = new OGRLineString();
  testLine->assignSpatialReference(srs_);
  testLine->addPoint(5.8222624, 58.4732253);
  testLine->addPoint(5.8223, 58.475);

  double test = ag_cost(testLine);

  delete testLine;
  ROS_INFO("MAP Initialized");
*/
}

HazardMap::~HazardMap()
{

}


void HazardMap::updateMap(double x, double y)
{

  // Update position
  pos_.setX(X2LON*x + XORIGIN);
  pos_.setY(Y2LAT*y + YORIGIN);
  ROS_INFO("New POS: [%.5f, %.5f]", pos_.getX(), pos_.getY());
  // Create clip section
  OGRGeometry *area;
  area = pos_.Buffer(0.01,4);
  OGRFeatureDefn *featDefn;
  featDefn = src_->GetLayerDefn();
  OGRFeature *feat;
  feat = OGRFeature::CreateFeature(featDefn);
  feat->SetGeometryDirectly(area);

  // Delete old features in clip and area
  OGRFeature *nxtFeat;
  clip_->ResetReading();
  while((nxtFeat = clip_->GetNextFeature()) != NULL)
  {
    if(clip_->DeleteFeature(nxtFeat->GetFID()) != OGRERR_NONE)
      ROS_ERROR("Error deleting clip feature!");
  }
  area_->ResetReading();
  while((nxtFeat = area_->GetNextFeature()) != NULL)
  {
    if(area_->DeleteFeature(nxtFeat->GetFID()) != OGRERR_NONE)
      ROS_ERROR("Error deleting area feature");
  }

  if(clip_->CreateFeature(feat) != OGRERR_NONE)
  {
    ROS_ERROR("Something went wrong updating the clip layer");
  }

  if(src_->Intersection(clip_, area_, NULL, NULL, NULL) != OGRERR_NONE)
    ROS_ERROR("Error creating area layer");
  // Delete old area and update new


}

void HazardMap::createLayer(GDALDataset *&ds, OGRLayer *&lyr, std::string ds_name, OGRSpatialReference *srs)
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

//  GDALDataset *ds;
  ds = drv_->Create(src_path.c_str(), 0, 0, 0, GDT_Unknown, NULL);
  if(ds == NULL)
  {
    ROS_ERROR("Error creating dataset");
  }
  lyr = ds->CreateLayer(ds_name.c_str(), srs, wkbPolygon, NULL);
  ROS_INFO("Before Createlayer: %s",lyr->GetName());
}

OGRSpatialReference* HazardMap::getSRS()
{
  return srs_;
}

double HazardMap::ag_cost(OGRLineString *traj_safe, OGRLineString *traj_close, OGRLineString *traj_ahead)
{
  // Trajectory: predicted over 300 seconds
  // intersection within 0 - 10 sec should be safe
  // Intersection within 10 - 30 sec considered  close
  // Intersection within 30 - 300? sec considered ahead

  // Split trajectory into three trajectories
  OGRFeature *feat;
  double cost = 0;
  area_->ResetReading();
  while((feat = area_->GetNextFeature()) != NULL)
  {
    OGRGeometry *geom;
    geom = feat->GetGeometryRef();

    if(geom->Intersects(traj_safe))
      return 100.0;

    if(geom->Intersects(traj_close))
      cost = 50;

    if(cost == 0)
    {
      if(geom->Intersects(traj_ahead))
        cost = 10;
    }
  }

  return cost;
}
