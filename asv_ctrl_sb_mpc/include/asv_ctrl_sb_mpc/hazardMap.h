#ifndef HAZARDMAP_H
#define HAZARDMAP_H

#include <string>
#include <Eigen/Dense>
#include <gdal/ogrsf_frmts.h>

class HazardMap
{
  public:
    HazardMap(double T, double DT);
    ~HazardMap();

    void createLayer(GDALDataset *&ds, OGRLayer *&lyr, std::string ds_name, OGRSpatialReference *srs);
    void updateMap(double x, double y);
    OGRSpatialReference* getSRS();
    double ag_cost(OGRLineString *traj_safe, OGRLineString *traj_close, OGRLineString *traj_ahead);
  private:
    double T_;
    double DT_;
    int n_samp;

    std::string dir_;

    GDALDataset *ds_src_;
    GDALDataset *ds_clip_;
    GDALDataset *ds_area_;

    GDALDriver *drv_;
    OGRSpatialReference *srs_;
    OGRLayer *src_;
    OGRFeatureDefn *featDefn_;
    OGRLayer *clip_;
    OGRLayer *area_;

    OGRPoint pos_;

};


#endif  // HAZARDMAP_H
