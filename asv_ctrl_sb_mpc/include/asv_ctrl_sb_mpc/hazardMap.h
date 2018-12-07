#ifndef HAZARDMAP_H
#define HAZARDMAP_H

#include <string>

#include <gdal/ogrsf_frmts.h>

class HazardMap
{
  public:
    HazardMap();
    ~HazardMap();

    void createLayer(OGRLayer *lyr, std::string ds_name, OGRSpatialReference *srs);
    void updateCloseArea(double x, double y);

  private:
    std::string dir_;

    GDALDriver *drv_;
    OGRSpatialReference *srs_;
    OGRLayer *src_;
    OGRFeatureDefn *featDefn_;
    OGRLayer *clip_;
    OGRLayer *area_;

    OGRPoint pos_;

};


#endif  // HAZARDMAP_H
