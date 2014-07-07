#ifndef PROXEMIC_LAYER_H_
#define PROXEMIC_LAYER_H_
#include <ros/ros.h>
#include <social_navigation_layers/social_layer.h>
#include <dynamic_reconfigure/server.h>
#include <social_navigation_layers/ProxemicLayerConfig.h>

double gaussian(double x, double y, double x0, double y0, double A, double varx, double vary, double skew);
double get_radius(double cutoff, double A, double var);

namespace social_navigation_layers
{
  class ProxemicLayer : public SocialLayer
  {
    public:
      ProxemicLayer() { layered_costmap_ = NULL; }

      virtual void onInitialize();
      virtual void updateBoundsFromPeople(double* min_x, double* min_y, double* max_x, double* max_y);
      virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

    protected:
      void configure(ProxemicLayerConfig &config, uint32_t level);
      double cutoff_, amplitude_, covar_, factor_;
      dynamic_reconfigure::Server<ProxemicLayerConfig>* server_;
      dynamic_reconfigure::Server<ProxemicLayerConfig>::CallbackType f_;
  };
};


#endif

