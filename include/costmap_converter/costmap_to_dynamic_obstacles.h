ifndef COSTMAP_TO_DYNAMIC_OBSTACLES_H_
#define COSTMAP_TO_DYNAMIC_OBSTACLES_H_

#include <ros/ros.h>
#include <costmap_converter/costmap_converter_interface.h>


namespace costmap_converter
{
  
/**
 * @class CostmapToDynamicObstacles
 * @brief This class converts the costmap_2d into dynamic obstacles.
 */

class CostmapToDynamicObstacles : public BaseCostmapToPolygons
{
  public:
    CostmapToDynamicObstacles();
    
    ~CostmapToDynamicObstacles();
    
    virtual void initialize(ros::NodeHandle nh);
      
    virtual void compute();
      
    virtual void setCostmap2D(costmap_2d::Costmap2D* costmap);
      
    virtual void updateCostmap2D();
      
  protected:
    

   
  private:
       
   
}; 

  
} //end namespace costmap_converter

#endif /* COSTMAP_TO_DYNAMIC_OBSTACLES_H_ */

