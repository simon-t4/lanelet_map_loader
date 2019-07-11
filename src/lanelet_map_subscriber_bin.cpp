#include "ros/ros.h"
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>


#include "std_msgs/String.h"
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_io/io_handlers/Factory.h>
#include <lanelet2_io/io_handlers/Parser.h>
#include <lanelet2_io/io_handlers/OsmHandler.h>
#include <lanelet2_io/io_handlers/OsmFile.h>

#include <lanelet2_projection/UTM.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/LineStringOrPolygon.h>

#include <lanelet2_traffic_rules/TrafficRules.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>



#include <lanelet2_routing/Route.h>
#include <lanelet2_routing/RoutingCost.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_routing/RoutingGraphContainer.h>

#include <lanelet2_core/geometry/BoundingBox.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/Point.h>

#include <lanelet2_core/primitives/Lanelet.h>
#include <Eigen/Eigen>

//#include <autoware_msgs/Signals.h>

#include <cstdio>

#include <sstream>
//#include "rosUTM.h"
//#include "libLaneletMap.h"
//#include <lanelet_msgs/PointArray.h>
//#include <lanelet_msgs/MapXML.h>
//#include <pugixml.hpp>


#include <lanelet2_extension/io/message_conversion.hpp>

#include <lanelet2_extension/projection/mgrs_projector.hpp>
#include <lanelet2_msgs/MapBin.h>

#include <lanelet2_extension/regulatory_elements/autoware_traffic_light.hpp>

#include <unistd.h>
#include <ios>
#include <iostream>
#include <fstream>
#include <string>

//std::string example_map_path = "/home/simon/work/catkin_ws/src/lanelet2/lanelet2_maps/res/mapping_example.osm";
std::string example_map_path = "/home/simon/work/peoria_data/map/Lanelet2/Peoria_0301_2019_fixed.osm";




//-------------------------------------------------------------------------
//
//
//
//-------------------------------------------------------------------------





lanelet::LaneletMapPtr lanelet_map;
//-------------------------------------------------------------------------
//
//
//
//-------------------------------------------------------------------------

void binMapCallback(lanelet2_msgs::MapBin msg)
{
 lanelet_utils::Map::fromBinMsg(msg, lanelet_map);
 for ( auto lanelet: lanelet_map->laneletLayer )
   {
     //You can access to traffic light element as AutowareTrafficLight class
     auto autoware_traffic_lights = lanelet.regulatoryElementsAs<lanelet::autoware::AutowareTrafficLight>();
     if(autoware_traffic_lights.empty()) continue;
     for (auto light: autoware_traffic_lights)
       {
	 std::cout << "light_bulbs: ";
	 for(auto light_string: light->lightBulbs() )
	   {
	     std::cout << light_string.id() << " ";
	   }
	 std::cout << std::endl;
       }
     //You can also access to traffic light element as default TrafficLight class
     auto traffic_lights = lanelet.regulatoryElementsAs<lanelet::TrafficLight>();
     for (auto light: traffic_lights)
       {
	 std::cout << "traffic lights: ";
	 for(auto light_string: light->trafficLights() )
	   {
	     std::cout << light_string.id() << " ";
	   }
	 std::cout << std::endl;
       }
     std::cout << std::endl;
   }
 
}
//-------------------------------------------------------------------------
//
//
//
//-------------------------------------------------------------------------

int main (int argc, char **argv)
{

  
  ros::init(argc, argv, "lanelet_map_subscriber");  
  ros::NodeHandle rosnode;
  
  ros::Subscriber xml_map_sub = rosnode.subscribe("/lanelet_map_bin", 10000,  binMapCallback);

  
    
  ros::spinOnce();
  ros::Rate loop_rate(10);
  int loop_count = 0;
  
  while(ros::ok()) // && loop_count < 10)
    {
      
      ros::spinOnce();
      loop_count++;
    }
  
  return 0;
}
