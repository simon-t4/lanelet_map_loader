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
#include <lanelet2_io/io_handlers/Writer.h>
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

#include <autoware_msgs/Signals.h>

#include <cstdio>

#include <sstream>
#include "rosUTM.h"

#include "mgrs_projector.hpp"

#include "libLaneletMap.h"

#include <lanelet_msgs/PointArray.h>
#include <lanelet_msgs/MapXML.h>
#include <pugixml.hpp>

#include <unistd.h>
#include <ios>
#include <iostream>
#include <fstream>
#include <string>
//using namespace lanelet;

#define TL_BY_LANELET 0
#define TL_BY_NEAREST 1

#define signalLampRadius 0.3

static std::string camera_id_str;


//std::string example_map_path = "/home/simon/work/catkin_ws/src/lanelet2/lanelet2_maps/res/mapping_example.osm";
std::string example_map_path = "/home/simon/work/peoria_data/map/Lanelet2/Peoria_0301_2019_fixed_edit.osm";


//-------------------------------------------------------------------------
//
//
//
//-------------------------------------------------------------------------

int main (int argc, char **argv)
{

  // UTM 1KM BLOCK
  lanelet::Origin origin({0,0});

  // origin not used in projector but projector template requires one
  lanelet::projection::RosUtmProjector ros_projector(origin);
  lanelet::ErrorMessages errors;
  lanelet::LaneletMapPtr map = load(example_map_path, ros_projector, &errors);
 
  ros::init(argc, argv, "lanelet_map_loader");  
  ros::NodeHandle rosnode;
  
  // publisher to visualise lanelet elements within rviz
  //  ros::Publisher map_xml_pub = rosnode.advertise<lanelet_msgs::MapXML>("/lanelet_map_xml", 1, true);
  ros::Publisher map_bin_pub = rosnode.advertise<lanelet_msgs::MapBin>("/lanelet_map_bin", 1, true);

  //  lanelet_msgs::MapXML map_xml_msg;
  lanelet_msgs::MapBin map_bin_msg;
  //  int status = lanelet_utils::Map::toXMLMsg(map, map_xml_msg, ros_projector);

  lanelet_utils::Map::toBinMsg(map, map_bin_msg);
  

  ros::spinOnce();
  // main loop

  ros::Rate loop_rate(10);  
  int loop_count = 0;
  
  //map_xml_pub.publish(map_xml_msg);
  map_bin_pub.publish(map_bin_msg);

  while(ros::ok()) // && loop_count < 10)
    {
 
      ros::spinOnce();
      loop_count++;
      loop_rate.sleep();
    }
  
  return 0;
}
