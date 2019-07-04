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

#include <autoware_msgs/Signals.h>

#include <cstdio>

#include <sstream>
#include "rosUTM.h"
#include "libLaneletMap.h"
#include <lanelet_msgs/PointArray.h>
#include <lanelet_msgs/MapXML.h>
#include <lanelet_msgs/MapBin.h>
#include <pugixml.hpp>





#include <unistd.h>
#include <ios>
#include <iostream>
#include <fstream>
#include <string>

//std::string example_map_path = "/home/simon/work/catkin_ws/src/lanelet2/lanelet2_maps/res/mapping_example.osm";
std::string example_map_path = "/home/simon/work/peoria_data/map/Lanelet2/Peoria_0301_2019_fixed_edit.osm";




//-------------------------------------------------------------------------
//
//
//
//-------------------------------------------------------------------------



static bool loaded_lanelet_map;
static lanelet::LaneletMapPtr viz_lanelet_map;


static ros::Publisher road_border_pub;


//-------------------------------------------------------------------------
//
//
//
//-------------------------------------------------------------------------

void xmlMapCallback(lanelet_msgs::MapXML msg)
{
  std::cerr << "recived map\n";
  int status = lanelet_utils::Map::fromXMLMsg(msg, viz_lanelet_map);
  loaded_lanelet_map = true;
}
void binMapCallback(lanelet_msgs::MapBin msg)
{
  std::cerr << "recived map\n";
  lanelet_utils::Map::fromBinMsg(msg, viz_lanelet_map);
  loaded_lanelet_map = true;
}
//-------------------------------------------------------------------------
//
//
//
//-------------------------------------------------------------------------

int main (int argc, char **argv)
{

  loaded_lanelet_map = false;


  bool load_from_file =false;

  std::cerr << "map_visualizaer: load from file = " << load_from_file << "\n";
    ros::init(argc, argv, "lanelet_map_vsiualizer");  
  ros::NodeHandle rosnode;
  ros::Subscriber xml_map_sub;
  ros::Subscriber bin_map_sub;
   
  if (load_from_file==true) {
    lanelet::Origin origin({49, 8.4});
    
    // origin not used in projector but projector template requires one
    lanelet::projection::RosUtmProjector ros_projector(origin);
    
    lanelet::ErrorMessages errors;
    
    
    double vm, rss, vm1, rss1;
    
    viz_lanelet_map = load(example_map_path, ros_projector, &errors);
    
    
    
    loaded_lanelet_map = true;
  }
  else {
  


    std::cerr << " setting up subscriber\n";
    // xml_map_sub = rosnode.subscribe("/lanelet_map_xml", 10000,  xmlMapCallback);
    bin_map_sub = rosnode.subscribe("/lanelet_map_bin", 10000, binMapCallback);
    
  }
  
  ros::Publisher crosswalk_polygon_pub = rosnode.advertise<jsk_recognition_msgs::PolygonArray>("crosswalk_lanelets_poly", 1);;
  ros::Publisher road_polygon_pub = rosnode.advertise<jsk_recognition_msgs::PolygonArray>("road_lanelets_poly", 1);;
  ros::Publisher stopline_pub = rosnode.advertise<visualization_msgs::MarkerArray>("stop_lines", 1);
  ros::Publisher traffic_light_pub = rosnode.advertise<visualization_msgs::MarkerArray>("traffic_lights", 1);
  ros::Publisher traffic_light_ls_pub = rosnode.advertise<visualization_msgs::MarkerArray>("traffic_lights_ls", 1);

  road_border_pub = rosnode.advertise<visualization_msgs::MarkerArray>("road_lanelets", 1000);

  
  float stop_lines_color_params[4] = { 1.0, 0.0, 0.0, 0.3};
  float road_ll_color_params[4] = { 1.0, 1.0, 1.0, 0.2};

    
  ros::spinOnce();
  ros::Rate loop_rate(10);
  int loop_count = 0;

  // wait until map loaded
  while(ros::ok() && !loaded_lanelet_map) // && loop_count < 10)
    {
      //std::cerr << "waiting for map\n";
      ros::spinOnce();
      loop_count++;
      //  loop_rate.sleep();
    }
  
  std::cerr << "map loaded from subscriber - now visualize\n";
  
  lanelet::Lanelets all_lanelets = lanelet_utils::Query::laneletLayer(viz_lanelet_map);
  
  std::cerr <<"1\n";
  
  lanelet::Lanelets road_lanelets = lanelet_utils::Query::roadLanelets(all_lanelets);
  lanelet::Lanelets crosswalk_lanelets = lanelet_utils::Query::crosswalkLanelets(all_lanelets);

  std::vector<lanelet::ConstLineString3d> stop_lines = lanelet_utils::Query::stopLinesLanelets(road_lanelets);
  std::vector<lanelet::TrafficLight::Ptr> tl_reg_elems = lanelet_utils::Query::trafficLights(all_lanelets);
  
  
  std::cerr << "size of road laneles = " << road_lanelets.size() << "\n";

  
  while (ros::ok()) {
    //    lanelet_utils::Visualization::publishLaneletsAsPolygonArray(crosswalk_lanelets, crosswalk_polygon_pub);
  ros::spinOnce();
    loop_count++;
    loop_rate.sleep();


    lanelet_utils::Visualization::publishLaneletsAsMarkerArray(road_lanelets, road_border_pub, road_ll_color_params);
    lanelet_utils::Visualization::publishLaneletsAsPolygonArray(road_lanelets, road_polygon_pub);
    lanelet_utils::Visualization::publishLaneletsAsPolygonArray(crosswalk_lanelets, crosswalk_polygon_pub);
    
    
    lanelet_utils::Visualization::publishLineStringsAsMarkerArray(stop_lines, stopline_pub, "stop_lines", stop_lines_color_params, true);
    lanelet_utils::Visualization::publishTrafficLightsLineStringsAsMarkerArray(tl_reg_elems, traffic_light_ls_pub);
 lanelet_utils::Visualization::publishTrafficLightsAsMarkerArray(tl_reg_elems, traffic_light_pub);
   
    
   }
  
  return 0;
}
