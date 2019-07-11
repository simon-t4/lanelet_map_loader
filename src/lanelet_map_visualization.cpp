#include <ros/ros.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
 
#include <lanelet2_projection/UTM.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_msgs/MapBin.h>

#include <jsk_recognition_msgs/PolygonArray.h>
#include <geometry_msgs/PolygonStamped.h>

#include <lanelet2_extension/io/message_conversion.hpp>
#include <lanelet2_extension/query/autoware_query.h>
#include <lanelet2_extension/visualization/autoware_visualization.h>
#include <lanelet2_extension/projection/mgrs_projector.hpp>
#include <lanelet2_extension/regulatory_elements/autoware_traffic_light.hpp>

#include <unistd.h>
#include <ios>
#include <iostream>
#include <fstream>
#include <string>
#include <cstdio>
#include <sstream>

//std::string example_map_path = "/home/simon/work/catkin_ws/src/lanelet2/lanelet2_maps/res/mapping_example.osm";
//std::string example_map_path = "/home/simon/work/peoria_data/map/Lanelet2/Peoria_0301_2019_fixed_edit.osm";
std::string example_map_path = "/home/simon/work/peoria_data/map/Autoware_lanelet2/Peoria_autoware_lanelet_20190702.osm";



static bool viz_lanelets_borders = true;
static bool viz_lanelets_poly = false;
static bool viz_lanelets_centerline = true;

//-------------------------------------------------------------------------
//
//
//
//-------------------------------------------------------------------------


static bool loaded_lanelet_map;
static lanelet::LaneletMapPtr viz_lanelet_map;

//-------------------------------------------------------------------------
//
//
//
//-------------------------------------------------------------------------
 
void binMapCallback(lanelet2_msgs::MapBin msg)
{
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

  ros::init(argc, argv, "lanelet_map_vsiualizer");  
  ros::NodeHandle rosnode;
  //  ros::Subscriber xml_map_sub;
  ros::Subscriber bin_map_sub;
   
  if (load_from_file==true) {

    lanelet::ErrorMessages errors;
    lanelet::projection::MGRSProjector projector;
    viz_lanelet_map = load(example_map_path, projector, &errors);
    
    loaded_lanelet_map = true;
  }
  else {  
    bin_map_sub = rosnode.subscribe("/lanelet_map_bin", 10000, binMapCallback);
  }


  
  ros::Publisher crosswalk_polygon_pub = rosnode.advertise<jsk_recognition_msgs::PolygonArray>("crosswalk_lanelets_poly", 1, true);;
  ros::Publisher road_polygon_pub = rosnode.advertise<jsk_recognition_msgs::PolygonArray>("road_lanelets_poly", 1, true);;
  ros::Publisher stopline_pub = rosnode.advertise<visualization_msgs::MarkerArray>("stop_lines", 1, true);
  ros::Publisher traffic_light_pub = rosnode.advertise<visualization_msgs::MarkerArray>("traffic_lights", 1, true);
  ros::Publisher traffic_light_ls_pub = rosnode.advertise<visualization_msgs::MarkerArray>("traffic_lights_ls", 1, true);
  ros::Publisher traffic_light_polygon_pub = rosnode.advertise<jsk_recognition_msgs::PolygonArray>("traffic_lights_poly", 1, true);
  ros::Publisher road_border_pub = rosnode.advertise<visualization_msgs::MarkerArray>("road_lanelets", 1, true);


  // color params r,g,b, line width
  float stop_lines_color_params[4] = { 1.0, 0.0, 0.0, 0.2};
  float road_ll_color_params[4] = { 1.0, 1.0, 1.0, 0.1};
  
    
  ros::spinOnce();
  ros::Rate loop_rate(10);
  int loop_count = 0;
  
  // wait until map loaded
  while(ros::ok() && !loaded_lanelet_map)
    {
      ros::spinOnce();
      // loop_rate.sleep();
    }
  std::cerr << "Map is loaded\n";

  // get lanelets etc to visualize
  lanelet::Lanelets all_lanelets = lanelet_utils::Query::laneletLayer(viz_lanelet_map);
  lanelet::Lanelets road_lanelets = lanelet_utils::Query::roadLanelets(all_lanelets);
  lanelet::Lanelets crosswalk_lanelets = lanelet_utils::Query::crosswalkLanelets(all_lanelets);
  std::vector<lanelet::ConstLineString3d> stop_lines = lanelet_utils::Query::stopLinesLanelets(road_lanelets);
  std::vector<lanelet::TrafficLight::Ptr> tl_reg_elems = lanelet_utils::Query::trafficLights(all_lanelets);
  std::vector<lanelet::autoware::AutowareTrafficLight::Ptr> aw_tl_reg_elems = lanelet_utils::Query::autowareTrafficLights(all_lanelets);


  
  
  // publish - shift to 
  while (ros::ok()) {


    if (viz_lanelets_borders) lanelet_utils::Visualization::publishLaneletsAsMarkerArray(road_lanelets, road_border_pub, viz_lanelets_centerline, road_ll_color_params);
    if (viz_lanelets_poly) lanelet_utils::Visualization::publishLaneletsAsPolygonArray(road_lanelets, road_polygon_pub);
    lanelet_utils::Visualization::publishLaneletsAsPolygonArray(crosswalk_lanelets, crosswalk_polygon_pub); 
    lanelet_utils::Visualization::publishLineStringsAsMarkerArray(stop_lines, stopline_pub, "stop_lines", stop_lines_color_params, true);
    lanelet_utils::Visualization::publishTrafficLightsLineStringsAsMarkerArray(tl_reg_elems, traffic_light_ls_pub);
    //lanelet_utils::Visualization::publishTrafficLightsAsMarkerArray(tl_reg_elems, traffic_light_pub);
    lanelet_utils::Visualization::publishAutowareTrafficLightsAsMarkerArray(aw_tl_reg_elems, traffic_light_pub);
    lanelet_utils::Visualization::publishTrafficLightsAsPolygonArray(tl_reg_elems, traffic_light_polygon_pub);


    ros::spinOnce();
    loop_count++;
    loop_rate.sleep();
    
  }   
 
  return 0;
}
