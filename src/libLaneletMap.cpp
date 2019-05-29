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
#include <Eigen/Eigen>

#include <cstdio>

#include <sstream>
#include "libLaneletMap.h"

#include <unistd.h>
#include <ios>
#include <iostream>
#include <fstream>
#include <string>

using namespace lanelet_utils;


//-------------------------------------------------------------------------
//
// toXMLMsg()
// takes a projector as a parameter as assume most cases have loaded directly
// from file - in which case have created a projector already
//
//-------------------------------------------------------------------------

int Map::toXMLMsg(lanelet::LaneletMapPtr map, lanelet_msgs::MapXML& msg, lanelet::projection::RosUtmProjector& projector)
{
  // convert laneletmap into osm file data structure
  std::unique_ptr<lanelet::osm::File> map_osm;
  lanelet::ErrorMessages errors;
  
  // base class of IOHandler has projector as an argument for the constructor - must have
  // IOHandler -> Writer -> OsmWriter
  lanelet::io_handlers::OsmWriter osm_writer(projector);
  
  map_osm = osm_writer.toOsmFile(*map, errors);  // write map to an osm file data structure
  
  std::stringstream ss;
  std::unique_ptr<pugi::xml_document> pugi_xml_doc;
  
  pugi_xml_doc = lanelet::osm::write(*map_osm); // write osm file data to a pugi_xml doc
  pugi_xml_doc->save(ss); // write xml to stringstream
  
  //std::string all_xml = ss.str();  // then to string for ros message
  msg.xml_map = ss.str();
  int status = 1;
  return status;
}

//-------------------------------------------------------------------------
//
//
//
//-------------------------------------------------------------------------

int Map::fromXMLMsg(lanelet_msgs::MapXML msg, lanelet::LaneletMapPtr map)
{
  int status = 0;
  //  lanelet::Origin origin({49, 8.4});
  lanelet::Origin origin({0,0});
  
  // origin not used in projector but projector template requires one
  lanelet::projection::RosUtmProjector ros_projector(origin);
  
  lanelet::ErrorMessages errors;
  
  std::cerr << "Recievd lanelet map xml string message size = " << msg.xml_map.size() << "\n";

  // got map in xml form
  // now reverse the steps used to publish
  // string -> srtingstream -> xml doc -> osm format ->lanelet map

  
  std::stringstream ss;
  ss << msg;
  pugi::xml_document pugi_xml_doc;
  pugi::xml_parse_result result = pugi_xml_doc.load(ss); 
  if (result.status == pugi::status_ok) {
    // failed beacuse no default projector set, now works
    lanelet::osm::File map_osm = lanelet::osm::read(pugi_xml_doc);  // read a map in to an osm file data structure
    lanelet::io_handlers::OsmParser osm_parser(ros_projector);
    
    map = osm_parser.fromOsmFile(map_osm, errors);
    status = 1;
   
  }
  else {
    std::cerr << "WARNING xmlMapToLaneletMap(): could not load xml string stream into xml document\n";
  }
  return status;

}
