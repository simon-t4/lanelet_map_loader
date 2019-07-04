#pragma once
#include <lanelet2_io/Exceptions.h>
#include <lanelet2_io/Projection.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_io/io_handlers/Factory.h>
#include <lanelet2_io/io_handlers/Writer.h>
#include <lanelet2_io/io_handlers/Serialize.h>

#include <lanelet2_io/io_handlers/OsmHandler.h>
#include <lanelet2_io/io_handlers/OsmFile.h>
#include <lanelet2_projection/UTM.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/LineStringOrPolygon.h>
#include <lanelet2_core/primitives/Lanelet.h>
//#include <Eigen/Eigen>

#include <jsk_recognition_msgs/PolygonArray.h>
#include <geometry_msgs/PolygonStamped.h>
#include "rosUTM.h"
#include "mgrs_projector.hpp"
//#include <lanelet_msgs/PointArray.h>
#include <lanelet_msgs/MapXML.h>
#include <lanelet_msgs/MapBin.h>
#include <pugixml.hpp>


#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>

#include <cstdio>
#include <sstream>

namespace lanelet_utils {


  class Map {
  public:
    
    static int toXMLMsg(lanelet::LaneletMapPtr map, lanelet_msgs::MapXML& msg, lanelet::projection::RosUtmProjector& projector);
    static int fromXMLMsg(lanelet_msgs::MapXML msg, lanelet::LaneletMapPtr &map);

    static void toBinMsg(const lanelet::LaneletMapPtr map, lanelet_msgs::MapBin& msg);
    static void fromBinMsg(const lanelet_msgs::MapBin msg, lanelet::LaneletMapPtr& map);
    
  private:
    Map(); //contructor private - do not instatiate class
  };


  class Visualization {
  public:

    static void lineString2Marker(int lane_id, lanelet::ConstLineString3d ls, visualization_msgs::Marker& points,
				  visualization_msgs::Marker& line_strip,
				  std::string frame_id, std::string ns, float lr, float lg, float lb, float lss= 0.1, bool fake_z = false);
    static void publishLineStringsAsMarkerArray(std::vector<lanelet::ConstLineString3d> line_strings, ros::Publisher &pub, std::string name_space,
						float *line_param = NULL, bool fake_z = false);
    static  void publishTrafficLightsLineStringsAsMarkerArray(std::vector<lanelet::TrafficLight::Ptr> tl_reg_elems, ros::Publisher & pub);

    static void publishTrafficLightsAsMarkerArray(std::vector<lanelet::TrafficLight::Ptr> tl_reg_elems, ros::Publisher & pub);

    static void publishLaneletAsMarkers(lanelet::ConstLanelet & lanelet, ros::Publisher &pub,
				       float *lline_color, float *rline_color, std::string ns, int id);

    static void publishLaneletsAsPolygonArray(lanelet::Lanelets & lanelets, ros::Publisher & pub);

    static void publishLaneletsAsMarkers(lanelet::Lanelets & lanelets, ros::Publisher &pub,
					 float *lline_param = NULL, float *rline_param = NULL);
    static void publishLaneletsAsMarkerArray(lanelet::Lanelets & lanelets, ros::Publisher &pub,
					     float *lline_param = NULL, float *rline_param = NULL);
    static void lanelet2PolygonStamped(int id, lanelet::Lanelet ll, geometry_msgs::PolygonStamped & polygon,
				       std::string frame_id);

    //static void visualizeLaneletMap(lanelet::LaneletMapPtr ll_map);
    
  private:
    Visualization(); // constructor provate - do not instantiate class
  };

  class Query {
  public:

    static lanelet::Lanelets laneletLayer(lanelet::LaneletMapPtr ll_Map);
    static lanelet::Lanelets subtypeLanelets(lanelet::Lanelets lls, const char subtype[]);
    static lanelet::Lanelets crosswalkLanelets(lanelet::Lanelets lls);
    static lanelet::Lanelets roadLanelets(lanelet::Lanelets lls);

    static std::vector<lanelet::TrafficLight::Ptr> trafficLights(lanelet::Lanelets lanelets);

    static std::vector<lanelet::ConstLineString3d> stopLinesLanelets(lanelet::Lanelets lanelets);
    static std::vector<lanelet::ConstLineString3d> stopLinesLanelet(lanelet::Lanelet ll);
  private:
    Query(); // constructor private - do not instantiate class
  };
}

