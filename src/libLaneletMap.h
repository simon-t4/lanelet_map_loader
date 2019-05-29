#pragma once
#include <lanelet2_io/Exceptions.h>
#include <lanelet2_io/Projection.h>
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
#include <lanelet2_core/primitives/Lanelet.h>
//#include <Eigen/Eigen>


#include "rosUTM.h"
#include <lanelet_msgs/PointArray.h>
#include <lanelet_msgs/MapXML.h>
#include <pugixml.hpp>

#include <cstdio>
#include <sstream>

namespace lanelet_utils {


  class Map {
  public:
    
    static int toXMLMsg(lanelet::LaneletMapPtr map, lanelet_msgs::MapXML& msg, lanelet::projection::RosUtmProjector& projector);
    static int fromXMLMsg(lanelet_msgs::MapXML msg, lanelet::LaneletMapPtr map);

    
  private:
    Map(); //contructor private - do not instatiate class
  };
  
}

