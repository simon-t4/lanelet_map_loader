#include "ros/ros.h"
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <jsk_recognition_msgs/PolygonArray.h>
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



// returns all lanelets in laneletLayer - don't know how to convert PrimitveLayer<Lanelets> -> std::vector<Lanelets>
lanelet::Lanelets Query::laneletLayer(lanelet::LaneletMapPtr ll_map)
{
  lanelet::Lanelets lanelets;

  std::cerr << "here";
  for (auto li = ll_map->laneletLayer.begin(); li !=ll_map->laneletLayer.end(); li++) {
   
    lanelets.push_back(*li);
  }
    std::cerr << "here";

  return lanelets;
}

lanelet::Lanelets Query::subtypeLanelets(lanelet::Lanelets lls, const char subtype[])
{
  lanelet::Lanelets subtype_lanelets;

  
  for (auto li = lls.begin(); li !=lls.end(); li++) {
    lanelet::Lanelet ll = *li;
    
    if (ll.hasAttribute(lanelet::AttributeName::Subtype)){
      lanelet::Attribute attr = ll.attribute(lanelet::AttributeName::Subtype);
      if (attr.value() == subtype) {
	subtype_lanelets.push_back(ll);
      }
    }
     
  }
  
  //std::cerr << "Found " << cross_walks.size() << " crosswalk lanelets\n";
  return subtype_lanelets;
}
lanelet::Lanelets Query::crosswalkLanelets(lanelet::Lanelets lls)
{

  return (Query::subtypeLanelets(lls, lanelet::AttributeValueString::Crosswalk));
}

lanelet::Lanelets Query::roadLanelets(lanelet::Lanelets lls)
{
  return (Query::subtypeLanelets(lls, lanelet::AttributeValueString::Road));
}

std::vector<lanelet::TrafficLight::Ptr> Query::trafficLights(lanelet::Lanelets lanelets)
{
  std::vector<lanelet::TrafficLight::Ptr> tl_reg_elems;

  for (auto i = lanelets.begin(); i != lanelets.end(); i++) {
    
    lanelet::Lanelet ll = *i;
    std::vector<lanelet::TrafficLight::Ptr> ll_tl_re = ll.regulatoryElementsAs<lanelet::TrafficLight>();

    // insert unique tl into array
    for (auto tli = ll_tl_re.begin(); tli != ll_tl_re.end(); tli++) {

      lanelet::TrafficLight::Ptr tl_ptr = *tli;
      lanelet::Id id = tl_ptr->id();
      bool unique_id = true;
      for  (auto ii = tl_reg_elems.begin(); ii != tl_reg_elems.end(); ii++){
	if (id == (*ii)->id()){
	  unique_id = false;
	  break;
	}
      }
      if (unique_id) tl_reg_elems.push_back(tl_ptr);
	
    }
  }
  return tl_reg_elems;
}


// return all stop lines and ref lines from a given set of lanelets
std::vector<lanelet::ConstLineString3d> Query::stopLinesLanelets(lanelet::Lanelets lanelets)
{
  std::vector<lanelet::ConstLineString3d> stoplines;

  for (auto lli = lanelets.begin(); lli != lanelets.end(); lli++) {
      std::vector<lanelet::ConstLineString3d> ll_stoplines;
      ll_stoplines = Query::stopLinesLanelet(*lli);
      stoplines.insert(stoplines.end(), ll_stoplines.begin(), ll_stoplines.end());
  }
  return stoplines;
}

// return all stop and ref lines from a given lanel
std::vector<lanelet::ConstLineString3d> Query::stopLinesLanelet(lanelet::Lanelet ll)
{ 
  std::vector<lanelet::ConstLineString3d> stoplines;

  // find stop lines referened by right ofway reg. elems.
  std::vector<std::shared_ptr<const lanelet::RightOfWay>> right_of_way_reg_elems = ll.regulatoryElementsAs<const lanelet::RightOfWay>();
  
  if (right_of_way_reg_elems.size() > 0)
    {
    
      // lanelet has a right of way elem elemetn
      for (auto j = right_of_way_reg_elems.begin(); j < right_of_way_reg_elems.end(); j++)
	{
	  if ((*j)->getManeuver(ll) == lanelet::ManeuverType::Yield)
	    {
	      // lanelet has a yield reg. elem.
	      lanelet::Optional<lanelet::ConstLineString3d> row_stopline_opt = (*j)->stopLine();
	      if (!!row_stopline_opt) stoplines.push_back(row_stopline_opt.get());
	  }
	  
	}
    }
  
  // find stop lines referenced by traffic lights
  std::vector<std::shared_ptr<const lanelet::TrafficLight>> traffic_light_reg_elems = ll.regulatoryElementsAs<const lanelet::TrafficLight>();
  
  if (traffic_light_reg_elems.size() > 0)
    {
	
      // lanelet has a traffic light elem elemetn
      for (auto j = traffic_light_reg_elems.begin(); j < traffic_light_reg_elems.end(); j++)
	{
	  
	  lanelet::Optional<lanelet::ConstLineString3d> traffic_light_stopline_opt = (*j)->stopLine();
	  if (!!traffic_light_stopline_opt) stoplines.push_back(traffic_light_stopline_opt.get());
	  
	}
      
    }
  // find stop lines referenced by traffic signs
  std::vector<std::shared_ptr<const lanelet::TrafficSign>> traffic_sign_reg_elems = ll.regulatoryElementsAs<const lanelet::TrafficSign>();
  
  if (traffic_sign_reg_elems.size() > 0)
    {
      
      // lanelet has a traffic sign reg elem - can have multiple ref lines (but stop sign shod have 1
      for (auto j = traffic_sign_reg_elems.begin(); j < traffic_sign_reg_elems.end(); j++)
	{
	  
	  lanelet::ConstLineStrings3d traffic_sign_stoplines = (*j)->refLines();
	  if (traffic_sign_stoplines.size() > 0) stoplines.push_back(traffic_sign_stoplines.front());
	  
	}
      
    }
  return stoplines;
}


void Visualization::publishTrafficLightsAsMarkerArray(std::vector<lanelet::TrafficLight::Ptr> tl_reg_elems, ros::Publisher & pub)
{

  
  visualization_msgs::MarkerArray tl_marker_array;
	
  int tl_count = 0;
  for (auto tli = tl_reg_elems.begin(); tli != tl_reg_elems.end(); tli++) {
	    
    lanelet::LineString3d ls;
  
    //std::cerr << "size of individual traffic light list of line strings = " <<  (*tli)->trafficLights().size() << "\n";


    //    for (auto li = (*tli)->trafficLights().begin(); li != (*tli)->trafficLights().end(); li++) {
    lanelet::LineStringOrPolygon3d lights = (*tli)->trafficLights().front();
    if (lights.isLineString()) { // traffic ligths can either polygons or linestrings
      ls = static_cast<lanelet::LineString3d>(lights);
      
      //std::cerr << "light = " << lights << "\n";
   	      
      int point_count = 0;
      for (auto pi = ls.begin(); pi != ls.end(); pi++) {
	lanelet::Point3d p = *pi;
	
	// visualise traffic lights
	visualization_msgs::Marker marker;
	uint32_t box_shape = visualization_msgs::Marker::CUBE; 
	
	marker.header.frame_id = "map";
	marker.header.stamp = ros::Time();
	marker.ns = "traffic_light";
	marker.id = tl_count;
	marker.type = box_shape;
	marker.pose.position.x = p.x();
	marker.pose.position.y = p.y();
	marker.pose.position.z = p.z();
  
	float s = 0.3;
	
	marker.scale.x = s;
	marker.scale.y = s;
	marker.scale.z = s;
	
	marker.color.r = 0.0f;
	marker.color.g = 0.0f;
	marker.color.b = 0.0f;
	marker.color.a = 1.0f;
	
	if (point_count == 0) marker.color.r = 1.0f;
	else if (point_count == 1) marker.color.g = 1.0f;
	if (point_count == 2) marker.color.b = 1.0f;
	
	marker.lifetime = ros::Duration();
	
	tl_marker_array.markers.push_back(marker);
	point_count++;
	tl_count++;
	
      }
      
      
    } 
  }
  
  pub.publish(tl_marker_array);
}

void Visualization::publishTrafficLightsLineStringsAsMarkerArray(std::vector<lanelet::TrafficLight::Ptr> tl_reg_elems, ros::Publisher & pub)
{

  
  //convert to to an array of linestrings and publish as marker array using exisitng function
  
  int tl_count = 0;
  std::vector<lanelet::ConstLineString3d> line_strings;
  for (auto tli = tl_reg_elems.begin(); tli != tl_reg_elems.end(); tli++) {
    
    lanelet::ConstLineString3d ls;
  
    //std::cerr << "size of individual traffic light list of line strings = " <<  (*tli)->trafficLights().size() << "\n";


    //    for (auto li = (*tli)->trafficLights().begin(); li != (*tli)->trafficLights().end(); li++) {
    lanelet::LineStringOrPolygon3d lights = (*tli)->trafficLights().front();
    if (lights.isLineString()) { // traffic ligths can either polygons or linestrings
      ls = static_cast<lanelet::LineString3d>(lights);
      line_strings.push_back(ls);
    }
    
    
  } 
  
  float line_param[4] = {1.0, 1.0, 0.0, 0.5};
  
  Visualization::publishLineStringsAsMarkerArray(line_strings, pub, "tl_line_strings", line_param);
  
}


void Visualization::publishLaneletsAsMarkers(lanelet::Lanelets & lanelets, ros::Publisher & pub,
					     float *lline_param, float *rline_param)
{
  if (lline_param == NULL) { // set defaul values rgb and line width
    lline_param = (float *) malloc (4*sizeof(float));
    
    lline_param[0] = 1.0; lline_param[1] = 0.0; lline_param[2] = 0.0; lline_param[3] = 0.4;
    
  }
   if (rline_param == NULL) {
     
     rline_param = (float *) malloc (4*sizeof(float));
     rline_param[0] = 0.0; rline_param[1] = 0.0; rline_param[2] = 1.0; rline_param[3] = 0.4;
     
   }
   int lcount = 0;

   for (auto li = lanelets.begin(); li !=lanelets.end(); li++) {
     lanelet::ConstLanelet lll = *li;
     
     // if (lcount < max_size)
     Visualization::publishLaneletAsMarkers(lll, pub, lline_param, rline_param, "lanelets", lcount);
     lcount++;
   }
   
   
}



void Visualization::publishLineStringsAsMarkerArray(std::vector<lanelet::ConstLineString3d> line_strings, ros::Publisher & pub,
						    std::string name_space, float *line_param, bool fake_z)
{


    if (line_param == NULL) { // set defaul values rgb and line width
      line_param = (float *) malloc (4*sizeof(float));
    
      line_param[0] = 1.0; line_param[1] = 0.0; line_param[2] = 0.0; line_param[3] = 0.2;
    }
  visualization_msgs::MarkerArray ls_marker_array;
  int ls_count = 0;
  for (auto i = line_strings.begin(); i != line_strings.end(); i++) {
    
    lanelet::ConstLineString3d ls = *i;
    visualization_msgs::Marker ls_marker, pt_marker;

    Visualization::lineString2Marker(ls_count, ls, pt_marker, ls_marker, "map", name_space, line_param[0], line_param[1], line_param[2], line_param[3], fake_z);

    ls_marker_array.markers.push_back(ls_marker);
    ls_count++;
    
  }

  std::cerr << "in publish line string - stop lines size = " << ls_marker_array.markers.size() << "\n";
  pub.publish(ls_marker_array);
}
 
void Visualization::publishLaneletsAsMarkerArray(lanelet::Lanelets & lanelets, ros::Publisher & pub,
					     float *lline_param, float *rline_param)
{
  if (lline_param == NULL && rline_param == NULL) { // set defaul values rgb and line width
    lline_param = (float *) malloc (4*sizeof(float));
    rline_param = (float *) malloc (4*sizeof(float));
    
    lline_param[0] = 1.0; lline_param[1] = 0.0; lline_param[2] = 0.0; lline_param[3] = 0.4;
    rline_param[0] = 0.0; rline_param[1] = 0.0; rline_param[2] = 1.0; rline_param[3] = 0.4;
    
  }
  else if (rline_param == NULL) { // if only one set use same color for both
     
    rline_param = (float *) malloc (4*sizeof(float));
    rline_param[0] = lline_param[0]; rline_param[1] =  lline_param[1]; rline_param[2] =  lline_param[2]; rline_param[3] =  lline_param[3];
     
   }
   int lcount = 0;
   visualization_msgs::MarkerArray lanelets_marker_array;
   for (auto li = lanelets.begin(); li !=lanelets.end(); li++) {
     lanelet::ConstLanelet lll = *li;
     
     // if (lcount < max_size)
     lanelet::ConstLineString3d left_ls = lll.leftBound();
     lanelet::ConstLineString3d right_ls = lll.rightBound();
     
     
     visualization_msgs::Marker left_line_strip, left_points, right_line_strip, right_points;
     
     Visualization::lineString2Marker(lcount, left_ls, left_points, left_line_strip, "map", "left_lane_bound", lline_param[0], lline_param[1], lline_param[2], lline_param[3], true);
     Visualization::lineString2Marker(lcount,right_ls, right_points, right_line_strip, "map", "right_lane_bound", rline_param[0], rline_param[1], rline_param[2], rline_param[3], true);
     
     lanelets_marker_array.markers.push_back(left_line_strip);
     lanelets_marker_array.markers.push_back(right_line_strip);
     lcount++;
   }
   
   pub.publish(lanelets_marker_array);
}

void Visualization::publishLaneletsAsPolygonArray(lanelet::Lanelets & lanelets, ros::Publisher & pub)
{
   int lcount = 0;
   jsk_recognition_msgs::PolygonArray lanelets_polygon_array;


   lanelets_polygon_array.header.frame_id = "map";
   lanelets_polygon_array.header.seq = 200001;
   lanelets_polygon_array.header.stamp = ros::Time();

   //uint32 i = 0:
   for (auto li = lanelets.begin(); li !=lanelets.end(); li++) {

     lanelet::Lanelet ll = *li;

     geometry_msgs::PolygonStamped polygon;
     //     std::cerr << "lanelet to poluygon\n";

     Visualization::lanelet2PolygonStamped(lcount, ll, polygon, "map");
     // std::cerr << "before add to arrays\n";
     lanelets_polygon_array.polygons.push_back(polygon);
     lanelets_polygon_array.labels.push_back(lcount);
     lanelets_polygon_array.likelihood.push_back(0.4);
     lcount++;
    
   }
   //std::cerr << "size of polygon array = "<< lanelets_polygon_array.polygons.size() << "\n";
   pub.publish(lanelets_polygon_array);
}



void Visualization::publishLaneletAsMarkers(lanelet::ConstLanelet & lanelet, ros::Publisher &pub,
					   float *lline_param, float *rline_param, std::string ns, int id)
{
  
  
  visualization_msgs::Marker left_line_strip, left_points, right_line_strip, right_points;
  
  lanelet::ConstLineString3d left_ls = lanelet.leftBound();
  lanelet::ConstLineString3d right_ls = lanelet.rightBound();
  
  
  Visualization::lineString2Marker(id*1000, left_ls, left_points, left_line_strip, "map", "left_lane_bound", lline_param[0], lline_param[1], lline_param[2], lline_param[3]);
  Visualization::lineString2Marker(id*1000+1,right_ls, right_points, right_line_strip, "map", "right_lane_bound", rline_param[0], rline_param[1], rline_param[2], rline_param[3]);
  
  pub.publish(left_points);
  pub.publish(left_line_strip);
  pub.publish(right_points);
  pub.publish(right_line_strip);


}
//-------------------------------------------------------------------------
//
// toXMLMsg()
// takes a projector as a parameter as assume most cases have loaded directly
// from file - in which case have created a projector already
//
//-------------------------------------------------------------------------


void Visualization::lanelet2PolygonStamped(int id, lanelet::Lanelet ll, geometry_msgs::PolygonStamped & polygon,
					   std::string frame_id)
{
  polygon.header.frame_id = frame_id;
  polygon.header.seq = id;
  polygon.header.stamp = ros::Time();

  
  lanelet::ConstLineString3d left_ls = ll.leftBound();
  lanelet::ConstLineString3d right_ls = ll.rightBound();

  int polygon_size= left_ls.size() +right_ls.size();
  // polygon.polygon.points.resize(polygon_size);
  geometry_msgs::Point32 p;
  
  for (auto i = left_ls.begin(); i != left_ls.end(); i++) {
 geometry_msgs::Point32 p;
 
    p.x = (*i).x();
    p.y = (*i).y();
    p.z = 118.0;//(*i).z(); // peoria data has no z
    polygon.polygon.points.push_back(p);
    
  }
  
  for (auto i = right_ls.end(); i != right_ls.begin();) {
    i--;
     geometry_msgs::Point32 p;
 
   p.x = (*i).x();
    p.y = (*i).y();
    p.z = 118.0;//(*i).z(); // peoria data has no z
    polygon.polygon.points.push_back(p);
    
  }
  
  
}

void Visualization::lineString2Marker(int lane_id, lanelet::ConstLineString3d ls, visualization_msgs::Marker& points,
				  visualization_msgs::Marker& line_strip,
				      std::string frame_id, std::string ns, float lr, float lg, float lb, float lss, bool fake_z)
{
  
  
  points.header.frame_id = line_strip.header.frame_id = frame_id;
  points.header.stamp = line_strip.header.stamp = ros::Time();
  points.ns = line_strip.ns = ns;
  points.action = line_strip.action = visualization_msgs::Marker::ADD;
  
  points.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
  points.id = lane_id;
  line_strip.id = lane_id;

  points.type = visualization_msgs::Marker::POINTS;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;
	
  points.scale.x = 0.2;
  points.scale.y = 0.2;
  line_strip.scale.x = lss; 


  points.color.g = 1.0f;
  points.color.a = 1.0f;
  line_strip.color.r = lr;
  line_strip.color.g = lg;
  line_strip.color.b = lb;
  line_strip.color.a = 1.0f;

  // fill out lane line
  for (auto i = ls.begin(); i != ls.end(); i++){
    geometry_msgs::Point p;
    p.x = (*i).x();
    p.y = (*i).y();
    p.z = (*i).z();
    if (fake_z) p.z = 118.0;
    points.points.push_back(p);
    line_strip.points.push_back(p);
  }
}



//-------------------------------------------------------------------------
//
// toXMLMsg()
// takes a projector as a parameter as assume most cases have loaded directly
// from file - in which case have created a projector already
//
//-------------------------------------------------------------------------



void Map::toBinMsg(const lanelet::LaneletMapPtr map, lanelet_msgs::MapBin& msg)
{
  std::stringstream ss;
  boost::archive::binary_oarchive oa(ss);
  oa << *map;
  auto id_counter = lanelet::utils::getId();
  oa << id_counter;
  msg.data = ss.str();
  std::cout << ss.str().size() << std::endl;
}

void Map::fromBinMsg(const lanelet_msgs::MapBin msg, lanelet::LaneletMapPtr& map)
{
  std::stringstream ss;
  std::unique_ptr<lanelet::LaneletMap> laneletMap = std::make_unique<lanelet::LaneletMap>();
  // map = std::make_shared<lanelet::LaneletMap>();
  ss << msg.data;
  boost::archive::binary_iarchive oa(ss);
  oa >> *laneletMap;
  lanelet::Id id_counter;
  oa >> id_counter;
  lanelet::utils::registerId(id_counter);
  map = std::move(laneletMap);
  // for(auto lanelet: map->laneletLayer)
  //   std::cout << lanelet << std::endl;

}

int Map::toXMLMsg(lanelet::LaneletMapPtr map, lanelet_msgs::MapXML& msg, lanelet::projection::RosUtmProjector& projector)
{
  // convert laneletmap into osm file data structure
  std::unique_ptr<lanelet::osm::File> map_osm;
  lanelet::ErrorMessages errors;
  lanelet::projection::MGRSProjector mprojector;
 
  // base class of IOHandler has projector as an argument for the constructor - must have
  // IOHandler -> Writer -> OsmWriter
  lanelet::io_handlers::OsmWriter osm_writer(mprojector);
  
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

int Map::fromXMLMsg(lanelet_msgs::MapXML msg, lanelet::LaneletMapPtr& map)
{
  int status = 0;
  lanelet::Origin origin({49, 8.4});
  //lanelet::Origin origin({0,0});
  
  // origin not used in projector but projector template requires one
  // lanelet::projection::RosUtmProjector ros_projector(origin);
  lanelet::projection::MGRSProjector projector;
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
    lanelet::io_handlers::OsmParser osm_parser(projector);
    map = osm_parser.fromOsmFile(map_osm, errors);
    status = 1;
   
  }
  else {
    std::cerr << "WARNING xmlMapToLaneletMap(): could not load xml string stream into xml document\n";
  }
  return status;

}
