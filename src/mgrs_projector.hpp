#ifndef __mgrs_projector_hpp__
#define __mgrs_projector_hpp__
#include <lanelet2_io/Exceptions.h>
#include <lanelet2_io/Projection.h>
#include <GeographicLib/UTMUPS.hpp>
#include <GeographicLib/MGRS.hpp>
namespace lanelet {
  namespace projection {
    class MGRSProjector : public Projector
    {
    public:
      explicit MGRSProjector(Origin origin = Origin({0.0, 0.0}));
      /**
       * [MGRSProjector::forward projects gps lat/lon to MGRS 100km grid]
       * @param  gps [point with latitude longitude information]
       * @return     [projected point in MGRS coordinate]
       */
      BasicPoint3d forward(const GPSPoint& gps) const override;
      /**
       * [MGRSProjector::forward projects gpgs lat/lon to MGRS xyz coordinate]
       * @param  gps       [point with latitude longitude information]
       * @param  precision [resolution of MGRS Grid 0=100km, 1=10km, 2=1km, 3=100m, 4=10m, 5=1m]
       * @return           [projected point in MGRS coordinate]
       */
      BasicPoint3d forward( const GPSPoint& gps,const int precision) const;
      /**
       * [MGRSProjector::reverse projects point within MGRS 100km grid into gps lat/lon (WGS84)]
       * @param  mgrs_point [3d point in MGRS 100km grid]
       * @return            [projected point in WGS84]
       */
      GPSPoint reverse(const BasicPoint3d& mgrs) const override;
      /**
       * [MGRSProjector::reverse projects point within MGRS grid into gps lat/lon (WGS84)]
       * @param  mgrs_point [3d point in MGRS grid]
       * @param  mgrs_code  [MGRS grid code]
       * @return            [projected point in WGS84]
       */
      GPSPoint reverse(const BasicPoint3d& mgrs_point, const std::string &mgrs_code) const;
      /**
       * [MGRSProjector::setMGRSCode sets MGRS code used for reverse projection]
       * @param mgrs_code [MGRS code. Minimum requirement is GZD and 100 km Grid Square ID. e.g. "4QFJ"]
       */
      void setMGRSCode(const std::string &mgrs_code);
      /**
       * [MGRSProjector::setMGRSCode sets MGRS code used for reverse projection from gps lat/lon values]
       * @param gps       [gps point used to find GMRS Grid]
       * @param precision [resolution of MGRS Grid 0=100km, 1=10km, 2=1km, 3=100m, 4=10m, 5=1m]
       */
      void setMGRSCode(const GPSPoint& gps, const int precision = 0);
      /**
       * [getProjectedMGRSGrid returns mgrs]
       * @return [description]
       */
      std::string getProjectedMGRSGrid()const {return projected_grid_;};
      /**
       * [isMGRSCodeSet checks if mgrs code is set for reverse projection]
       * @return [true if mgrs_code member is set]
       */
      bool isMGRSCodeSet()const {return !mgrs_code_.empty();};
    private:
      /**
       * mgrs grid code used for reverse function
       */
      std::string mgrs_code_;
      /**
       * mgrs grid code that was last projected in previous foward function.
       * reverse function will use this if isMGRSCodeSet() returns false.
       */
      mutable std::string projected_grid_;
    };
  }
}

#endif
