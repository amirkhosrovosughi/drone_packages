#include "gps/gps_local_frame.hpp"

#include <cmath>

namespace
{

// WGS84 ellipsoid constants — implementation detail, intentionally file-local.
// Anonymous namespace is the correct C++ idiom for file-local helpers and
// constants; it prevents external linkage without the verbosity of 'static'.
constexpr double kWgs84SemiMajorAxisM = 6378137.0;
constexpr double kWgs84FirstEccentricitySquared = 6.69437999014e-3;
constexpr double kDegToRad = M_PI / 180.0;

Eigen::Vector3d geodeticToEcef(const GeodeticCoordinate& coordinate)
{
  const double latitudeRad = coordinate.latitudeDeg * kDegToRad;
  const double longitudeRad = coordinate.longitudeDeg * kDegToRad;
  const double sinLatitude = std::sin(latitudeRad);
  const double cosLatitude = std::cos(latitudeRad);
  const double sinLongitude = std::sin(longitudeRad);
  const double cosLongitude = std::cos(longitudeRad);

  const double radiusOfCurvature =
    kWgs84SemiMajorAxisM /
    std::sqrt(1.0 - kWgs84FirstEccentricitySquared * sinLatitude * sinLatitude);

  const double x =
    (radiusOfCurvature + coordinate.altitudeM) * cosLatitude * cosLongitude;
  const double y =
    (radiusOfCurvature + coordinate.altitudeM) * cosLatitude * sinLongitude;
  const double z =
    (radiusOfCurvature * (1.0 - kWgs84FirstEccentricitySquared) +
    coordinate.altitudeM) * sinLatitude;

  return Eigen::Vector3d(x, y, z);
}

Eigen::Vector3d wgs84ToLocalEnu(
  const GeodeticCoordinate& coordinate,
  const GeodeticCoordinate& anchor)
{
  const Eigen::Vector3d coordinateEcef = geodeticToEcef(coordinate);
  const Eigen::Vector3d anchorEcef = geodeticToEcef(anchor);
  const Eigen::Vector3d deltaEcef = coordinateEcef - anchorEcef;

  const double anchorLatitudeRad = anchor.latitudeDeg * kDegToRad;
  const double anchorLongitudeRad = anchor.longitudeDeg * kDegToRad;
  const double sinLatitude = std::sin(anchorLatitudeRad);
  const double cosLatitude = std::cos(anchorLatitudeRad);
  const double sinLongitude = std::sin(anchorLongitudeRad);
  const double cosLongitude = std::cos(anchorLongitudeRad);

  Eigen::Matrix3d ecefToEnu;
  ecefToEnu <<
    -sinLongitude,              cosLongitude,             0.0,
    -sinLatitude * cosLongitude, -sinLatitude * sinLongitude, cosLatitude,
     cosLatitude * cosLongitude,  cosLatitude * sinLongitude, sinLatitude;

  return ecefToEnu * deltaEcef;
}

GeodeticCoordinate toGeodeticCoordinate(const GpsReference& reference)
{
  GeodeticCoordinate coordinate;
  coordinate.latitudeDeg = reference.latitudeDeg;
  coordinate.longitudeDeg = reference.longitudeDeg;
  coordinate.altitudeM = reference.altitudeM;
  return coordinate;
}

GeodeticCoordinate toGeodeticCoordinate(const px4_msgs::msg::SensorGps& msg)
{
  GeodeticCoordinate coordinate;
  coordinate.latitudeDeg = msg.latitude_deg;
  coordinate.longitudeDeg = msg.longitude_deg;
  coordinate.altitudeM = msg.altitude_msl_m;
  return coordinate;
}

}  // namespace

bool GpsLocalFrame::hasAnchor() const
{
  return _hasAnchor;
}

void GpsLocalFrame::setAnchor(const LocalFrameAnchor& anchor)
{
  _anchor = anchor;
  _hasAnchor = true;
}

const LocalFrameAnchor& GpsLocalFrame::anchor() const
{
  if (!_hasAnchor)
  {
    throw std::logic_error("GPS local frame anchor is not available yet.");
  }

  return _anchor;
}

Eigen::Vector3d GpsLocalFrame::toEnu(const GpsReference& reference) const
{
  if (!_hasAnchor)
  {
    throw std::logic_error("Cannot project GPS reference without a local anchor.");
  }

  const Eigen::Vector3d projectedPosition = wgs84ToLocalEnu(
    toGeodeticCoordinate(reference),
    toGeodeticCoordinate(_anchor.anchorReference));
  return projectedPosition + _anchor.initialEnuPosition;
}

Eigen::Vector3d GpsLocalFrame::toEnu(const px4_msgs::msg::SensorGps& msg) const
{
  if (!_hasAnchor)
  {
    throw std::logic_error("Cannot project GPS sample without a local anchor.");
  }

  const Eigen::Vector3d projectedPosition = wgs84ToLocalEnu(
    toGeodeticCoordinate(msg),
    toGeodeticCoordinate(_anchor.anchorReference));
  return projectedPosition + _anchor.initialEnuPosition;
}