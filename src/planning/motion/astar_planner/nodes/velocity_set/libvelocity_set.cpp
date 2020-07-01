#include "libvelocity_set.h"

geometry_msgs::Point ObstaclePoints::getObstaclePoint(const EControl &kind) const
{
  geometry_msgs::Point point;

  if (kind == EControl::STOP || kind == EControl::STOPLINE)
  {
    for (const auto &p : stop_points_)
    {
      point.x += p.x;
      point.y += p.y;
      point.z += p.z;
    }
    point.x /= stop_points_.size();
    point.y /= stop_points_.size();
    point.z /= stop_points_.size();

    return point;
  }
  else  // kind == DECELERATE
  {
    for (const auto &p : decelerate_points_)
    {
      point.x += p.x;
      point.y += p.y;
      point.z += p.z;
    }
    point.x /= decelerate_points_.size();
    point.y /= decelerate_points_.size();
    point.z /= decelerate_points_.size();

    return point;
  }
}
