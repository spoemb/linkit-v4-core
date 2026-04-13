/**
 * @file haversine.hpp
 * @brief Haversine formula — great-circle distance between two GPS coordinates.
 */

#pragma once

/// @brief Compute distance (km) between two GPS points using the haversine formula.
/// @param lon1  Longitude of point 1 (degrees).
/// @param lat1  Latitude of point 1 (degrees).
/// @param lon2  Longitude of point 2 (degrees).
/// @param lat2  Latitude of point 2 (degrees).
/// @return Distance in kilometers.
double haversine_distance(double lon1, double lat1, double lon2, double lat2);
