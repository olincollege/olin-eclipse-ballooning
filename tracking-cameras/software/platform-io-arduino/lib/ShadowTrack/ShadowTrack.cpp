// Program to calculate desired orientation of balloon camera based on time (UTC) and GPS data.
// adapted from azimuth/distance calculator: http://cosinekitty.com/compass.html
// eclipse shadow path data from: https://eclipse.gsfc.nasa.gov/SEpath/SEpath2001/SE2023Oct14Apath.html

#include <iostream>
#include <cmath>
#include "read_csv.h"
#include "ShadowTrack.h"
#include "Arduino.h"

// // defines coordinate structure
// struct Coordinates
// {
//     double lat;
//     double lon;
//     double alt;
// };

// // define point structure
// struct Point
// {
//     double x;
//     double y;
//     double z;
//     double radius;
//     double nx;
//     double ny;
//     double nz;
// };

// find radius of earth at given latitude (earth is elliptical)
double find_earth_radius(double lat)
{
    double a = 6378137.0; // equatorial radius in meters
    double b = 6356752.3; // polar radius in meters
    double cos_lat = std::cos(lat);
    double sin_lat = std::sin(lat);
    double t1 = a * a * cos_lat;
    double t2 = b * b * sin_lat;
    double t3 = a * cos_lat;
    double t4 = b * sin_lat;
    return std::sqrt((t1 * t1 + t2 * t2) / (t3 * t3 + t4 * t4));
}

// earth is not a sphere -- used in converting between lat/lon to cartesian
double find_geocentric_latitude(double lat)
{
    double e2 = 0.00669437999014;
    double clat = std::atan((1.0 - e2) * std::tan(lat));
    return clat;
}

// convert location to x, y, z
Point location_to_point(const Coordinates &coords)
{
    double lat = coords.lat * (M_PI / 180.0);
    double lon = coords.lon * (M_PI / 180.0);
    double radius = find_earth_radius(lat);
    double clat = find_geocentric_latitude(lat);

    double cos_lon = std::cos(lon);
    double sin_lon = std::sin(lon);
    double cos_lat = std::cos(clat);
    double sin_lat = std::sin(clat);

    double x = radius * cos_lon * cos_lat;
    double y = radius * sin_lon * cos_lat;
    double z = radius * sin_lat;

    double cos_glat = std::cos(lat);
    double sin_glat = std::sin(lat);

    double nx = cos_glat * cos_lon;
    double ny = cos_glat * sin_lon;
    double nz = sin_glat;

    x += coords.alt * nx;
    y += coords.alt * ny;
    z += coords.alt * nz;

    return {x, y, z, radius, nx, ny, nz};
}

// rotate coordinate system so point a is at 0, 0
Point rotate_globe(const Coordinates &b, const Coordinates &a, double b_radius, double a_radius)
{
    Coordinates b_rotated = {b.lat, b.lon - a.lon, b.alt};
    Point b_rotated_point = location_to_point(b_rotated);

    double a_lat = -a.lat * M_PI / 180.0;
    a_lat = find_geocentric_latitude(a_lat);

    double a_cos = std::cos(a_lat);
    double a_sin = std::sin(a_lat);

    double b_x = (b_rotated_point.x * a_cos) - (b_rotated_point.z * a_sin);
    double b_y = b_rotated_point.y;
    double b_z = (b_rotated_point.x * a_sin) + (b_rotated_point.z * a_cos);

    return {b_x, b_y, b_z, b_radius};
}

// normalize vector between point a and b
Point normalize_vector_diff(const Point &point_b, const Point &point_a)
{
    double dx = point_b.x - point_a.x;
    double dy = point_b.y - point_a.y;
    double dz = point_b.z - point_a.z;
    double dist2 = dx * dx + dy * dy + dz * dz;
    if (dist2 == 0)
    {
        return {0, 0, 0, 0};
    }
    double dist = std::sqrt(dist2);
    return {dx / dist, dy / dist, dz / dist, 1.0};
}

// main function to convert lat/lon/alt to azimuth/elevation of shadow wrt balloon
std::pair<double, double> calc_angle_to_shadow(const Coordinates &shadow, const Coordinates &balloon)
{
    Point balloon_point = location_to_point(balloon);
    Point shadow_point = location_to_point(shadow);

    Point shadow_rotated = rotate_globe(shadow, balloon, shadow_point.radius, balloon_point.radius);
    double azimuth = 0.0, altitude = 0.0;

    if ((shadow_rotated.z * shadow_rotated.z + shadow_rotated.y * shadow_rotated.y) > 1.0e-6)
    {
        double theta = std::atan2(shadow_rotated.z, shadow_rotated.y) * 180.0 / M_PI;
        azimuth = 90.0 - theta;
        if (azimuth < 0.0)
        {
            azimuth += 360.0;
        }
        if (azimuth > 360.0)
        {
            azimuth -= 360.0;
        }
    }

    Point pointing_vector = normalize_vector_diff(shadow_point, balloon_point);
    if (pointing_vector.radius == 1.0)
    {
        altitude = 90.0 - (180.0 / M_PI) * std::acos(pointing_vector.x * balloon_point.nx + pointing_vector.y * balloon_point.ny + pointing_vector.z * balloon_point.nz);
    }

    return std::make_pair(azimuth, altitude);
}

std::pair<double, double> ShadowTrack(int hours, int mins, int seconds, float balloon_lat, float balloon_long, float balloon_elev)
{
    pair<double, double> shadowCoordinates = calculate_lat_lon(hours, mins, seconds);

    Coordinates shadow_coordinates = {shadowCoordinates.first, shadowCoordinates.second, 0};
    Coordinates balloon_coordinates = {balloon_lat, balloon_long, balloon_elev};

    // Return pair of angles: azimuth and altitude of shadow relative to balloon location
    pair<double, double> pointing_angles = calc_angle_to_shadow(shadow_coordinates, balloon_coordinates);
    return pointing_angles;
}
