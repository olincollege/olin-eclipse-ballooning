// Helper file to include shadow calculations in camera tracking code

#ifndef __SHADOWTRACK_H
#define __SHADOWTRACK_H

#include "stdio.h"
#include <cmath>
#include <string>
#include <vector>
#include <SD.h>
#include <SPI.h>
#include <Wire.h>

using namespace std;

struct Coordinates
{
    double lat;
    double lon;
    double alt;
};

struct Point
{
    double x;
    double y;
    double z;
    double radius;
    double nx;
    double ny;
    double nz;
};

struct PathCoordinate
{
public:
    PathCoordinate(
        int hours,
        int mins,
        double latitude,
        double longitude)
    {
        Hours = hours;
        Mins = mins;
        Latitude = latitude;
        Longitude = longitude;
    }

    int Hours;
    int Mins;
    double Latitude;
    double Longitude;
};

// vector<PathCoordinate> load_csv_data();
vector<PathCoordinate> load_from_sd();
pair<double, double> calculate_lat_lon(int hours, int mins, int seconds);

double find_earth_radius(double lat);
double find_geocentric_latitude(double lat);
Point location_to_point(const Coordinates &coords);
Point rotate_globe(const Coordinates &b, const Coordinates &a, double b_radius, double a_radius);
Point normalize_vector_diff(const Point &point_b, const Point &point_a);
pair<double, double> calc_angle_to_shadow(const Coordinates &shadow, const Coordinates &balloon);
pair<double, double> ShadowTrack(int hours, int mins, int seconds, float balloon_lat, float balloon_long, float balloon_elev);

#endif