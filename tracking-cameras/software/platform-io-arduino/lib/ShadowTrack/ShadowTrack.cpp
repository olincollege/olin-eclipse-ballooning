// Program to calculate desired orientation of balloon camera based on time (UTC) and GPS data.
// adapted from azimuth/distance calculator: http://cosinekitty.com/compass.html
// eclipse shadow path data from: https://eclipse.gsfc.nasa.gov/SEpath/SEpath2001/SE2023Oct14Apath.html

#include "ShadowTrack.h"
#include "Arduino.h"

// Access csv data from SD card to calculate shadow locaiton from current time
vector<PathCoordinate> load_from_sd()
{
    if (!SD.begin(BUILTIN_SDCARD))
    {
        Serial.println("initialization failed!");
    }

    // Set up data structure
    vector<PathCoordinate> pathPoints;

    File myFile = SD.open("eclipse_path.csv"); // open the file
    if (myFile)
    {
        // Set up temporary values for filling data structure
        String tempString = "";

        int hour = -1;
        int minute = -1;
        double lat = 0.0;
        double lon = 0.0;

        while (myFile.available())
        {
            char character = myFile.read(); // read one character at a time

            if (character == '\n')
            {
                lon = tempString.toFloat();
                tempString = "";

                PathCoordinate point(hour, minute, lat, lon);
                pathPoints.push_back(point);

                hour = -1;
                minute = -1;
                lat = 0.0;
                lon = 0.0;
            }
            else if (character == ',')
            {
                if (hour == -1)
                {
                    hour = tempString.toInt();
                }
                else if (minute == -1)
                {
                    minute = tempString.toInt();
                }
                else if (!lat)
                {
                    lat = tempString.toFloat();
                }
                tempString = "";
            }
            else
            {
                tempString += character;
            }
        }
    }
    else
    {
        Serial.println("error opening test.csv");
    }
    myFile.close();

    return pathPoints;
}

pair<double, double> calculate_lat_lon(int hours, int mins, int seconds)
{
    vector<PathCoordinate> points = load_from_sd();

    double latitude;
    double longitude;
    double rtcMins = 60 * hours + mins + seconds / double(60);
    double pointMins;
    int upperInx = -1;

    // find known csv entries on either side of point
    unsigned int vecSize = points.size();
    for (unsigned int i = 0; i < vecSize; i++)
    {
        pointMins = 60 * points[i].Hours + points[i].Mins;
        if (upperInx != -1)
        {
            break;
        }
        else if (pointMins > rtcMins)
        {
            upperInx = i;
        }
    }

    // set upper and lower points
    // upper extreme
    if (upperInx == -1)
    {
        latitude = points[vecSize - 1].Latitude;
        longitude = points[vecSize - 1].Longitude;
    }
    // lower extreme
    else if (upperInx < 2)
    {
        latitude = points[1].Latitude;
        longitude = points[1].Longitude;
    }
    // within range
    else
    {
        PathCoordinate lower = points[upperInx - 1];
        PathCoordinate upper = points[upperInx];

        // calculate percentage of segment based on time
        double lowerMins = 60 * lower.Hours + lower.Mins;
        double upperMins = 60 * upper.Hours + upper.Mins;

        // calculate new latitude/longitude based on time percentage
        double timeFraction = (rtcMins - lowerMins) / (upperMins - lowerMins);
        latitude = lower.Latitude + timeFraction * (upper.Latitude - lower.Latitude);
        longitude = lower.Longitude + timeFraction * (upper.Longitude - lower.Longitude);
    }

    Serial.print("Shadow Latitude: ");
    Serial.println(latitude);
    Serial.print("Shadow Longitude: ");
    Serial.println(longitude);

    return make_pair(latitude, longitude);
};

// find radius of earth at given latitude (earth is elliptical)
double find_earth_radius(double lat)
{
    double a = 6378137.0; // equatorial radius in meters
    double b = 6356752.3; // polar radius in meters
    double cos_lat = cos(lat);
    double sin_lat = sin(lat);
    double t1 = a * a * cos_lat;
    double t2 = b * b * sin_lat;
    double t3 = a * cos_lat;
    double t4 = b * sin_lat;
    return sqrt((t1 * t1 + t2 * t2) / (t3 * t3 + t4 * t4));
}

// earth is not a sphere -- used in converting between lat/lon to cartesian
double find_geocentric_latitude(double lat)
{
    double e2 = 0.00669437999014;
    double clat = atan((1.0 - e2) * tan(lat));
    return clat;
}

// convert location to x, y, z
Point location_to_point(const Coordinates &coords)
{
    double lat = coords.lat * (M_PI / 180.0);
    double lon = coords.lon * (M_PI / 180.0);
    double radius = find_earth_radius(lat);
    double clat = find_geocentric_latitude(lat);

    double cos_lon = cos(lon);
    double sin_lon = sin(lon);
    double cos_lat = cos(clat);
    double sin_lat = sin(clat);

    double x = radius * cos_lon * cos_lat;
    double y = radius * sin_lon * cos_lat;
    double z = radius * sin_lat;

    double cos_glat = cos(lat);
    double sin_glat = sin(lat);

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

    double a_cos = cos(a_lat);
    double a_sin = sin(a_lat);

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
    double dist = sqrt(dist2);
    return {dx / dist, dy / dist, dz / dist, 1.0};
}

// main function to convert lat/lon/alt to azimuth/elevation of shadow wrt balloon
pair<double, double> calc_angle_to_shadow(const Coordinates &shadow, const Coordinates &balloon)
{
    Point balloon_point = location_to_point(balloon);
    Point shadow_point = location_to_point(shadow);

    Point shadow_rotated = rotate_globe(shadow, balloon, shadow_point.radius, balloon_point.radius);
    double azimuth = 0.0, altitude = 0.0;

    if ((shadow_rotated.z * shadow_rotated.z + shadow_rotated.y * shadow_rotated.y) > 1.0e-6)
    {
        double theta = atan2(shadow_rotated.z, shadow_rotated.y) * 180.0 / M_PI;
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
        altitude = 90.0 - (180.0 / M_PI) * acos(pointing_vector.x * balloon_point.nx + pointing_vector.y * balloon_point.ny + pointing_vector.z * balloon_point.nz);
    }

    return make_pair(azimuth, altitude);
}

pair<double, double> ShadowTrack(int hours, int mins, int seconds, float balloon_lat, float balloon_long, float balloon_elev)
{
    pair<double, double> shadowCoordinates = calculate_lat_lon(hours, mins, seconds);

    Coordinates shadow_coordinates = {shadowCoordinates.first, shadowCoordinates.second, 0};
    Coordinates balloon_coordinates = {balloon_lat, balloon_long, balloon_elev};

    // Return pair of angles: azimuth and altitude of shadow relative to balloon location
    pair<double, double> pointing_angles = calc_angle_to_shadow(shadow_coordinates, balloon_coordinates);
    return pointing_angles;
}
