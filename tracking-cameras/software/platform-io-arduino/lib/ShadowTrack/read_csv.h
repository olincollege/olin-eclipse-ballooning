#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <cstring>
#include <vector>
#include "ShadowTrack.h"
using namespace std;

// struct PathCoordinate
// {
// public:
//     PathCoordinate(
//         int hours,
//         int mins,
//         double latitude,
//         double longitude)
//     {
//         Hours = hours;
//         Mins = mins;
//         Latitude = latitude;
//         Longitude = longitude;
//     }

//     // void display()
//     // {
//     //     cout << "Hours: " << Hours << endl;
//     //     cout << "Minutes: " << Mins << endl;
//     //     cout << "Latitude: " << Latitude << endl;
//     //     cout << "Longitude: " << Longitude << endl;
//     //     cout << endl;
//     // }

//     int Hours;
//     int Mins;
//     double Latitude;
//     double Longitude;
// };

vector<PathCoordinate> load_csv_data()
{

    ifstream file;
    file.open("eclipse_path.csv");

    vector<PathCoordinate> pathPoints;

    string line;
    while (getline(file, line))
    {
        int hours;
        int mins;
        double latitude;
        double longitude;

        string tempString;
        stringstream inputString(line);

        getline(inputString, tempString, ',');
        hours = atoi(tempString.c_str());
        getline(inputString, tempString, ',');
        mins = atoi(tempString.c_str());
        getline(inputString, tempString, ',');
        latitude = atof(tempString.c_str());
        getline(inputString, tempString, ',');
        longitude = atof(tempString.c_str());

        PathCoordinate point(hours, mins, latitude, longitude);
        pathPoints.push_back(point);
    }

    file.close();
    return pathPoints;
};

pair<double, double> calculate_lat_lon(int hours, int mins, int seconds)
{
    vector<PathCoordinate> points = load_csv_data();

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

    return make_pair(latitude, longitude);
};
