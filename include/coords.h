#ifndef COORDS_H
#define COORDS_H

#include "../include/mission.h"
#include "../../shared/include/initialization_interface.h"
#include "../../shared/include/ipc_messages_initialization.h"
#include "../../shared/include/ipc_messages_autopilot_connector.h"
#include "../../shared/include/ipc_messages_credential_manager.h"
#include "../../shared/include/ipc_messages_navigation_system.h"
#include "../../shared/include/ipc_messages_periphery_controller.h"
#include "../../shared/include/ipc_messages_server_connector.h"

#include <cmath>
#include <stdexcept>


constexpr int64_t A = 637813700; 	//большая полуось Земли в см
constexpr double  F = 1/298.257223563; 	//Сплюснутость
constexpr double  E = std::sqrt(F*(2-F));

struct Vector3D
{
    double x;
    double y;
    double z;

    Vector3D()
    {
        x=0;
        y=0;
        z=0;
    }

    Vector3D(double x_, double y_, double z_): x{x_}, y{y_}, z{z_} {}
    Vector3D(const int32_t latit, const int32_t longit, const int32_t h)
    {
        double latitude = M_PI*latit/(180*1e7);
        double longitude = M_PI*longit/(180*1e7);
        double nu = A /(std::sqrt(1 - std::pow(E*std::sin(latitude),2)));	// радиус кривизны в главном вертикале

        x = (nu + h)*std::cos(latitude)*std::cos(longitude);
        //fprintf(stderr, "x = %f\n", x);
        y = (nu + h)*std::cos(latitude)*std::sin(longitude);
        //fprintf(stderr, "y = %f\n", y);
        z = ((1 - E*E)*nu + h)*std::sin(latitude);
        //fprintf(stderr, "z = %f\n", z);
    }

    Vector3D(const CommandWaypoint& p): Vector3D(p.latitude, p.longitude, p.altitude) {}

    double abs() const;

    Vector3D& operator += (const Vector3D& other)
    {
        x += other.x;
	    y += other.y;
	    z += other.z;
	    return *this;
    }
    
    Vector3D& operator -= (const Vector3D& other)
    {
        x -= other.x;
	    y -= other.y;
	    z -= other.z;
	    return *this;
    }
    
    Vector3D& operator *= (const double l)
    {
        x *= l;
	    y *= l;
	    z *= l;
	    return *this;
    }

    Vector3D& operator *= (const Vector3D& other)
    {
        double a = y*other.z - z*other.y;
	    double b = z*other.x - x*other.z;
        double c = x*other.y - y*other.x;
	x = a;
	y = b;
	z = c;
	return *this;
    }
};


Vector3D operator + (const Vector3D& a, const Vector3D& b);
Vector3D operator * (double l, const Vector3D& vec3D);
Vector3D operator * (const Vector3D& vec3D, double l);
Vector3D operator - (const Vector3D& b);
Vector3D operator - (const Vector3D& a, const Vector3D& b);

bool operator == (const Vector3D& a, const Vector3D& b);

double operator , (const Vector3D& a, const Vector3D& b);   // Dot product
Vector3D operator * (const Vector3D& a, const Vector3D& b); // Cross product

double abs (const Vector3D& a);

double distance (const Vector3D& a, const Vector3D& b);
double distance (const CommandWaypoint& a, const CommandWaypoint& b);


struct Line
{
    Vector3D r0;
    Vector3D a;

    Line (const Vector3D& r1, const Vector3D& r2): r0{r1}, a{r2 - r1} 
    {
        if (r1 == r2)
	        throw std::runtime_error("Points must be different to create line");
    }
    Line (const CommandWaypoint& r1, const CommandWaypoint& r2): Line(Vector3D(r1), Vector3D(r2)) {}
};

bool operator== (const Line& l1, const Line& l2);

double distance (const Vector3D& M, const Line& l);
double distance (const CommandWaypoint& M, const Line& l);


struct Trajectory
{
    std::vector<CommandWaypoint> points;

    Trajectory(const std::vector<MissionCommand>& other_commands) : points{}
    {
        char message[512] = {0};
        char response[512] = {0};
        for(uint i=0; i<other_commands.size(); i++)
        {
            //fprintf(stderr, "ALL lat=%d lon=%d alt=%d\n", other_commands[i].content.waypoint.latitude, other_commands[i].content.waypoint.longitude, other_commands[i].content.waypoint.altitude);
            if(other_commands[i].type == CommandType::HOME || other_commands[i].type == CommandType::LAND)
            {
                points.emplace_back(other_commands[i].content.waypoint.latitude, other_commands[i].content.waypoint.longitude, 0);
                int ter = (other_commands[i].type == CommandType::HOME) 
                          ? snprintf(message, 512, "Point %d is HOME with coords: latitude = %d, logitude = %d, altitude = %d\n", i, points[i].latitude, points[i].longitude, 0)
                          : snprintf(message, 512, "Point %d is LAND with coords: latitude = %d, logitude = %d, altitude = %d\n", i, points[i].latitude, points[i].longitude, 0);
                //fprintf(stderr, "HOME lat=%d lon=%d alt=%d\n", points[points.size() - 1].latitude, points[points.size() - 1].longitude, points[points.size() - 1].altitude);
            }
            else if(other_commands[i].type == CommandType::TAKEOFF)
            {
                points.emplace_back( other_commands[i-1].content.waypoint.latitude
                                   , other_commands[i-1].content.waypoint.longitude
                                   , other_commands[i].content.takeoff.altitude);
                snprintf(message, 512, "Point %d is TAKEOFF with coords: latitude = %d, logitude = %d, altitude = %d\n", i, points[i].latitude, points[i].longitude, points[i].altitude);
                //fprintf(stderr, "TAKEOFF lat=%d lon=%d alt=%d\n", points[points.size() - 1].latitude, points[points.size() - 1].longitude, points[points.size() - 1].altitude);
            }
            else if(other_commands[i].type == CommandType::WAYPOINT)
            {
                points.push_back(other_commands[i].content.waypoint);
                snprintf(message, 512, "Point %d is WAYPOINT with coords: latitude = %d, logitude = %d, altitude = %d\n", i, points[i].latitude, points[i].longitude, points[i].altitude);
                //fprintf(stderr, "WAYPOINT lat=%d lon=%d alt=%d\n", points[points.size() - 1].latitude, points[points.size() - 1].longitude, points[points.size() - 1].altitude);
            }
            else if(other_commands[i].type == CommandType::SET_SERVO)
            {
                snprintf(message, 512, "Point %d is SERVO with number = %d, pwm = %d\n", i, other_commands[i].content.servo.number, other_commands[i].content.servo.pwm);
            }
            sendLogMessage(message, response, "Fail in sending mission point\n");
        }
    }
    void print_points()
    {
        for(uint i = 0; i<points.size(); i++)
        {
            fprintf(stderr, "lon - %d lat - %d alt - %d\n", points[i].longitude, points[i].latitude, points[i].altitude);
        }
    }
};


#endif // COORDS
