#ifndef SECURITY_H
#define SECURITY_H

#include "../include/mission.h"
#include "../../shared/include/initialization_interface.h"
#include "../../shared/include/ipc_messages_initialization.h"
#include "../../shared/include/ipc_messages_autopilot_connector.h"
#include "../../shared/include/ipc_messages_credential_manager.h"
#include "../../shared/include/ipc_messages_navigation_system.h"
#include "../../shared/include/ipc_messages_periphery_controller.h"
#include "../../shared/include/ipc_messages_server_connector.h"

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>

#include "../include/coords.h"


#include <ctime>

class Timer
{
  clock_t delay;
  clock_t prev;

public: 
  Timer (unsigned int milliseconds) 
    : delay {CLOCKS_PER_SEC * milliseconds / 1000}
    , prev {clock()} 
  {}

  bool operator() ()
  {
    fprintf(stderr, "check timer, delay = %d\n", delay);
    if ((clock() - prev) > delay)
    {
      prev = clock();
      return true;
    }
    else 
      return false;
  }

  void restart()
  {
    fprintf(stderr, "timer restart, delay = %d\n", delay);
    prev = clock();
  }
};


constexpr int32_t check_is_flying_distance = 20;


constexpr int32_t check_altitude_is_correct_distance = 30;
constexpr int32_t check_servo_is_nearby_distance = 50;
constexpr int32_t max_speed = 225;
constexpr int32_t check_no_deviation_from_cource_distance = 200;
constexpr int32_t current_command_update_distance = 175;

constexpr int32_t alt_delay = 3000;
constexpr int32_t velocity_delay = 300;
constexpr int32_t waypoint_delay = 3000;

constexpr int32_t kill_altitude = 100;
constexpr int32_t kill_deviation = 250;

constexpr int32_t speed_change_counter = 2;
constexpr int32_t speed_kill_counter = 50;

class Security
{
public:
    std::vector<MissionCommand> commands;
    int32_t home_alt;
    uint32_t number_set_servo_waypoint = 0; //Если оно останется равном 0, значит не надо сбрасывать груз
    uint32_t current_command = 1;
    bool takeoff_land = 1;
    bool at_pause = 0;
    double t;

    Vector3D current_pos;
    Vector3D prev_pos;
    Vector3D prev_prev_pos;
    bool cargo_open = false;
    
    
    Timer alt_timer {alt_delay};
    Timer velocity_timer {velocity_delay};
    Timer waypoint_timer {waypoint_delay};

    int32_t deviation_counter = 0;
    
    Trajectory trajectory;
    int32_t number_cur_waypoint = 1;
    CommandWaypoint current_waypoint;

    const Vector3D g;
    
    Security(const std::vector<MissionCommand>& other_commands,const double t_)
            : commands{other_commands}
            , t{t_}
            , trajectory {other_commands}
            , current_waypoint {trajectory.points[number_cur_waypoint]}
            , g{Vector3D(Vector3D(other_commands[0].content.waypoint).x/(A*A), Vector3D(other_commands[0].content.waypoint).y/(A*A), Vector3D(other_commands[0].content.waypoint).z/(A*sqrt(1-E*E)*A*sqrt(1-E*E)))}
    {
      int32_t a;
      int32_t b;

      trajectory.print_points();

      getCoords(a,b,home_alt);
      commands[0].content.waypoint.latitude = a;
      commands[0].content.waypoint.longitude = b;
      commands[0].content.waypoint.altitude = home_alt;

      char message[512] = {0};
      char response[512] = {0};

      trajectory.points[0].latitude = a;
      trajectory.points[0].longitude = b;

      snprintf(message, 512, "Changed trajectory point 0 to coords: latitude = %d, logitude = %d, altitude = %d", a, b, trajectory.points[0].altitude);
      sendLogMessage(message, response, "Fail in sending changed mission point");

      trajectory.points[1].latitude = a;
      trajectory.points[1].longitude = b; 

      snprintf(message, 512, "Changed trajectory point 1 to coords: latitude = %d, logitude = %d, altitude = %d", a, b, trajectory.points[0].altitude);
      sendLogMessage(message, response, "Fail in sending changed mission point");


      current_waypoint = trajectory.points[1];

      // trajectory.print_points();

      for(uint i=0; i<other_commands.size(); i++)
      {
           if(other_commands[i].type == CommandType::SET_SERVO)
           {
              number_set_servo_waypoint = i-1;
              break;
          }
      }
    }

    bool check_is_flying();

    void tick();

    void at_pause_flight();

private:

    bool block_update = 0;

    uint32_t get_prev_waypoint_number (uint32_t command_number);//Возвращает -1, если не получается найти 

    double check_altitude_is_correct(const CommandWaypoint& drone_possition, int32_t command_number);//Land и takeoff или до этого takeoff всегда true

    bool check_set_servo_is_nearby(const CommandWaypoint& drone_possition);

    bool check_speed_is_correct(const CommandWaypoint& drone_possition);

    double check_no_deviation_from_cource(const CommandWaypoint& drone_possition);//Для Land и takeoff всегда true

    void update_current_command(const CommandWaypoint& drone_possition);

    CommandWaypoint convert_to_waypoint(const MissionCommand& commmand);

};




#endif // SECURITY
