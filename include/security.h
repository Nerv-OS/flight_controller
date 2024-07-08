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


class Security
{
public:
    std::vector<MissionCommand> commands;
    int32_t home_alt;
    uint32_t number_set_servo_waypoint = 0; //Если оно останется равном 0, значит не надо сбрасывать груз
    uint32_t current_command = 2;
    double t;

    Vector3D current_pos;
    Vector3D prev_pos;
    Vector3D prev_prev_pos;
    bool cargo_open = false;
    
    


    Security(const std::vector<MissionCommand>& other_commands,const double t_): commands{other_commands}, home_alt{other_commands[0].content.waypoint.altitude}, t{t_}
    {
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

    

private:

    bool check_altitude_is_correct(const CommandWaypoint& drone_possition, int32_t command_number);//Может принимать Takeoff и Waypoint, а для Land всегда true

    bool check_set_servo_is_nearby(const CommandWaypoint& drone_possition);

    bool check_speed_is_correct(const CommandWaypoint& drone_possition);

    bool check_no_deviation_from_cource(const CommandWaypoint& drone_possition);//Для Land всегда true

    void update_current_command(const CommandWaypoint& drone_possition);

};


#endif // SECURITY
