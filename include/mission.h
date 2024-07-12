#pragma once

#include "../../shared/include/initialization_interface.h"
#include "../../shared/include/ipc_messages_initialization.h"
#include "../../shared/include/ipc_messages_autopilot_connector.h"
#include "../../shared/include/ipc_messages_credential_manager.h"
#include "../../shared/include/ipc_messages_navigation_system.h"
#include "../../shared/include/ipc_messages_periphery_controller.h"
#include "../../shared/include/ipc_messages_server_connector.h"


#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vector>

int sendLogMessage(char* input, char* response, char* errorMessage);

enum CommandType {
    HOME,
    TAKEOFF,
    WAYPOINT,
    LAND,
    SET_SERVO
};

struct CommandTakeoff {
    int32_t altitude;

    CommandTakeoff(int32_t alt) {
        altitude = alt;
    }
};

struct CommandWaypoint {
    int32_t latitude;
    int32_t longitude;
    int32_t altitude;

    CommandWaypoint(int32_t lat, int32_t lng, int32_t alt) {
        latitude = lat;
        longitude = lng;
        altitude = alt;
    }
};



struct CommandServo {
    int32_t number;
    int32_t pwm;

    CommandServo(int32_t num, int32_t pwm_) {
        number = num;
        pwm = pwm_;
    }
};

union CommandContent {
    CommandTakeoff takeoff;
    CommandWaypoint waypoint;
    CommandServo servo;
};

struct MissionCommand {
    CommandType type;
    CommandContent content;
};

int parseMission(char* response);
void printMission();

std::vector<CommandWaypoint> get_command_waypoints();

std::vector<MissionCommand> get_commands();


int32_t get_home_altitude();