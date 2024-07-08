#include "../include/security.h"

bool Security::check_is_flying()
{
    int32_t lat, lon, alt;
    getCoords(lat,lon,alt);

    if (abs(alt-home_alt-commands[1].content.takeoff.altitude)<50)
    {
        return true;
    }
    else
    {
        //Но по-хорошему нужны ещё проверки, чтобы дрон не взломали при взлёте
        return false;
    }
}



void Security::tick()
{
    int32_t lat, lon, alt;
    getCoords(lat,lon,alt);
    CommandWaypoint drone_possition(lat,lon,alt);

    if(!check_altitude_is_correct(drone_possition,current_command))
    {
        changeAltitude(commands[current_command].content.waypoint.altitude);
        sleep(1);
    }

    if(check_set_servo_is_nearby(drone_possition))
    {
        if(!cargo_open)
        {
            cargo_open=true;
            setCargoLock(1);
            //fprintf(stderr, "Cargo open\n");
        }
    }
    else
    {
        if(cargo_open)
        {
            cargo_open=false;
            setCargoLock(0);
            //fprintf(stderr, "Cargo close\n");
        }
    }

    if(!check_speed_is_correct(drone_possition))
        changeSpeed(2);
    

    if(!check_no_deviation_from_cource(drone_possition))
    {
        int32_t current_waypoint_lat=commands[current_command].content.waypoint.latitude;
        int32_t current_waypoint_lon=commands[current_command].content.waypoint.longitude;
        int32_t current_waypoint_alt=commands[current_command].content.waypoint.altitude;
        changeWaypoint(current_waypoint_lat, current_waypoint_lon, current_waypoint_alt);
        sleep(1);
    }

    update_current_command(drone_possition);
    //fprintf(stderr, "current_command = %d\n", current_command);
}

bool Security::check_altitude_is_correct(const CommandWaypoint& drone_possition,int32_t command_number)
{
    int32_t command_altitude;
    if(commands[command_number].type == CommandType::TAKEOFF)
    {
        command_altitude=commands[command_number].content.takeoff.altitude;
    }

    if(commands[command_number].type == CommandType::WAYPOINT)
    {
        command_altitude=commands[command_number].content.waypoint.altitude;
    }
    if(commands[command_number].type == CommandType::LAND)
    {
        return true;
    }


    return abs(drone_possition.altitude-home_alt-command_altitude)<50;
}

bool Security::check_set_servo_is_nearby(const CommandWaypoint& drone_possition)
{
    if(number_set_servo_waypoint==0)
        return false;

    CommandWaypoint command = commands[number_set_servo_waypoint].content.waypoint;
    command.altitude=drone_possition.altitude;//Нужно, чтобы не учитывать разницу высот
    if(distance(command,drone_possition)<200)
        return true;
    return false;
}

 bool Security::check_speed_is_correct(const CommandWaypoint& drone_possition)
 {
    prev_prev_pos=prev_pos;
    prev_pos = current_pos;
    current_pos=Vector3D(drone_possition);
    if(prev_prev_pos == Vector3D())
    {
        return true;
    }
    else
    {
        //fprintf(stderr, "current_speed = %f\n", abs(current_pos-prev_prev_pos)*0.5/t);
        if(abs(current_pos-prev_prev_pos)*0.5/t>350)
        {
            prev_prev_pos=Vector3D();
            prev_pos=Vector3D();
            current_pos=Vector3D();
            return false;
        }
        return true;
    }
 }

bool Security::check_no_deviation_from_cource(const CommandWaypoint& drone_possition)
{
    //По умолчанию считаем, что при спуске это верно
    if(current_command == commands.size()-1)
        return true;


    CommandWaypoint current_waypoint=commands[current_command].content.waypoint;
    current_waypoint.altitude=drone_possition.altitude;//Нужно, чтобы не учитывать разницу высот
    uint32_t prev_waypoint_number;
    

    if(commands[current_command-1].type == CommandType::TAKEOFF)
        prev_waypoint_number=0;

    if(commands[current_command-1].type == CommandType::SET_SERVO)
        prev_waypoint_number = number_set_servo_waypoint;

    if(commands[current_command-1].type == CommandType::WAYPOINT)
        prev_waypoint_number = current_command-1;

    CommandWaypoint prev_waypoint(commands[prev_waypoint_number].content.waypoint.latitude,
    commands[prev_waypoint_number].content.waypoint.longitude,current_waypoint.altitude);//Нужно, чтобы не учитывать разницу высот


    //fprintf(stderr, "deviation = %f\n", distance(drone_possition,Line(current_waypoint,prev_waypoint)));
    return distance(drone_possition,Line(current_waypoint,prev_waypoint)) < 500;
}

void Security::update_current_command(const CommandWaypoint& drone_possition)
{
    if(current_command+1<commands.size())
    {
        CommandWaypoint command = commands[current_command].content.waypoint;
        int32_t step = 0;

        if(commands[current_command+1].type == CommandType::SET_SERVO)
        {
            step = 2;
        }
        else//Т.е. это Waypoint или Land
        {
            step = 1;
        }
        
        command.altitude=drone_possition.altitude;//Нужно, чтобы не учитывать разницу высот

        if(distance(drone_possition,command)<100)
                current_command+=step;
    }
}