#include "../include/security.h"

bool Security::check_is_flying()
{
    int32_t lat, lon, alt;
    getCoords(lat,lon,alt);

    //fprintf(stderr, "takeoff diff = %d\n", abs(alt-home_alt-commands[1].content.takeoff.altitude));
    if (abs(alt-home_alt-commands[1].content.takeoff.altitude)<check_is_flying_distance)
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
    fprintf(stderr, "----------------------------------------------------\n");
    int32_t lat, lon, alt;
    getCoords(lat,lon,alt);
    alt-=home_alt;//Чтобы сразу перейти в локальные и больше об этом не думать
    CommandWaypoint drone_possition(lat,lon,alt);
    
    double ret;

    if(!takeoff_land && (ret = check_altitude_is_correct(drone_possition,current_command)) != 0)
    {
        if(alt_timer())
        {
            changeAltitude(current_waypoint.altitude);
            //changeAltitude(commands[current_command].content.waypoint.altitude);
            alt_timer.restart();
        }
        if(ret>kill_altitude)
        {
            setKillSwitch(0);
            fprintf(stderr, "kill alt\n");
            exit(EXIT_SUCCESS);
        }
    }
    if (takeoff_land && abs(alt - current_waypoint.altitude) < check_altitude_is_correct_distance && !at_pause)
    {
        takeoff_land = 0;
    }

    //task: мб поменять, так как надо будет сбрасывать после посадки
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

    fprintf(stderr, "deviation_counter = %d\n", deviation_counter);
    if(!check_speed_is_correct(drone_possition))
    {
        deviation_counter ++;
        if(deviation_counter >= speed_change_counter && velocity_timer())
        {
            changeSpeed(2);
            velocity_timer.restart();
        }
        if(deviation_counter >= speed_kill_counter)
        {
            setKillSwitch(0);
            fprintf(stderr, "kill speed\n");
            exit(EXIT_SUCCESS);
        }
    }
    else
    {
        deviation_counter = 0;
    }

    if((ret = check_no_deviation_from_cource(drone_possition)) != 0)
    {
        if(waypoint_timer())
        {   
            int32_t current_waypoint_lat=current_waypoint.latitude;
            int32_t current_waypoint_lon=current_waypoint.longitude;
            int32_t current_waypoint_alt=current_waypoint.altitude;
            /*
            int32_t current_waypoint_lat=commands[current_command].content.waypoint.latitude;
            int32_t current_waypoint_lon=commands[current_command].content.waypoint.longitude;
            int32_t current_waypoint_alt=commands[current_command].content.waypoint.altitude;
            */
            changeWaypoint(current_waypoint_lat, current_waypoint_lon, current_waypoint_alt);
            waypoint_timer.restart();
        }
        
        if(ret>kill_deviation)
        {
            setKillSwitch(0);
            fprintf(stderr, "kill deviation\n");
            exit(EXIT_SUCCESS);
        }
    }

    update_current_command(drone_possition);
    fprintf(stderr, "current_command = %d\n", current_command);
    fprintf(stderr, "takeoff_land = %d, at_pause = %d\n", takeoff_land, at_pause);
}

double Security::check_altitude_is_correct(const CommandWaypoint& drone_possition,int32_t command_number)
{
    /*if(commands[command_number].type == CommandType::LAND ||commands[command_number].type == CommandType::TAKEOFF || 
    commands[command_number-1].type == CommandType::TAKEOFF)
    {
        return 0;
    }*/
    
    /*CommandWaypoint current_waypoint=commands[current_command].content.waypoint;
    uint32_t prev_waypoint_number = get_prev_waypoint_number(command_number);
    CommandWaypoint prev_waypoint{0,0,0};
    if(prev_waypoint_number == -1)
    {
        if(commands[command_number-1].type == CommandType::LAND)
        {
            prev_waypoint=commands[command_number-1];
               prev_waypoint.altitude-=home_alt;
        }   
        if(commands[command_number-1].type == CommandType::TAKEOFF)
        {
            prev_waypoint.longitude = commands[command_number].content.waypoint.longitude

            prev_waypoint.altitude = commands[command_number-1].content.takeoff.altitude
            
        }

    }
    else
    {
        prev_waypoint=commands[prev_waypoint_number];
    }
    */
    //fprintf(stderr, "%d, %d, %d - cur, %d, %d, %d - prev\n", current_waypoint.latitude, current_waypoint.longitude,current_waypoint.altitude, trajectory.points[number_cur_waypoint-1].latitude, trajectory.points[number_cur_waypoint-1].longitude, trajectory.points[number_cur_waypoint-1].altitude);

    Line line = Line(current_waypoint, trajectory.points[number_cur_waypoint - 1]);
    double ro = fabs((drone_possition - line.r0, g * line.a)) / abs(g*line.a); // FIXME when g*a == 0
    double dist = distance(drone_possition, line);
    double d = sqrt(dist*dist - ro*ro);
    fprintf(stderr, "altitude_distance = %f\n", d);
    if( d < check_altitude_is_correct_distance)
    {
        return 0;   
    }
    else
    {
        return d;
    }
    //return abs(drone_possition.altitude-home_alt-command_altitude)<check_altitude_is_correct_distance;
}

bool Security::check_set_servo_is_nearby(const CommandWaypoint& drone_possition)
{
    if(number_set_servo_waypoint == 0)
        return false;

    CommandWaypoint command = commands[number_set_servo_waypoint].content.waypoint;
    //command.altitude=drone_possition.altitude;//Нужно, чтобы не учитывать разницу высот
    //fprintf(stderr, "servo_distance = %f\n", distance(command,drone_possition));
    if(distance(command,drone_possition)<check_servo_is_nearby_distance)
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
        //fprintf(stderr, "current_speed1 = %f\n", abs(current_pos-prev_prev_pos)*0.5/t);
        fprintf(stderr, "current_speed2 = %f\n", abs(3*current_pos-4*prev_pos+prev_prev_pos)*0.5/t);
        if(abs(3*current_pos-4*prev_pos+prev_prev_pos)*0.5/t>max_speed)
        {
            //prev_prev_pos=Vector3D();
            //prev_pos=Vector3D();
            //current_pos=Vector3D();
            return false;
        }
        return true;
    }
 }

double Security::check_no_deviation_from_cource(const CommandWaypoint& drone_possition)
{
    double ret;
    //По умолчанию считаем, что при спуске это верно
    // if(commands[current_command].type == CommandType::TAKEOFF || commands[current_command].type == CommandType::LAND 
    // || commands[current_command-1].type == CommandType::TAKEOFF )
        // return 0;
    CommandWaypoint prev_waypoint=trajectory.points[number_cur_waypoint-1];

    // CommandWaypoint current_waypoint=commands[current_command].content.waypoint;
    // current_waypoint.altitude=drone_possition.altitude;//Нужно, чтобы не учитывать разницу высот
    // uint32_t prev_waypoint_number;
    

    // if(commands[current_command-1].type == CommandType::TAKEOFF)
    //     prev_waypoint_number = 0;

    // if(commands[current_command-1].type == CommandType::SET_SERVO)
    //     prev_waypoint_number = number_set_servo_waypoint;

    // if(commands[current_command-1].type == CommandType::WAYPOINT)
    //     prev_waypoint_number = current_command-1;

    //CommandWaypoint prev_waypoint(commands[prev_waypoint_number].content.waypoint.latitude,
    //commands[prev_waypoint_number].content.waypoint.longitude,current_waypoint.altitude);//Нужно, чтобы не учитывать разницу высот
    // CommandWaypoint prev_waypoint=commands[prev_waypoint_number].content.waypoint;//3D

    // fprintf(stderr, "where is deviation = %f\n", distance(drone_possition,Line(current_waypoint,prev_waypoint)));
    //return distance(drone_possition,Line(current_waypoint,prev_waypoint)) < check_no_deviation_from_cource_distance;
    if((ret = distance(drone_possition,Line(current_waypoint,prev_waypoint))) < check_no_deviation_from_cource_distance)
    {
        fprintf(stderr, "ret - %f\n", ret);
        return 0;
    }
    else
    {
        fprintf(stderr, "ret - %f\n", ret);
        return ret;
    }
}

void Security::update_current_command(const CommandWaypoint& drone_possition)
{
    if (!at_pause)
        block_update = 0;
    if(current_command+1<commands.size() && !block_update)
    {
        CommandWaypoint command = commands[current_command].content.waypoint;
        switch (commands[current_command].type)
        {
            case TAKEOFF:
                command.latitude  = commands[current_command - 1].content.waypoint.latitude;
                command.longitude = commands[current_command - 1].content.waypoint.longitude;
                command.altitude  = commands[current_command].content.takeoff.altitude;
                break;

            case LAND:
                command.altitude = 0;
                break; 

            case WAYPOINT:
                break;


            default:
                fprintf(stderr, "Error: HOME or SET_SERVO is nexr command\n");
        }

        int32_t step = 0;

        if(commands[current_command+1].type == CommandType::SET_SERVO)
        {
            step = 2;
        }
        else//Т.е. это Waypoint или Land
        {
            step = 1;
        }
        
        //command.altitude=drone_possition.altitude;//Нужно, чтобы не учитывать разницу высот


        fprintf(stderr, "dinstane to current_command = %f\n", distance(drone_possition,command));
        fprintf(stderr, "dinstane to current_waypoint = %f\n", distance(drone_possition,current_waypoint));
        if(distance(drone_possition,command)< current_command_update_distance)
        {
                current_command += step;
                if (commands[current_command].type == LAND)
                    takeoff_land = 1;
        }
        if(distance(current_waypoint, drone_possition) < current_command_update_distance)
        {
            number_cur_waypoint++;
            current_waypoint = trajectory.points[number_cur_waypoint];        
            if (at_pause)
                block_update = 1;
        }

    }
}

uint32_t Security::get_prev_waypoint_number (uint32_t command)
{
    uint32_t prev_waypoint_number = -1;

    if(commands[command-1].type == CommandType::SET_SERVO)
        prev_waypoint_number = number_set_servo_waypoint;

    if(commands[command-1].type == CommandType::WAYPOINT)
        prev_waypoint_number = command-1;

    return prev_waypoint_number;
}

void Security::at_pause_flight()
{
    fprintf(stderr, "Flight pause\n");
    int32_t lat, lon, alt;
    getCoords(lat, lon, alt);
    //FFFFFFFFFFFFF
    trajectory.points.insert(trajectory.points.begin()+number_cur_waypoint, CommandWaypoint(lat, lon, alt - home_alt));
    trajectory.points.insert(trajectory.points.begin()+number_cur_waypoint+1, CommandWaypoint(lat, lon, 0));
    trajectory.points.insert(trajectory.points.begin()+number_cur_waypoint+2, CommandWaypoint(lat, lon, alt - home_alt));
    takeoff_land = 1;
    current_waypoint = trajectory.points[++number_cur_waypoint];
}

/*
CommandWaypoint Security::convert_to_waypoint(const MissionCommand& commmand)
{
    if(commmand.type == CommandType::WAYPOINT)
    {
        return commmand;
    }

    CommandWaypoint ret(0,0,0);
    if(commmand.type == CommandType::LAND)
    {
        ret.longitude = commmand.content.waypoint.longitude;
        ret.latitude = commmand.content.waypoint.latitude;
        ret.altitude = commmand.content.waypoint.altitude - home_alt;
        return ret;
    }
    
    if(commmand.type == CommandType::TAKEOFF)
    {
        ret.longitude = commmand.content.waypoint.longitude;
        ret.latitude = commmand.content.waypoint.latitude;
        ret.altitude = commmand.content.waypoint.altitude;
        return ret;
    }
}*/