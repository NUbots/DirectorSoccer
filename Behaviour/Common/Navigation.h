/*! @file Navigation.h
    @brief centrally defined navigation commands for new behaviours.

    @author Josiah Walker

 Copyright (c) 2012 Josiah Walker

 This file is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This file is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with NUbot.  If not, see <http://www.gnu.org/licenses/>.
 */



#ifndef NAVIGATION_H
#define NAVIGATION_H

#include "Infrastructure/NUBlackboard.h"
#include "Infrastructure/FieldObjects/FieldObjects.h"
#include "Infrastructure/FieldObjects/Object.h"
#include "Infrastructure/GameInformation/GameInformation.h"
#include "Infrastructure/Jobs/MotionJobs/WalkJob.h"
#include "Tools/Math/General.h"



class Navigation {
private:
    //classification so we know which foot to line the ball up with
    enum DIRECTION 
        {
        LEFT = 1,
        BOTH = 0,
        RIGHT = -1;
        };
    enum WALK_TYPE 
        {
        USELASTCOMMAND = -1,
        GOTOPOINT = 0,
        GOTOOBJECT = 1,
        GOTOBALL = 2
        };
    

//XXX: load all these from central config ----------------------------------------
    //load these values from walk config
    float m_turn_speed = 0.4;
    float m_walk_speed = 0.9;
    float m_feet_separation = 14.0;
    
    //load from robot model
    float m_foot_size = 10.0;
    
    //timers for starting turning and walking
    double m_walk_start_time = 0.2;
    double m_walk_turn_time = 0.2;
    
    //walk accel/deccel controls
    double m_acceleration_time = 0.2;
    float m_acceleration_fraction = 0.5;
    
    //approach speeds
    float m_close_approach_speed = 0.2;
    float m_close_approach_distance = 30.0;
    float m_mid_approach_speed = 0.6;
    float m_mid_approach_distance = 60.0;
    
    //turning values
    float m_turn_deviation = 0.1;
    float m_turn_speed = 0.4;
    
    //hystereses
    float m_distance_hysteresis = 10.0;
    float m_turning_hysteresis = 0.1;
    float m_position_hysteresis = 30.0;
    
    //ball lineup
    vector<float> m_ball_approach_angle;
    vector<int> m_ball_kick_foot;
    float m_ball_lineup_distance = 15.0;
    int m_ball_lineup_min_distance = 12.0;
    
    //extra config options
    bool m_use_localisation_avoidance = false;
    float m_assumed_obstacle_width = 25.0;
    float m_avoid_distance = 50.0;
//END config variables section-------------------------------------------------------------------
    
    //hysteresis variables
    int m_turning;
    int m_distance_increment;
    int m_approach_distance;
    
    //info for the current walk
    Object* current_object;
    vector<float> current_point;
    int current_command;
    vector<float> current_walk_command;
    float current_heading;
    
    /*! @brief Given a distance, and relative bearing and heading, returns a new walk command based on the navigation parameters
     */
    vector<float> generateWalk(float distance, float relative_bearing, float relative_heading, bool avoidObstacles = true);
    
    /*! @brief Returns a new direction (bearing) to move that avoids all obstacles
     */
    float avoidObstacles(const vector<float> position, float relative_bearing);
    
    /*! @brief Returns a new direction (bearing) to move that aligns the designated foot with the ball
     */
    float alignFoot(float distance, float relative_bearing, int use_foot);
    
    /*! @brief Updates the configuration values of the Navigation module.
     */
    void updateConfiguration();
    
    /*! @brief reset hystereses on walk command type change.
     */
    void resetHystereses();
    
public:
    
    int getCurrentCommand() {
        return currentCommand;
    }
    
    bool isStopped() {
        const float epsilon = 0.05;
        return current_walk_command[0] < epsilon and MathGeneral::abs(current_walk_command[1]) < epsilon and MathGeneral::abs(current_walk_command[2]) < epsilon
    }
        
    /*! @brief Go to a point and face a heading. Returned vector is walk command vector.
     */
    vector<float> goToPoint(float distance, float relative_bearing, float relative_heading);
    
    /*! @brief Go to a point and face a heading. Returned vector is walk command vector.
     */
    vector<float> goToPoint(Object fieldObject, float heading);
    
    /*! @brief Go to a point and face a heading. Returned vector is walk command vector.
     */
    vector<float> goToPoint(const vector<float> point);
    
    /*! @brief Approach the ball with a good angle to kick from. Returned vector is walk command vector.
     */
    vector<float> goToBall(Object kickTarget = NULL);
    
    /*! @brief Update the goto calculations and send the walk commands (if actions are not active).
     */
    void update();
    
    static Navigation* getInstance();
    
};

#endif

