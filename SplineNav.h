/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

//////////////////////////////////////////////////////////
// SplineNav Spline Navigation Controller for ArduCopter 3.0
// Version 0.1
// Created by David Dewey <david@mavbot.com>
// 
// License: GNU 2.1 or later (See diydrones/ardupilot/COPYING.txt on GitHub)
//////////////////////////////////////////////////////////

#ifndef SPLINENAV_H
#define SPLINENAV_H

// acceleration towards full speed in cm/s^2; and max acceleration going around curves
#define SPLINE_ACCELERATION 250.0

// set to false if you don't want a spline loop
#define SPLINE_LOOP true

class SplineNav
{
public:

    // spline_start -- start the spline WP navigation
    void start(const Vector3f current_position);

    // update_spline - spline position controller's main call which in turn
    //calls loiter controller with updated target position
    void update(float dt);

protected:

    // current target speed for motion along spline curve
    float spline_target_speed;

    // current t parameter on spline segment from 0.0 to 1.0
    float spline_t;
    
    // max climb rate
    float spline_max_climb;
    
    // max descent rate (always a negative number)
    float spline_max_descent;
    
    // max wp speed
    float spline_max_speed;

    // index of next command to read from storage
    int spline_next_cmd_index;
    
    // some helpful vectors
    Vector3f spline_start_point;
    Vector3f spline_p0;
    Vector3f spline_p1;
    Vector3f spline_p2;
    Vector3f spline_p0_prime;
    Vector3f spline_p1_prime;
    Vector3f spline_a;
    Vector3f spline_b;
    Vector3f spline_target;
    Vector3f spline_derivative;

    // perform initialization in preparation for the new spline segment
    void initialize_spline_segment();

    // continue to the next spline segment
    void next_spline_segment();

    // get the next spline waypoint
    Vector3f next_spline_waypoint();

    //evaluate Catmull-Rom spline formula at point t
    Vector3f evaluate_spline(float t) const {
        // spline t square and cube
        float t_sq = t * t;
        float t_cu = t_sq * t;
        return spline_a * t_cu + spline_b * t_sq + spline_p0_prime * t + spline_p0;
    }

    //evaluate Catmull-Rom spline derivative at point t
    Vector3f evaluate_spline_derivative(float t) const {
        float t_sq = t * t;
        return spline_a*3.0*t_sq + spline_b*2.0*t + spline_p0_prime;
    }

    //evaluate Catmull-Rom spline 2nd derivative length at point t
    float spline_2nd_derivative_length(float t) const {
        return (spline_a*6.0*t + spline_b*2.0).length();
    }

};
#endif  // SPLINENAV_H
