/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

//////////////////////////////////////////////////////////
// SplineNav Spline Navigation Controller for ArduCopter 3.0
// Version 0.1
// Created by David Dewey <david@mavbot.com>
// 
// License: GNU 2.1 or later (See diydrones/ardupilot/COPYING.txt on GitHub)
//////////////////////////////////////////////////////////

#include "SplineNav.h"

// start -- begin the spline WP navigation
void SplineNav::start(const Vector3f current_position)
{
    spline_t = 0.0;
    spline_next_cmd_index = 1;

    // set spline p0, p1, p2, and p0 prime
    spline_p0 = spline_start_point = current_position;
    spline_p1 = next_spline_waypoint();
    spline_p2 = next_spline_waypoint();
    spline_p0_prime = spline_p1 - spline_p0;

    // initialise other variables
    spline_target_speed = 0.0;
    spline_max_speed = wp_nav.get_horizontal_velocity();
    spline_max_climb = wp_nav.get_climb_velocity();
    spline_max_descent = -wp_nav.get_descent_velocity();
    initialize_spline_segment();

    // allow spline curve z to control climb and decent
    set_throttle_mode(THROTTLE_AUTO);
}

// update - spline position controller's main call which in turn
// calls loiter controller with updated target position
void SplineNav::update(float dt)
{
    // accelerate gradually to our WP velocity
    spline_target_speed += dt * SPLINE_ACCELERATION;
    if (spline_target_speed > spline_max_speed) spline_target_speed = spline_max_speed;
    
    // compute next target for loiter controller
    spline_target = evaluate_spline(spline_t);
    
    // compute curve derivative at next point
    spline_derivative = evaluate_spline_derivative(spline_t);
    float spline_derivative_length = spline_derivative.length();
    
    float t_speed = 1.0;
    // avoid divide by zero
    if (spline_derivative_length > 0.0) 
        // factor to scale the velocity as we move along the spline curve
        t_speed = spline_target_speed / spline_derivative_length;
    
    // calculate our climb/decent rate at next point
    float climb_rate = spline_derivative.z * t_speed;
    // constrain speed to not exceed max configured climb speed 
    if (climb_rate > spline_max_climb) {
        t_speed = spline_max_climb / spline_derivative.z;
    }
    // constrain speed to not exceed max configured decent speed
    else if (climb_rate < spline_max_descent) {
        t_speed = spline_max_descent / spline_derivative.z;
    }
    
    // calculate our acceleration at next point
    float spline_sdl = spline_2nd_derivative_length(spline_t);
    // note: t_speed is scale factor for velocity, so need to square it to scale acceleration
    float spline_acceleration = spline_sdl * t_speed * t_speed;
    if (spline_acceleration > SPLINE_ACCELERATION) {
        // constrain target speed to not exceed max allowed acceleration
        t_speed = safe_sqrt(SPLINE_ACCELERATION / spline_sdl);
    }
    
    // calculate new spline_target_speed in case any of the above constraints applied
    spline_target_speed = t_speed * spline_derivative_length;
    
    // update the spline segment position
    float spline_t_delta = dt * t_speed; // note: actually just an estimate of spline_t_delta, accurate for small dt
    // todo: add code to improve t_delta estimate
    spline_t += spline_t_delta; // moving forward on the spline curve
    if (spline_t >= 1.0) next_spline_segment();

    // yaw look at point 50 meters ahead in direction of derivative
    spline_derivative.z = 0.0; // ignore z part of derivative for yaw targeting
    spline_derivative.normalize();
    // if derivative was zero it may now be nan or inf after normalization
    if (!spline_derivative.is_nan() && !spline_derivative.is_inf()) {
        yaw_look_at_WP = spline_target + spline_derivative * 5000.0;
    }

    // re-use loiter position controller
    wp_nav.set_loiter_target(spline_target);
    
    // call loiter controller
    wp_nav.update_loiter();
}


// perform initialization in preparation for the new spline segment
void SplineNav::initialize_spline_segment() {

    // derivative of Catmull-Rom spline at p1 based on difference of previous and next points
    spline_p1_prime = (spline_p2 - spline_p0) * 0.5;

    // compute a and b vectors used in the Catmull-Rom spline formula
    spline_a = spline_p0*2.0 - spline_p1*2.0 + spline_p0_prime + spline_p1_prime;
    spline_b = spline_p1*3.0 - spline_p0*3.0 - spline_p0_prime*2.0 - spline_p1_prime;
}

// continue to the next spline segment
void SplineNav::next_spline_segment() {
    // start t back at near the beginning of the new segment
    spline_t -= 1.0;

    spline_p0 = spline_p1;
    spline_p1 = spline_p2;
    spline_p0_prime = spline_p1_prime;
    spline_p2 = next_spline_waypoint();

    initialize_spline_segment();
}

// get the next spline waypoint
Vector3f SplineNav::next_spline_waypoint() {
    struct Location command;
    // search for the next WAYPOINT command
    // todo: recognize some other commands as well, for example ROI would be nice
    int n = g.command_total.get();
    for (int i=1; i<n; i++) {
        if (spline_next_cmd_index >= n) {
            if (SPLINE_LOOP) spline_next_cmd_index = 1; // wrap around to fly a spline loop
            else break;
        }
        command = get_cmd_with_index(spline_next_cmd_index++);
        if (command.id == MAV_CMD_NAV_WAYPOINT) {
            return pv_location_to_vector(command);
        }
    }

    // give up and just return spline start point if we don't find a waypoint
    return spline_start_point;
}

