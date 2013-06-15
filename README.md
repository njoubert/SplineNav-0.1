SplineNav
=========

Class for flying 3D Spline Curves between Waypoints with ArduCopter 3.0

The source code for SplineNav is in the form of a C++ class. There's a .PDE file and an .H file, both of which go in your ArduCopter 3.0 sketch folder:

SplineNav.h
SplineNav.pde

You will also need to make some minor changes elsewhere in the ArduCopter 3.0 code:

1. Add a SplineNav #include and create a SplineNav object in UserCode.pde:

#include "SplineNav.h"
SplineNav spline_nav;

2. Remove the // comment from this line in APM_Config.h:

#define USERHOOK_VARIABLES "UserVariables.h" 

3. Since there's no SPLINE mode defined in ArduCopter, for now just for testing purposes you can commandeer CIRCLE mode instead. Change navigation.pde as follows:

In function set_nav_mode change the NAV_CIRCLE case:

    case NAV_CIRCLE:
        // set start of spline to current position
        spline_nav.start(inertial_nav.get_position());
        nav_initialised = true;
        break;            

Likewise in function update_nav_mode change the NAV_CIRCLE case:

    case NAV_CIRCLE:
        // call spline controller which in
        //turn calls loiter controller
        spline_nav.update(dTnav);
        break;

Now you should be able to compile with the special ArduPilot version of the Arduino IDE, and upload to your copter. Then set your waypoints with Mission Planner, or with the channel 7 switch, and test out SplineNav.
