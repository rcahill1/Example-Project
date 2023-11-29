#include "main.h"
#include "pros/motors.hpp"


/////
// For instalattion, upgrading, documentations and tutorials, check out website!
// https://ez-robotics.github.io/EZ-Template/
/////


// Chassis constructor
Drive chassis (
  // Left Chassis Ports (negative port will reverse it!)
  //   the first port is the sensored port (when trackers are not used!)
  {-3, -1, 2}

  // Right Chassis Ports (negative port will reverse it!)
  //   the first port is the sensored port (when trackers are not used!)
  ,{8, 10, -9}

  // IMU Port
  ,11

  // Wheel Diameter (Remember, 4" wheels are actually 4.125!)
  //    (or tracking wheel diameter)
  ,3.25

  // Cartridge RPM
  //   (or tick per rotation if using tracking wheels)
  ,600

  // External Gear Ratio (MUST BE DECIMAL)
  //    (or gear ratio of tracking wheel)
  // eg. if your drive is 84:36 where the 36t is powered, your RATIO would be 2.333.
  // eg. if your drive is 36:60 where the 60t is powered, your RATIO would be 0.6.
  ,1.667

  // Uncomment if using tracking wheels
  /*
  // Left Tracking Wheel Ports (negative port will reverse it!)
  // ,{1, 2} // 3 wire encoder
  // ,8 // Rotation sensor

  // Right Tracking Wheel Ports (negative port will reverse it!)
  // ,{-3, -4} // 3 wire encoder
  // ,-9 // Rotation sensor
  */

  // Uncomment if tracking wheels are plugged into a 3 wire expander
  // 3 Wire Port Expander Smart Port
  // ,1
);



/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  // Print our branding over your terminal :D
  ez::print_ez_template();
  
  pros::delay(500); // Stop the user from doing anything while legacy ports configure.

  // Configure your chassis controls
  chassis.toggle_modify_curve_with_controller(true); // Enables modifying the controller curve with buttons on the joysticks
  chassis.set_active_brake(0); // Sets the active brake kP. We recommend 0.1.
  chassis.set_curve_default(0, 0); // Defaults for curve. If using tank, only the first parameter is used. (Comment this line out if you have an SD card!)  
  default_constants(); // Set the drive to your own constants from autons.cpp!
  exit_condition_defaults(); // Set the exit conditions to your own constants from autons.cpp!

  // These are already defaulted to these buttons, but you can change the left/right curve buttons here!
  // chassis.set_left_curve_buttons (pros::E_CONTROLLER_DIGITAL_LEFT, pros::E_CONTROLLER_DIGITAL_RIGHT); // If using tank, only the left side is used. 
  // chassis.set_right_curve_buttons(pros::E_CONTROLLER_DIGITAL_Y,    pros::E_CONTROLLER_DIGITAL_A);

  // Autonomous Selector using LLEMU
  ez::as::auton_selector.add_autons({
    Auton("autonSr\n\nMy second autonR, close auton", autonSclose),
    Auton("autonSl\n\nMy first autonL, far auton", autonSfar),
    Auton("skillaout\n\nMy skill aout", skillaout),
    
  });

  // Initialize chassis and auton selector
  chassis.initialize();
  ez::as::initialize();
}



/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
  // . . .
}



/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
  // . . .
}



/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
  chassis.reset_pid_targets(); // Resets PID targets to 0
  chassis.reset_gyro(); // Reset gyro position to 0
  chassis.reset_drive_sensor(); // Reset drive sensors to 0
  chassis.set_drive_brake(MOTOR_BRAKE_HOLD); // Set motors to hold.  This helps autonomous consistency.

  ez::as::auton_selector.call_selected_auton(); // Calls selected auton from autonomous selector.
}



/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
  // This is preference to what you like to drive on.

  chassis.set_drive_brake(MOTOR_BRAKE_COAST);
  pros::Controller master (pros::E_CONTROLLER_MASTER); //sets the controller name to "master"
  pros::Motor catamotor (5); //sets catapult motor to port 5
  pros::Motor bluemot (6); //sets the intake motor to port 6
  pros::Rotation rotationSensor(4); // Rotation sensor connected to port 4
  pros::ADIDigitalOut firstpiston ('A'); //sets the wing piston to port A
  pros::ADIDigitalOut secondpiston ('B'); //sets the auton piston to port B
  rotationSensor.reset_position(); //value of rotation sensor set to 0 at the position it is in when the robot starts
  firstpiston.set_value(false); //value used for pushing the wings
  secondpiston.set_value(false); //value used for pushing the swiper
  bool state = false; //value used for wings
  bool state2 = false; //value used for swiper
  bool mid = true; //value used for catapult functions

  while (true) {
    //Driving
    chassis.arcade_standard(ez::SPLIT); // Standard split arcade control is set

    float currentRotations = rotationSensor.get_position(); //gets a value determining the position the rotation sensor is at
    // pros::lcd::clear(); //Clears the space on the screen where the value of current rotation will be placed
    // pros::lcd::print(0, "Current Position: %f", currentRotations); //prints the value represented by "currentRotations" to the brain

    //INTAKE SYSTEM
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) { 
     bluemot = (127); //Pressing R1 enables triball intake at full speed
     } 
    else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
     bluemot = (-127); //Pressing R2 reverses the motor to releasing the held triball
     } 
    else { //No input turns the intake system off 
      bluemot = (0);
    }

    //PISTONS AND WINGS SYSTEM
   if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) { 
    firstpiston.set_value(!(state)); state = !state; //pressing A will extend the wings
    }
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) { 
    secondpiston.set_value(!(state2)); state2 = !state2; //pressing B will do thing
    }

    //AUTOMATIC SHOOTING AND POSITIONING
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) { //pressing R1 moves the catapult down
      catamotor.move_velocity(100);
    }
    else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)){
      mid = !mid; //pressing Y enables or disables the mid value
    }
    else if ((rotationSensor.get_position() < 4500) && mid == true) {
      catamotor.move_velocity(100); //this puts the catapult in between the top and bottom to keep a triball in intake
    }
    else if ((rotationSensor.get_position() < 6000) && mid != true) {
      catamotor.move_velocity(100); //this puts the catapult down so triballs can be loaded from intake into the catapult
    }
    else{
      catamotor.move_velocity(0); //stops the catapult motor from moving when the two statements above are complete
    }

    pros::delay(ez::util::DELAY_TIME); // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}
