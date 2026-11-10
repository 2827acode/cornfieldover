#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/asset.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/motor_group.hpp"
#include "pros/motors.h"
#include "pros/rotation.hpp"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <math.h>

// SETS UP MOVEMENT AND MOTOR/MOTORGROUPS, PNEUMATICS, AND AUTON MOVEMENT
pros::Controller master(pros::E_CONTROLLER_MASTER);

// Motor Groups & Motors
pros::MotorGroup right_mg({18, -12, -19}, pros::MotorGearset::blue);
pros::MotorGroup left_mg({-15, 17, 6}, pros::MotorGearset::blue);
pros::MotorGroup DoubleBar({10, -21}, pros::MotorGearset::green);
pros::MotorGroup Cascade({20,-14}, pros::MotorGearset::green);
pros::Motor ClawArm(9, pros::MotorGearset::green);
pros::Motor DoubleBarAngle(21, pros::MotorGearset::green);
pros::Distance dist_sensor(8);

// Pneumatics
bool claw = false;




pros::ADIDigitalOut air('H', claw);


// Sensors
pros::Imu imu(2);

pros::Rotation horizontal_encoder(7);
pros::Rotation vertical_encoder(-4);

lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_275, .1);
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_2, .25);


lemlib::ExpoDriveCurve throttle_curve(3,
                                     10,
                                     1.019 
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steer_curve(3, 
                                  10, 
                                  1 
);

lemlib::Drivetrain drivetrain(&left_mg,
                              &right_mg,
                              11.4,
                              lemlib::Omniwheel::NEW_275,
                              450,
                              2
);

lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            &horizontal_tracking_wheel, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu// inertial sensor
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(7, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              25, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              100 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              7, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              100 // maximum acceleration (slew)
);

// create the chassis
lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors // odometry sensors
);


void on_center_button() {
    static bool pressed = false;
    pressed = !pressed;
    if (pressed) {
        pros::lcd::set_text(2, "I was pressed!");
    } else {
        pros::lcd::clear_line(2);
    }
}


// Start of functions
// Start of functions
// Start of functions


// ---------------- Claw functions ----------------


constexpr double CLAW_TICKS_PER_DEG      = 2.8;
constexpr double DOUBLEBAR_TICKS_PER_DEG = 2.8;

void moveClawToAngle(double angle, int velocity = 200) {
    ClawArm.move_absolute(angle * CLAW_TICKS_PER_DEG, velocity);
}

bool clawAtAngle(double angle, double gearRatio = 1.0, double tolerance = 2.0) {
    return std::fabs(ClawArm.get_position() - (angle * gearRatio)) <= tolerance * gearRatio;
}


volatile bool clawStallActive = false;
volatile bool clawStallCancel = false;


void moveClawUntilStalled(int speed = 100, int currentLimit = 1500,
                          int velocityLimit = 10, int timeout = 3000) {
    if (clawStallActive) return;

    clawStallActive = true;
    clawStallCancel = false;

    pros::Task([=]() {
        const uint32_t graceMs = 200;       
        const uint32_t stallConfirmMs = 50;
        uint32_t start = pros::millis();
        uint32_t stallStart = 0;

        ClawArm.move_velocity(speed);

        while (!clawStallCancel && (pros::millis() - start) < (uint32_t)timeout) {
            uint32_t now = pros::millis();
            bool stalled = ClawArm.get_current_draw() > currentLimit && std::fabs(ClawArm.get_actual_velocity()) < velocityLimit;

            if ((now - start) > graceMs && stalled) {
                if (stallStart == 0) stallStart = now;
                else if (now - stallStart >= stallConfirmMs) break;
            } else {
                stallStart = 0;
            }
            pros::delay(10);
        }

        ClawArm.brake();
        clawStallActive = false;
        ClawArm.tare_position();
    });
}


// ---------------- DoubleBar functions ----------------


bool doubleBarHeld = false;
bool letdoublebarmove = true;

volatile bool doubleBarStallActive = false;
volatile bool doubleBarStallCancel = false;

void moveDoubleBartoangle(double angle, double gearRatio = 5, double anglefix = 2.5, int velocity = 100) {
    DoubleBar.move_absolute(angle * gearRatio * anglefix, velocity);
}

void moveDoubleBarUntilStalled(int speed = 100, int currentLimit = 2000,
                          int velocityLimit = 10, int timeout = 3000) {
    if (doubleBarStallActive) return;

    doubleBarStallActive = true;
    doubleBarStallCancel = false;

    pros::Task([=]() {
        const uint32_t graceMs = 200;
        const uint32_t stallConfirmMs = 100;
        uint32_t start = pros::millis();
        uint32_t stallStart = 0;

        DoubleBar.move_velocity(speed);

        while (!doubleBarStallCancel && (pros::millis() - start) < (uint32_t)timeout) {
            uint32_t now = pros::millis();
            bool stalled = DoubleBar.get_current_draw() > currentLimit &&
            std::fabs(DoubleBar.get_actual_velocity()) < velocityLimit;

            if ((now - start) > graceMs && stalled) {
                if (stallStart == 0) stallStart = now;
                else if (now - stallStart >= stallConfirmMs) break;
            } else {
                stallStart = 0;
            }
            pros::delay(10);
        }

        DoubleBar.brake();
        doubleBarStallActive = false;
    });
}




// TUNING
double VFB_RATIO = .2;

double VFB_KP = 1;

double VFB_KFF = .49;

double VFB_TOLERANCE = 5.0;

volatile bool virtualFourBarActive = false;
volatile bool vfbSettling = false;          
volatile bool vfbHasReference = false;
double vfbClawOffset = 0.0;

void engageVirtualFourBar() {
    if (clawStallActive) return;
    if (!vfbHasReference) {
        vfbClawOffset = ClawArm.get_position() - DoubleBar.get_position() * VFB_RATIO;
        vfbHasReference = true;
    }
    virtualFourBarActive = true;
    vfbSettling = false;
}

void disengageVirtualFourBar() {
    if (!virtualFourBarActive) return;
    virtualFourBarActive = false;
    vfbSettling = true; // let the loop finish the move, then it brakes
}


void resetVirtualFourBarReference() {
    virtualFourBarActive = false;
    vfbSettling = false;
    vfbHasReference = false;
}

void virtualFourBarLoop() {
    const double maxVel = 200.0;
    double prevArmPos = 0.0;
    uint32_t prevTime = 0;
    double armVel = 0.0;
    bool wasRunning = false;

    while (true) {
        bool running = (virtualFourBarActive || vfbSettling) &&
                       vfbHasReference && !clawStallActive;

        if (running) {
            uint32_t now = pros::millis();
            double armPos = DoubleBar.get_position();

            if (!wasRunning) {
                prevArmPos = armPos;
                prevTime = now;
                armVel = 0.0;
            }

            double dt = (now - prevTime) / 1000.0;
            if (dt > 0.0) {
                double rawVel = (armPos - prevArmPos) / dt / 6.0;
                armVel += 0.5 * (rawVel - armVel);
                prevArmPos = armPos;
                prevTime = now;
            }

            double target = vfbClawOffset + armPos * VFB_RATIO;
            double error  = target - ClawArm.get_position();

            if (!virtualFourBarActive && std::fabs(error) < VFB_TOLERANCE) {
                ClawArm.move_absolute(target, 100);
                vfbSettling = false;
            } else {
                double pTerm = std::clamp(error * VFB_KP, -80.0, 80.0);
                double cmd = armVel * VFB_RATIO * VFB_KFF + pTerm;
                ClawArm.move_velocity(std::clamp(cmd, -maxVel, maxVel));
            }
        }

        wasRunning = running;
        pros::delay(10);
    }
}



// ---------------- Cascade functions ----------------

bool cascadeHeld = false;

volatile bool cascadeStallActive = false;
volatile bool cascadeStallCancel = false;

void moveCascadeUntilStalled(int speed = 100, int currentLimit = 1500,
                          int velocityLimit = 50, int timeout = 3000) {
    if (cascadeStallActive) return;

    cascadeStallActive = true;
    cascadeStallCancel = false;

    pros::Task([=]() {
        const uint32_t graceMs = 500;
        const uint32_t stallConfirmMs = 50;
        uint32_t start = pros::millis();
        uint32_t stallStart = 0;

        Cascade.move_velocity(speed);

        while (!cascadeStallCancel && (pros::millis() - start) < (uint32_t)timeout) {
            uint32_t now = pros::millis();
            bool stalled = Cascade.get_current_draw() > currentLimit && std::fabs(Cascade.get_actual_velocity()) < velocityLimit;

            if ((now - start) > graceMs && stalled) {
                if (stallStart == 0) stallStart = now;
                else if (now - stallStart >= stallConfirmMs) break;
            } else {
                stallStart = 0;
            }
            pros::delay(10);
        }

        Cascade.brake();
        cascadeStallActive = false;
        Cascade.tare_position();
    });
}







// End of functions
// End of functions
// End of functions

void initialize() {
    pros::lcd::initialize();
    chassis.calibrate();
    Cascade.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    DoubleBar.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    ClawArm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    DoubleBar.tare_position();

    pros::Task vfbTask(virtualFourBarLoop);

    pros::Task screen_task([&]() {
        while (true) {
            pros::lcd::print(0, "X: %f", chassis.getPose().x);
            pros::lcd::print(1, "Y: %f", chassis.getPose().y );
            pros::lcd::print(2, "Theta: %f",chassis.getPose().theta);
            pros::delay(50);
        }
    });
}

void disabled() {}
void competition_initialize() {}


lemlib::Pose currentPose = chassis.getPose();

void autonomous() {
    // left

    // chassis.moveToPoint(0,48,2000);

    // chassis.setPose(0,0,0);
    // moveCascadeUntilStalled(-200, 500, 10, 3000);

    // claw = false;
    // moveClawUntilStalled(-150, 500, 25, 3000);
    // claw = false;
    // chassis.tank(-45, -45);
    // pros::delay(300);
    // moveDoubleBarUntilStalled(-200, 500, 10, 3000);
    // chassis.tank(35, 35);
    // pros::delay(300);
    // chassis.tank(-65, -65);
    // ClawArm.tare_position();
    // DoubleBar.tare_position();
    // Cascade.tare_position();
    // pros::delay(300);
    // chassis.moveToPoint(0,15.5,1000);
    // chassis.turnToHeading(90, 400);
    // chassis.moveToPoint(16,16.5,525, {.maxSpeed = 65, .minSpeed = 50}, false);
    // claw = true;
    // air.set_value(true);
    // pros::delay(650);
    // chassis.moveToPoint(6,15, 500, {.forwards = false});
    // chassis.turnToHeading(135, 330);
    // DoubleBar.move_absolute(90, 150);
    // ClawArm.move_absolute(5, 150);
    // chassis.moveToPoint(21, .3, 1000, {.maxSpeed = 50, .minSpeed = 45, .earlyExitRange = 2});
    // chassis.swingToHeading(180, lemlib::DriveSide::RIGHT, 400, {.minSpeed = 85, .earlyExitRange = 10}, false);
    // chassis.tank(-65,-65);
    // pros::delay(100);
    // chassis.tank(65,65);
    // pros::delay(150);
    
    // claw = false;
    // air.set_value(false);
    // pros::delay(275);
    // chassis.tank(0,0);
    // Cascade.move_absolute(300, 200);
    // pros::delay(75);
    
    // chassis.turnToHeading(120, 450);
    // chassis.moveToPoint(3,14.5, 900, {.forwards = false, .minSpeed = 45});
    // Cascade.move_absolute(2150, 200);
    // DoubleBar.move_absolute(35, 50);
    // ClawArm.move_absolute(0, 150);
    // chassis.turnToHeading(90, 200);
    // chassis.moveToPoint(18,14.5, 750, {.maxSpeed = 65, .minSpeed = 55}, true);
    // pros::delay(500);
    // DoubleBar.brake();
    // Cascade.move_absolute(100, 200);
    // pros::delay(400);
    // claw = true;
    // air.set_value(true);
    // pros::delay(300);

    // chassis.moveToPoint(-7,14.5 ,1400, {.forwards = false, .maxSpeed = 65});
    // moveCascadeUntilStalled(-200, 500, 10, 3000);
    // pros::delay(300);
    // moveDoubleBarUntilStalled(-200, 500, 10, 3000);
    // moveClawUntilStalled(-150, 500, 25, 3000);
    // chassis.turnToHeading(220, 475);
    // DoubleBar.move_absolute(90, 150);
    // ClawArm.move_absolute(15, 150);
    // chassis.moveToPoint(-20.5, -.3, 3000, {.maxSpeed = 50, .minSpeed = 45, .earlyExitRange = 2});
    // chassis.swingToHeading(180, lemlib::DriveSide::LEFT, 400, {.minSpeed = 85, .earlyExitRange = 10}, false);

    // chassis.tank(-65,-65);
    // pros::delay(100);
    // chassis.tank(45,45);
    // pros::delay(150);
    
    // claw = false;
    // air.set_value(false);
    // pros::delay(275);
    // chassis.tank(0,0);
    // Cascade.move_absolute(300, 200);
    // pros::delay(75);

    // chassis.turnToHeading(260, 500, {.minSpeed = 65});
    // DoubleBar.move_absolute(100, 50);
    // ClawArm.move_absolute(0, 150);
    // chassis.moveToPoint(0,14, 1050, {.forwards = false, .maxSpeed = 100});
    // Cascade.move_absolute(2250, 200);
    // DoubleBar.brake();
    // chassis.turnToHeading(-90, 300);
    // chassis.moveToPoint(-24, 14, 1500, {.maxSpeed = 50, .minSpeed = 45}, true);
    // pros::delay(650);
    // Cascade.move_absolute(350, 200);
    // pros::delay(300);
    // claw = true;
    // air.set_value(true);
    // pros::delay(350);
    // chassis.moveToPoint(-1, 14, 700, {.forwards = false, .maxSpeed = 100});



    // right
    // chassis.setPose(0,0,0);
    // moveCascadeUntilStalled(-200, 500, 10, 3000);

    // claw = false;
    // moveClawUntilStalled(-150, 500, 25, 3000);
    // claw = false;
    // chassis.tank(-45, -45);
    // pros::delay(300);
    // moveDoubleBarUntilStalled(-200, 500, 10, 3000);
    // chassis.tank(35, 35);
    // pros::delay(300);
    // chassis.tank(-65, -65);
    // ClawArm.tare_position();
    // DoubleBar.tare_position();
    // Cascade.tare_position();
    // pros::delay(300);
    // chassis.moveToPoint(0,14,900);
    // chassis.turnToHeading(-90, 400);
    // chassis.moveToPoint(-16,10,750, {.maxSpeed = 65, .minSpeed = 50}, false);
    // claw = true;
    // air.set_value(true);
    // pros::delay(650);
    // chassis.moveToPoint(-6,14, 500, {.forwards = false});
    // chassis.turnToHeading(-135, 300);
    // DoubleBar.move_absolute(90, 150);
    // ClawArm.move_absolute(25, 150);
    // chassis.moveToPoint(-19.7, -.3, 3000, {.maxSpeed = 50, .minSpeed = 45, .earlyExitRange = 2});
    // chassis.swingToHeading(180, lemlib::DriveSide::LEFT, 400, {.minSpeed = 85, .earlyExitRange = 10}, false);

    // chassis.tank(-65,-65);
    // pros::delay(100);
    // chassis.tank(45,45);
    // pros::delay(150);
    
    // claw = false;
    // air.set_value(false);
    // pros::delay(275);
    // chassis.tank(0,0);
    // Cascade.move_absolute(300, 200);
    // pros::delay(75);

    // chassis.turnToHeading(-100, 500, {.minSpeed = 65});
    // DoubleBar.move_absolute(30, 50);
    // ClawArm.move_absolute(0, 150);
    // chassis.moveToPoint(3,13, 900, {.forwards = false, .maxSpeed = 100});
    // Cascade.move_absolute(2250, 200);
    // DoubleBar.brake();
    // chassis.turnToHeading(-90, 300);
    // chassis.moveToPoint(-18, 13, 1500, {.maxSpeed = 75, .minSpeed = 50}, true);
    // pros::delay(650);
    // Cascade.move_absolute(350, 200);
    // pros::delay(400);
    // claw = true;
    // air.set_value(true);
    // pros::delay(350);

    // chassis.moveToPoint(7,14.5 ,1400, {.forwards = false, .maxSpeed = 65});
    // pros::delay(300);
    // moveCascadeUntilStalled(-200, 500, 10, 3000);
    // moveClawUntilStalled(-150, 500, 25, 3000);
    // chassis.turnToHeading(140, 475);
    // moveDoubleBarUntilStalled(-200, 500, 10, 3000);
    // DoubleBar.move_absolute(90, 150);
    // ClawArm.move_absolute(25, 150);
    // chassis.moveToPoint(20, -.3, 3000, {.maxSpeed = 50, .minSpeed = 45, .earlyExitRange = 2});
    // chassis.swingToHeading(180, lemlib::DriveSide::RIGHT, 400, {.minSpeed = 85, .earlyExitRange = 10}, false);

    // chassis.tank(-65,-65);
    // pros::delay(100);
    // chassis.tank(45,45);
    // pros::delay(150);
    
    // claw = false;
    // air.set_value(false);
    // pros::delay(275);
    // chassis.tank(0,0);
    // Cascade.move_absolute(300, 200);
    // pros::delay(75);

    // chassis.turnToHeading(90, 500, {.minSpeed = 65});
    // DoubleBar.move_absolute(100, 50);
    // ClawArm.move_absolute(0, 150);
    // chassis.moveToPoint(0,14, 1050, {.forwards = false, .maxSpeed = 100});
    // Cascade.move_absolute(2250, 200);
    // DoubleBar.brake();
    // chassis.turnToHeading(90, 300);
    // chassis.moveToPoint(24, 14, 1500, {.maxSpeed = 85, .minSpeed = 65}, true);
    // pros::delay(550);
    // Cascade.move_absolute(350, 200);
    // pros::delay(300);
    // claw = true;
    // air.set_value(true);
    // pros::delay(350);
    // chassis.moveToPoint(-1, 14, 700, {.forwards = false, .maxSpeed = 100});







}


void opcontrol() {
    chassis.cancelAllMotions();
    chassis.setPose(0, 0, 0);

    doubleBarStallActive = false;
    doubleBarStallCancel = false;
    clawStallActive = false;
    clawStallCancel = false;
    cascadeStallActive = false;
    cascadeStallCancel = false;
    doubleBarHeld = false;
    cascadeHeld = false;
    

    while (true) {
        // Driver Control
        int dir = master.get_analog(ANALOG_LEFT_Y);
        int turn = master.get_analog(ANALOG_RIGHT_X);

        right_mg.move(dir - turn);
        left_mg.move(dir + turn);

        // -----Universal------ //
        if (master.get_digital_new_press(DIGITAL_X)) {
            pros::Task([] {
                moveClawUntilStalled(-150, 500, 25, 3000);
                moveCascadeUntilStalled(-200, 500, 10, 3000);
                while (cascadeStallActive) {
                    pros::delay(10);
                }
                Cascade.tare_position();
                moveDoubleBarUntilStalled(-200, 500, 10, 3000);
                while (doubleBarStallActive) {
                    pros::delay(10);
                }
                DoubleBar.tare_position();
                DoubleBarAngle.tare_position();
                ClawArm.move(0);
                pros::delay(50);
                ClawArm.brake();
                ClawArm.tare_position();
                air.set_value(true);
                claw = true;
                resetVirtualFourBarReference();

            });
        }
        // -----Double Bar----- //

        if (master.get_digital(DIGITAL_Y)) {
            doubleBarStallCancel = true;
            doubleBarStallActive = false;
            engageVirtualFourBar();
            if (DoubleBarAngle.get_position() < -1675) {
                DoubleBar.brake();
            } else {
                DoubleBar.move(127);
            }
            doubleBarHeld = true;
        }
        else if (master.get_digital(DIGITAL_RIGHT)) {
            letdoublebarmove = true;
            doubleBarStallCancel = true;
            doubleBarStallActive = false;
            engageVirtualFourBar();
            DoubleBar.move(-127);
            if (DoubleBarAngle.get_position() > -110) {
                DoubleBar.brake();
            } else {
                DoubleBar.move(-127);
            }
            doubleBarHeld = true;
        }


        else {
            disengageVirtualFourBar();
            if (doubleBarHeld && !doubleBarStallActive) {
                pros::Task([] {
                    DoubleBar.move(0);
                    pros::delay(50);
                    DoubleBar.brake();
                });
            }
            doubleBarHeld = false;
        }

        //-----Cascade-----//
        if (master.get_digital(DIGITAL_R2)) {
            cascadeStallCancel = true;
            cascadeStallActive = false;
            Cascade.move(-127);
            cascadeHeld = true;
        }
        else if (master.get_digital(DIGITAL_R1)) {
            cascadeStallCancel = true;
            cascadeStallActive = false;
            Cascade.move(127);
            cascadeHeld = true;
            if (DoubleBarAngle.get_position() > -50) {
                cascadeStallActive = true;
                DoubleBar.move_absolute(110, 200);
                cascadeStallActive = false;
            }
        }
        else {
            if (cascadeHeld && !cascadeStallActive) {
                pros::Task([] {
                    Cascade.move(0);
                    pros::delay(125);
                    Cascade.brake();
                });
            }
            cascadeHeld = false;
        }

        // -----Claw-----//

        if (master.get_digital(DIGITAL_L1)) {
            if (claw && dist_sensor.get() <= 37) {
                pros::Task ([] {
                    claw = false;
                    air.set_value(false);
                    pros::delay(350);
                    Cascade.move_absolute(200, 127);

                });
            }

        }

        if (master.get_digital_new_press(DIGITAL_UP)) {
            claw = !claw;
            air.set_value(claw);
        }
         
        if (master.get_digital_new_press(DIGITAL_L2)) {
            claw = true;
            air.set_value(true);
            if (Cascade.get_position() < 250) {
                Cascade.move_absolute(0, 200);
            }
        }






        

        pros::delay(20);
    }
}





















