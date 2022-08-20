#include "Subsystems/fly.hpp"
#include "main.h"
#include "pros/adi.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "util.hpp"

pros::ADIDigitalOut indexer {'H'};

// Flywheel Constructor
Fly::Fly(std::vector<int> flywheel_motor_ports, double ticks_per_rev, double flywheel_ratio)
    :flywheel([this] { this->flywheel_task(); }) {

        //Flywheel motors are held in a vector
        for (auto i : flywheel_motor_ports) {   //sets the ports of the flywheel motors
            pros::Motor temp(abs(i), is_reversed(i));
            flywheel_motors.push_back(temp);
        }

    TICKS_PER_REV = ticks_per_rev;
    FLYWHEEL_RATIO = flywheel_ratio;

    for (auto i : flywheel_motors) {
        i.set_brake_mode(pros::E_MOTOR_BRAKE_COAST); //sets brake mode to coast
        i.set_gearing(pros::E_MOTOR_GEARSET_06);    //Sets the cart of the motor
    }
}



double Fly::get_rawVel() { return flywheel_motors[0].get_actual_velocity() * FLYWHEEL_RATIO; } //returns flywheel velocity

///
// Moving Average
///
double Fly::get_vel() {
    velocities.push_back(get_rawVel()); //adds the current raw velocity to the end of the velocities vector
    if (velocities.size() >= 10) //the velocities vector holds 9 elements (previous velocities), if the vector holds more than 9 elements, erase the first element
        velocities.erase(velocities.begin());
    //the accumuate function sums the range of elements(in our case, the first velocity to the last velocity in our vector) and our init value of the sum is 0
    return std::accumulate(velocities.begin(), velocities.end(), 0.0) / velocities.size();
    //the sum is divided the the amount of elements in our vector to get an average giving us the filtered velocity
}

void Fly::set_rpm(double input) { target = input; } //sets the flywheel target rpm

void Fly::move_fly(int power) { //powers the flywheel motors at the value of power
    for (auto i : flywheel_motors) {
        i.move_voltage(power * (12000.0 / 127.0));
    }
}

double Fly::fly_sensor() {  return flywheel_motors[0].get_position();  } //returns the sensor value of the first motor in the vector

//WARNING: more accurate to use the get_actual_velocity function because it uses vexOS which is theoretically faster
double Fly::rpm_compute(double _delta_enc, double _delta_ms, int ticks_per_rev) { //computes the flywheel rpm
    rpm = (1000.0 / _delta_ms) * _delta_enc * 60.0 / ticks_per_rev; //velocity in rpm
    prev_rpm = rpm;
    return rpm;
}

// Exponential Moving Average (WARNING: not nearly as effective as the moving average above)
double Fly::ema_filter(double _prev_rpm, double _rpm, double past_scale, double present_scale) {
    rpm = (_prev_rpm * past_scale) + (_rpm * present_scale); //averages the rpm between the previous and current rpm reading
    flyVel = rpm * FLYWHEEL_RATIO; //rpm of the flywheel
    return flyVel;
}




///
// Feedforward PID
///
//Feedforard gets close to the target rpm and the proportion and derivative help make sure it gets to it (technically not pid because no integral)
double Fly::compute_pidf(double input) {
    error = target - input; //proportion
    derivative = error - prev_error;
    prev_error = error;

    final_power = (error * kp) + (derivative * kd) + (target * kf);
    if (final_power < 0) final_power = 0;
    return final_power;
}

void Fly::set_constants(double f, double p, double d) { //sets the pidf constants
    kf = f;
    kp = p;
    kd = d;
}



void Fly::print_vel() { //prints the raw and filtered velocity of the flywheel
    pros::lcd::clear_line(4);
    pros::lcd::print(4, "rawRPM:%.0f RPM:%.0f", get_rawVel(), get_vel());
}


void Fly::flywheel_task() {
    set_constants(1, 0, 0);
    while(true) {

        move_fly(compute_pidf(get_vel()));
        //util::print_with_delay( ([this] {this->print_vel();}), 1000);
        print_vel();

        pros::delay(util::DELAY_TIME);
    }
}
