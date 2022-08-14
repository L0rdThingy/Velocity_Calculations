#include "Subsystems/fly.hpp"
#include "main.h"
#include "pros/adi.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "util.hpp"

pros::ADIEncoder fly_enc {3, 4};
pros::ADIDigitalOut indexer {'H'};

// Flywheel Constructor
Fly::Fly(std::vector<int> flywheel_motor_ports, double ticks_per_rev, double flywheel_ratio)
    :flywheel([this] { this->flywheel_task(); }) {

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

// Moving Average
double Fly::get_vel() {
    velocities.push_back(get_rawVel());

    if (velocities.size() >= 10)
        velocities.erase(velocities.begin());

    return std::accumulate(velocities.begin(), velocities.end(), 0.0) / velocities.size();
}

void Fly::set_rpm(double input) { target = input; } //sets the flywheel target rpm

void Fly::move_fly(int power) { //powers the flywheel motors at the value of power
    for (auto i : flywheel_motors) {
        i.move_voltage(power * (12000.0 / 127.0));
    }
}

double Fly::fly_sensor() {  return flywheel_motors[0].get_position();  } //returns the sensor value of the first motor in the vector


double Fly::rpm_compute(double _delta_enc, double _delta_ms, int ticks_per_rev) { //computes the flywheel rpm
    rpm = (1000.0 / _delta_ms) * _delta_enc * 60.0 / ticks_per_rev; //velocity in rpm
    prev_rpm = rpm;
    return rpm;
}

// Exponential Moving Average
double Fly::ema_filter(double _prev_rpm, double _rpm, double past_scale, double present_scale) {
    rpm = (_prev_rpm * past_scale) + (_rpm * present_scale); //averages the rpm between the previous and current rpm reading
    flyVel = rpm * FLYWHEEL_RATIO; //rpm of the flywheel
    return flyVel;
}



void Fly::set_constants(double f, double p, double d) { //sets the pidf constants
    kf = f;
    kp = p;
    kd = d;
}

// Feedforward PID
double Fly::compute_pidf(double input) {
    error = target - input; //proportion
    derivative = error - prev_error;
    prev_error = error;

    final_power = (error * kp) + (derivative * kd) + (target * kf); //Feedforard gets close to the target rpm and the proportion and derivative help make sure it gets to it
    if (final_power < 0) final_power = 0;
    return final_power;
}

void Fly::print_vel() { //prints the velocity of the flywheel motor
    pros::lcd::clear_line(4);
    pros::lcd::print(4, "rawRPM:%.0f RPM:%.0f", get_rawVel(), get_vel());
}

void Fly::indexer_control() {
    if(master.get_digital(pros::E_CONTROLLER_DIGITAL_B) && lock == false) {
        out = !out;
        lock = true;
    }
    else if (!master.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
        lock = false;
    }
        if (out)    indexer.set_value(true);
        else    indexer.set_value(false);
}


void Fly::flywheel_task() {
    set_constants(1, 0, 0);
    time = 0;
    prev_time = 0;
    prev_rpm = 0;
    while(true) {

        //move_fly(compute_pidf(get_vel()));

        //util::print_with_delay( ([this] {this->print_vel();}), 1000);
        print_vel();

        prev_rpm = get_vel();
        prev_time = time;
        time += util::DELAY_TIME;
        pros::delay(util::DELAY_TIME);
    }
}