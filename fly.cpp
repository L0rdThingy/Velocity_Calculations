#include "fly.hpp"
#include "main.h"
#include "pros/llemu.hpp"
#include "pros/misc.hpp"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "util.hpp"

//Flywheel Constructor
Fly::Fly(std::vector<int> flywheel_motor_ports, double ticks_per_rev, double flywheel_ratio)
    :flywheel([this] { this->flywheel_task(); }) {

        for (auto i : flywheel_motor_ports) {   //sets the ports of the flywheel motors
            pros::Motor temp(abs(i), is_reversed(i));
            flywheel_motors.push_back(temp);
        }

    TICKS_PER_REV = ticks_per_rev;
    FLYWHEEL_RATIO = flywheel_ratio;
    flywheel_motors.front().set_brake_mode(pros::E_MOTOR_BRAKE_COAST); //sets brake mode to coast
}



double Fly::get_vel() { return flyVel; } //returns flywheel velocity

double Fly::fly_sensor() {  return flywheel_motors.front().get_position();  } //returns the sensor value of the first motor in the vector

void Fly::set_rpm(double input) { target = input; } //sets the flywheel target rpm

void Fly::move_fly(int power) { //powers the flywheel motors at the value of power
    for (auto i : flywheel_motors) {
        i.move_voltage(power * (12000.0 / 127.0));
    }
}

void Fly::print_vel() { //prints the velocity of the flywheel motor
    pros::lcd::clear_line(4);
    pros::lcd::print(4, "flyVel:%f", flyVel);
}



double Fly::rpm_compute(double delta_enc, double delta_ms, int ticks_per_rev) { //computes the flywheel rpm
    rpm = (1000.0 / delta_ms) * delta_enc * 60.0 / ticks_per_rev; //velocity in rpm
    return rpm;
}

//  Exponential Moving Average to filter the velocity
double Fly::ema_filter(double _rpm, double _prev_rpm) {
    rpm = (_prev_rpm * PAST_SCALE) + (_rpm * PRESENT_SCALE); //averages the rpm between the previous and current rpm reading
    flyVel = rpm * FLYWHEEL_RATIO; //rpm of the flywheel
    return flyVel;
}

void Fly::set_constants(double f, double p, double d) { //sets the pidf constants
    kf = f;
    kp = p;
    kd = d;
}

//Feedforward PID
double Fly::compute_pidf(double input) {
    error = target - input; //proportion

    derivative = error - prev_error;
    prev_error = error;

    final_power = (error * kp) + (derivative * kd) + (target * kf); //Feedforard gets close to the target rpm and the proportion and derivative help make sure it gets to it
    return final_power;
}



void Fly::flywheel_task() {
    set_constants(1, 0, 0);
    while(true) {
            
            rpm_compute( (fly_sensor() - prev_fly_pos), (time - prev_time), TICKS_PER_REV);
            ema_filter(rpm, prev_rpm);
            compute_pidf(flyVel);
            move_fly(final_power);

            print_vel();

            prev_fly_pos = fly_sensor();
            prev_rpm = rpm;
            prev_time = time;
            time += util::DELAY_TIME;
            pros::delay(util::DELAY_TIME);
    }
}
