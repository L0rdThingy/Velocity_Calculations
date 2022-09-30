#pragma once
#include "main.h"
#include "pros/misc.h"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include "util.hpp"
#include <functional>
#include <vector>
#include <numeric>

    class Fly {

    public: 

    /**
     * \brief Flywheel Constructor
     *
     * \param flywheel_motor_ports
     *        vector for the ports of the flywheel motors
     * \param ticks_per_rev
     *        ticks per revolution of the flywheel motors
     * \param flywheel_ratio
     *        external gear ratio of the flywheel
     */
    Fly(std::vector<int> flywheel_motor_ports, double ticks_per_rev, double flywheel_ratio);

    /**
     * \brief computes the rpm of the flywheel motor
     *
     * \param delta_enc
     *        change in the flywheel sensor from the past and current loop
     * \param delta_ms
     *        difference in time from the past and current loop
     * \param ticks_per_rev
     *        ticks per revolution of the flywheel motors
     */
    double rpm_compute(double _delta_enc, double _delta_ms, int ticks_per_rev);

    /**
     * \brief exponential moving average filter for velocity
     *
     * \param _prev_rpm
     *        the current motor velocity
     * \param _rpm
     *        the previous motor velocity
     * \param past_scale
     *        scaling the past rpm
     * \param present_scale
     *        scaling the present rpm
     */
    double ema_filter(double _prev_rpm, double _rpm, double past_scale, double present_scale);

    /**
     * \brief Feedforward PID
     *
     * \param input
     *        input value (current velocity)
     */
    double compute_pidf(double input);

    /**
     * \brief sets the constants for the pidf
     *
     * \param f
     *        feedback constant
     * \param p
     *        proportion constnat
     * \param d
     *        derivative constant
     */
    void set_constants(double f, double p, double d);

    /**
     * \brief reset the feedback pid targets
     */
    void reset_pidf_targets();
    
    /**
     * \brief returns the ticks per revolution
     */
    double get_tick_per_rev();
    
    /**
     * \brief print the velocity of the flywheel to the brain
     */
    void print_vel();

    /**
     * \brief prints the flywheel motor temp
     */
    void print_fly_motor_temp();

    /** 
     * \brief returns the flywheel sensor reading
     */
    double fly_sensor();

    /**
     * \brief resets the flywheel motor sensor
     */
    void reset_fly_sensor();

    /**
     * \brief flywheel task loop
     */
    void flywheel_task();

    /**
     * \brief powers the flywheel
     *
     * \param power
     *        power the flywheel motors will be set to
     */
    void move_fly(int power);

    /** 
     * \brief sets the target rpm
     *
     * \param input
     *        target rpm
     */
    void set_rpm(double input);

    /**
     * \brief returns the velocity
     */
    double get_rawVel();

    /**
      * \brief returns filtered velocity
      */
    double get_vel();

    /**
     * \brief indexer control
     */
    void indexer_control();

    /**
     * \brief modify rpm with controller
     */
    void modify_rpm();

    /**
     * \brief toggle modify rpm with controller
     *
     * \param toggle
     *        toggles modifying rpm
     */
    void toggle_modify_rpm(bool toggle);

    /**
     * \brief set rpm default
     */
    void set_rpm_default(double rpm);
    
    /**
     * \brief sets buttons for modifying the rpm with the controller
     *
     * \param decrease
     *        a pros button enumerator
     * \param increase
     *        a pros button enumerator
     */
    void set_rpm_buttons(pros::controller_digital_e_t decrease, pros::controller_digital_e_t increase);

    pros::Task flywheel;
    std::vector<pros::Motor> flywheel_motors;
    std::vector<double> velocities;

    void vel_to_volt_benchmark();

    bool disabled;
    double delta_enc;
    double delta_ms;
    double flyVel;
    double prev_flyVel;
    double TICKS_PER_REV;
    double FLYWHEEL_RATIO;
    double rpm;
    double prev_rpm;
    double target;
    double derivative;
    double integral;
    double error;
    double prev_error;
    double final_power;
    double kf;
    double kp;
    double kd;
    double prev_fly_pos;
    double PAST_SCALE = 0.7;
    double PRESENT_SCALE = 0.3;
    bool disable_rpm;
    double avg;
    int i = 0;
    double avg_rpm = 0;
    long elapsed_time = 0;
    bool done = false;
    


    private:
    long time;
    long prev_time;

    struct button_ {
      bool lock = false;
      bool release_reset = 0;
      int release_timer = 0;
      int hold_timer = 0;
      int increase_timer;
      pros::controller_digital_e_t button;
    };
    button_ rpm_increase_;
    button_ rpm_decrease_;

    /**
     * \brief Function for button Presses
     */
    void button_press(button_* input_name, int button, std::function<void()> changeRPM);

    /**
     * \brief Increase and Decrease rpm.
     */
    void rpm_increase();
    void rpm_decrease();


    };

    extern Fly fly;