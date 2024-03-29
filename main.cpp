#include <iostream>

#include "src/kf_bicycle_model.h"
#include "submodules/sensor_data/vehicle_simulator.h"
#include <memory>
#include "fstream"
#include "boost/numeric/odeint.hpp"

#include <sciplot/sciplot.hpp>
using namespace sciplot;

//[ integrate_observer
struct push_back_state_and_time
{
    std::vector< state_type >& m_states;
    std::vector< double >& m_times;

    push_back_state_and_time( std::vector< state_type > &states , std::vector< double > &times )
            : m_states( states ) , m_times( times ) { }

    void operator()( const state_type &x , double t )
    {
        m_states.push_back( x );
        m_times.push_back( t );
    }
};
//]

int main() {

    using namespace std;
    using namespace boost::numeric::odeint;

    // Initializing the input struct
    shared_ptr<inputs> input_struct = make_shared<inputs>();

    // Initializing steering input
    input_struct->delta = {0 , 0 , 0 , 0 , 0.05 , 0.05 , 0 , 0 , -0.05 , -0.05 , 0, 0} ;
    // Initializing time input
    input_struct->delta = {0 , 1 , 2 , 3 , 4 , 5 , 6 , 7 , 8 , 9 , 10 , 11 };

    // Initial Conditions
    double u_start{100/3.6};
    Eigen::VectorXd q0 = Eigen::VectorXd::Zero(30);
    q0(14) = u_start;
    q0(24) = u_start/(input_struct)->r_01;
    q0(25) = u_start/(input_struct)->r_02;
    q0(26) = u_start/(input_struct)->r_03;
    q0(27) = u_start/(input_struct)->r_04;

    // Initializing Kalman Filter Matrices
    Eigen::Matrix2d P = Eigen::Matrix2d::Zero();
    Eigen::Matrix2d Q;
    Q << 0.1, 0, 0, 0.005;
    double R = 0.01;
    Eigen::Vector2d x_kap = Eigen::Vector2d::Zero();

    // Initializing Vehicle Model Object
    shared_ptr<vehicle_model_fw_simplified> vehicle_model = make_shared<vehicle_model_fw_simplified>((*input_struct));
    shared_ptr<kf_bicycle_model> simulate_vehicle = make_shared<kf_bicycle_model>((*vehicle_model), P, Q, R, x_kap);


    // Starting Integration
    state_type x(30);
    x[14] = u_start;
    x[24] = u_start/(input_struct)->r_01;
    x[25] = u_start/(input_struct)->r_02;
    x[27] = u_start/(input_struct)->r_04;
    x[26] = u_start/(input_struct)->r_03;

    //[ integrate_observ
    vector<state_type> x_vec;
    vector<double> times;

    ofstream fout("sim.csv");
    if(!fout)
    {
        cout<<"\n error"<<endl;
    }

    size_t steps = integrate((*simulate_vehicle) ,
                             x , 0.0 , 5.0 , 0.01,
                             push_back_state_and_time(x_vec, times));

    //////////////////////////////////////////////////////////////
    // PLOTTING
    //////////////////////////////////////////////////////////////
    // Creating time-vector
    Vec t_vec = std::valarray<double>(times.data(), times.size());

    // Creating valarrays to hold all vehicle states of interest
    Vec long_speed(x_vec.size());
    Vec lat_speed(x_vec.size());
    Vec yaw_rate(x_vec.size());
    Vec roll_angle(x_vec.size());
    Vec chassis_heave(x_vec.size());
    Vec lat_speed_kf(simulate_vehicle->lat_speed_kf.size());
    Vec yaw_rate_kf(simulate_vehicle->yaw_rate_kf.size());


    for (size_t i = 0; i < x_vec.size(); ++i) {
        long_speed[i] = x_vec[i][14];
        lat_speed[i] = x_vec[i][15];
        yaw_rate[i] = x_vec[i][19];
        roll_angle[i] = x_vec[i][3];
        chassis_heave[i] = x_vec[i][2];
    }

    for (size_t i = 0; i < lat_speed_kf.size(); ++i)
    {
        lat_speed_kf[i] = simulate_vehicle->lat_speed_kf[i];
        yaw_rate_kf[i] = simulate_vehicle->yaw_rate_kf[i];
    }

    Plot2D plt_long_speed;
    plt_long_speed.legend().atOutsideBottom().displayHorizontal().displayExpandWidthBy(2);
    plt_long_speed.xlabel("Time [s]");
    plt_long_speed.ylabel("Speed [kmph]");
    plt_long_speed.drawCurve(t_vec, long_speed*3.6).label("Vx [kmph]");

    Plot2D plt_lat_speed;
    plt_lat_speed.legend().atOutsideBottom().displayHorizontal().displayExpandWidthBy(2);
    plt_lat_speed.xlabel("Time [s]");
    plt_lat_speed.ylabel("Lateral Speed [m/s]");
    plt_lat_speed.drawCurve(t_vec, lat_speed).label("Vy [m/s]");
    plt_lat_speed.drawCurve(t_vec, lat_speed_kf).label("Vy - KF [m/s]");

    Plot2D plt_yaw_rate;
    plt_yaw_rate.legend().atOutsideBottom().displayHorizontal().displayExpandWidthBy(2);
    plt_yaw_rate.xlabel("Time [s]");
    plt_yaw_rate.ylabel("Yaw Rate [deg/s]");
    plt_yaw_rate.drawCurve(t_vec, yaw_rate*57.4).label("r [deg/s]");
    plt_yaw_rate.drawCurve(t_vec, yaw_rate_kf*57.4).label("r - KF [deg/s]");

    Plot2D plt_roll_angle;
    plt_roll_angle.legend().atOutsideBottom().displayHorizontal().displayExpandWidthBy(2);
    plt_roll_angle.xlabel("Time [s]");
    plt_roll_angle.ylabel("Roll Angle [deg]");
    plt_roll_angle.drawCurve(t_vec, roll_angle*57.4).label("theta [deg]");

    // Create figure to hold plot
    Figure fig = {{plt_long_speed,plt_lat_speed}, {plt_yaw_rate, plt_roll_angle}};
    // Create canvas to hold figure
    Canvas canvas = {{fig}};
    canvas.size(749,749);

    // Show the plot in a pop-up window
    canvas.show();


    return 0;
}
