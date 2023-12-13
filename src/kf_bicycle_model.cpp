//
// Created by gradhakrish on 12/7/23.
//

#include "kf_bicycle_model.h"

kf_bicycle_model::kf_bicycle_model(vehicle_model_fw_simplified &_vehicle_model) : vehicle_model(_vehicle_model) {}

kf_bicycle_model::kf_bicycle_model(vehicle_model_fw_simplified &_vehicl_model, Eigen::Matrix2d _P, Eigen::Matrix2d _Q,
                                   double _R) : vehicle_model(_vehicl_model), P(_P), Q(_Q), R(_R) {}

void kf_bicycle_model::operator() ( const state_type &x , state_type &dxdt , const double  t  )
{
    // Interpolating the steering input
    double delta_c{0};

    if (t > 1.5 && t < 2.5)
    {
        delta_c = 0.05;
    }

    // Initializing a driving torque input
    double m_d_c{0};

    Eigen::VectorXd q = Eigen::VectorXd::Zero(28);

    for (int i = 0; i < x.size(); ++i) {
        q(i) = x[i];
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // SENSOR DATA
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Getting sensor data
    Eigen::VectorXd Qdot = vehicle_model.solve(q,delta_c, m_d_c);



    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // KALMAN FILTER
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Initializing Kalman Filter States
    Eigen::Vector2d x_kap = Eigen::Vector2d::Zero();
    x_kap << x[28], x[29];

    double U{delta_c};

    // Initializing Output Vector
    double y{x[19]};

    // Initializing Vehicle Inputs
    double m_s{vehicle_model.get_model_input().m_s}, I_zz{vehicle_model.get_model_input().J_z};
    double C_y_1{vehicle_model.get_model_input().C_y_1}, C_y_2{vehicle_model.get_model_input().C_y_2};
    double a{vehicle_model.get_model_input().a_1}, b{vehicle_model.get_model_input().b};
    double u{vehicle_model.get_outputs_model()[0]};

    // Kalman Filter - System Matrix
    Eigen::Matrix2d kf_A = Eigen::Matrix2d::Zero();
    kf_A << -( C_y_1 + C_y_2 )/m_s , u + (a*C_y_1 - b*C_y_2)/(m_s*u),
            (a*C_y_1 - b*C_y_2)/(I_zz*u), (a*a*C_y_1 + b*b*C_y_2)/(I_zz*u);

    Eigen::Vector2d kf_B = Eigen::Vector2d::Zero();
    kf_B << C_y_1/m_s , (a*C_y_1)/(I_zz*u);

    Eigen::RowVector2d kf_C(2);
    kf_C << 0, 1;

    Eigen::Vector2d x_kap_minus = Eigen::Vector2d::Zero();
    x_kap_minus = kf_A*x_kap + kf_B*U;

    Eigen::Matrix2d P_k_minus = Eigen::Matrix2d::Zero();
    P_k_minus = kf_A*P*kf_A.transpose() + Q;

    Eigen::Vector2d L_k;

    L_k = (P_k_minus * kf_C.transpose())/((kf_C * P_k_minus * kf_C.transpose()) + R);

    Eigen::Vector2d x_kap_plus = Eigen::Vector2d::Zero();
    x_kap_plus = x_kap_minus + L_k*(y - (kf_C*x_kap_minus));

    P = (Eigen::Matrix2d::Identity() - L_k*kf_C)*P_k_minus;











    for (int i=0; i < Qdot.size(); ++i)
    {
        dxdt[i] = (Qdot(i));
    }





}


