//
// Created by gradhakrish on 12/7/23.
//

#include "kf_bicycle_model.h"

kf_bicycle_model::kf_bicycle_model(vehicle_model_fw_simplified &_vehicle_model) : vehicle_model(_vehicle_model) {}

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

    Eigen::VectorXd Qdot = vehicle_model.solve(q,delta_c, m_d_c);

    for (int i=0; i < Qdot.size(); ++i)
    {
        dxdt[i] = (Qdot(i));
    }





}
