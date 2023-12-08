//
// Created by gradhakrish on 12/7/23.
//

#ifndef KALMAN_FILTER_FOUR_WHEEL_MODEL_KF_BICYCLE_MODEL_H
#define KALMAN_FILTER_FOUR_WHEEL_MODEL_KF_BICYCLE_MODEL_H

#include "eigen3/Eigen/Dense"
#include "../submodules/sensor_data/vehicle_model_fw_simplified.h"

typedef std::vector<double> state_type;


class kf_bicycle_model {

    vehicle_model_fw_simplified vehicle_model;

public:
    kf_bicycle_model(vehicle_model_fw_simplified &_vehicle_model);

    void operator() (const state_type &x, state_type  &dxdt, const double t);
};


#endif //KALMAN_FILTER_FOUR_WHEEL_MODEL_KF_BICYCLE_MODEL_H
