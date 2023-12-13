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

    Eigen::Matrix2d P;
    Eigen::Matrix2d Q;
    double R;

public:
    kf_bicycle_model(vehicle_model_fw_simplified &_vehicle_model);
    kf_bicycle_model(vehicle_model_fw_simplified &_vehicl_model, Eigen::Matrix2d _P, Eigen::Matrix2d _Q, double _R);

    const Eigen::Matrix2d &getP() const {return P;}
    const Eigen::Matrix2d &getQ() const {return Q;}
    const double &getR() const {return R;}

    void setP(const Eigen::Matrix2d &p) {P = p;}
    void setQ(const Eigen::Matrix2d &q) {Q = q;}
    void setR(const double &r) {R = r;}

    void operator() (const state_type &x, state_type  &dxdt, const double t);
};


#endif //KALMAN_FILTER_FOUR_WHEEL_MODEL_KF_BICYCLE_MODEL_H
