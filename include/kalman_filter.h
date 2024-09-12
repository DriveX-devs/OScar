#ifndef KALMANFILTER_H
#define KALMANFILTER_H
#include <iostream>
#include <vector>
#include <array>
#include <algorithm>
#include <numeric>
#include <cmath>
#include <cassert>
#include "vehicleDataDef.h"

class KalmanFilter
{
public:

    struct KFState{
        double x;
        double y;
        double vx;
        double vy;
        double ax;
        double ay;
        double lat;
        double lon;
    };

    KalmanFilter();
    KalmanFilter(double dt);

    void set_dt(double new_dt);

    void init_step(ldmmap::vehicleData_t vehicleData);
    void init_step(double x_init, double y_init, double vx, double vy, double ax, double ay);
    void init_step(double x_init, double y_init, double vx, double vy);
    void init_step(double x_init, double y_init);

    void predict();

    KFState get_state();

    KFState update(double x_meas, double y_meas, double lon0);

private:

    void initializeMatrices();
    std::vector<std::vector<double>> matrixMultiply(const std::vector<std::vector<double>>& A, const std::vector<std::vector<double>>& B);
    std::vector<double> matrixVectorMultiply(const std::vector<std::vector<double>>& A, const std::vector<double>& B);
    std::vector<std::vector<double>> transpose(const std::vector<std::vector<double>>& A);
    std::vector<std::vector<double>> inverse(const std::vector<std::vector<double>>& A);
    std::vector<std::vector<double>> identity(size_t size);

    double m_dt; // seconds
    uint64_t m_last_predict; // [ms]
    uint64_t m_last_update;  // [ms]
    std::vector<std::vector<double>> F; // State transition matrix
    std::vector<std::vector<double>> H; // Measurement matrix
    std::vector<std::vector<double>> R; // Measurement noise covariance
    std::vector<std::vector<double>> Q; // Process noise covariance
    std::vector<std::vector<double>> P; // Error covariance
    std::vector<double> x; // State vector

    KFState m_state{};
    double m_lon0;
};



#endif //KALMANFILTER_H
