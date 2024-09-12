#include "kalman_filter.h"
#include "utils.h"
#include "asn_utils.h"
#include "utmuts.h"

KalmanFilter::KalmanFilter() {
    m_dt = 0.04; // 40 ms
    m_last_update = 0;
    m_last_predict = 0;
    m_lon0 = 0;
    m_state = {0, 0, 0, 0, 0, 0, 0,0};
    initializeMatrices();
}

KalmanFilter::KalmanFilter(double m_dt) : m_dt(m_dt), m_last_predict(0), m_last_update(0) {
    m_state = {0, 0, 0, 0, 0, 0,0,0};
    m_lon0 = 0;
    initializeMatrices();
}

void KalmanFilter::set_dt(double new_dt) {
    m_dt = new_dt;
    m_lon0 = 0;
    initializeMatrices();
}

void KalmanFilter::init_step(ldmmap::vehicleData_t vehicleData) {
    x = {((double) vehicleData.xDistance)/CENTI, ((double) vehicleData.yDistance)/CENTI, 0, 0,0, 0};
    m_state = {x[0], x[1], x[2], x[3], x[4], x[5],vehicleData.lat,vehicleData.lon};
    this->predict();
}

void KalmanFilter::init_step(double x_init, double y_init, double vx, double vy, double ax, double ay) {
    x = {x_init, y_init, vx, vy, ax, ay};
    m_state = {x[0], x[1], x[2], x[3], x[4], x[5],0,0};
    this->predict();
}

void KalmanFilter::init_step(double x_init, double y_init, double vx, double vy) {
    x = {x_init, y_init, vx, vy, 0, 0};
    m_state = {x[0], x[1], x[2], x[3], x[4], x[5],0,0};
    this->predict();
}

void KalmanFilter::init_step(double x_init, double y_init) {
    x = {x_init, y_init, 0, 0, 0, 0};
    m_state = {x[0], x[1], x[2], x[3], x[4], x[5],0,0};
}

void KalmanFilter::predict() {
    double prev_x = x[0];
    double prev_y = x[1];
    x = matrixVectorMultiply(F, x);
    auto FT = transpose(F);
    P = matrixMultiply(matrixMultiply(F, P), FT);
    for (size_t i = 0; i < Q.size(); ++i) {
        for (size_t j = 0; j < Q[0].size(); ++j) {
            P[i][j] += Q[i][j];
        }
    }
    m_last_predict = get_timestamp_ms_gn();

    transverse_mercator_t tmerc = UTMUPS_init_UTM_TransverseMercator();
    double lat1, lon1, abs_x, abs_y, gammar=0,kr=0;
    TransverseMercator_Forward(&tmerc, m_lon0, m_state.lat, m_state.lon, &abs_x, &abs_y, &gammar, &kr);
    abs_x += prev_x - x[0];
    abs_y += prev_y - x[1];
    TransverseMercator_Reverse(&tmerc, m_lon0, abs_x, abs_y, &lat1, &lon1, &gammar, &kr);

    m_state = {x[0], x[1], x[2], x[3], x[4], x[5], lat1, lon1};

    //std::cout << "[" << get_timestamp_ms_gn() << "] Predicted state: " << m_state.x << " " << m_state.y << " " << m_state.vx << " " << m_state.vy << " " << m_state.ax << " " << m_state.ay << " " << m_state.lat << " " << m_state.lon << std::endl;
}

KalmanFilter::KFState KalmanFilter::update(double x_meas, double y_meas, double lon0) {
    uint64_t now = get_timestamp_ms_gn();
    m_lon0 = lon0;
    if (now - m_last_update >= (uint64_t) ((m_dt*MILLI)-1)) {
        this->predict();
        double prev_x = x[0];
        double prev_y = x[1];
        std::vector<double> z = {x_meas, y_meas};
        auto y = z;
        auto hx = matrixVectorMultiply(H, x);
        std::transform(y.begin(), y.end(), hx.begin(), y.begin(), std::minus<>());
        auto HT = transpose(H);
        auto S = matrixMultiply(matrixMultiply(H, P), HT);
        for (size_t i = 0; i < R.size(); ++i) {
            for (size_t j = 0; j < R[0].size(); ++j) {
                S[i][j] += R[i][j];
            }
        }
        auto K = matrixMultiply(matrixMultiply(P, HT), inverse(S));
        auto Ky = matrixVectorMultiply(K, y);
        std::transform(x.begin(), x.end(), Ky.begin(), x.begin(), std::plus<>());
        auto I = identity(P.size());
        auto KH = matrixMultiply(K, H);
        for (size_t i = 0; i < I.size(); ++i) {
            for (size_t j = 0; j < I[0].size(); ++j) {
                I[i][j] -= KH[i][j];
            }
        }
        P = matrixMultiply(I, P);
        m_last_update = now;

        transverse_mercator_t tmerc = UTMUPS_init_UTM_TransverseMercator();
        double lat1, lon1, abs_x, abs_y, gammar=0,kr=0;
        TransverseMercator_Forward(&tmerc, m_lon0, m_state.lat, m_state.lon, &abs_x, &abs_y, &gammar, &kr);
        abs_x += prev_x - x[0];
        abs_y += prev_y - x[1];
        TransverseMercator_Reverse(&tmerc, m_lon0, abs_x, abs_y, &lat1, &lon1, &gammar, &kr);

        //std::cout << "[" << get_timestamp_ms_gn() << "] New measurement: " << x_meas << " " << y_meas << std::endl;
        m_state = {x[0], x[1], x[2], x[3], x[4], x[5], lat1, lon1};
        //std::cout << "[" << get_timestamp_ms_gn() << "] Updated state: " << m_state.x << " " << m_state.y << " " << m_state.vx << " " << m_state.vy << " " << m_state.ax << " " << m_state.ay << " " << m_state.lat << " " << m_state.lon << std::endl;
    }
//    else
//    {
//        std::cout << "[" << get_timestamp_ms_gn() << "] Not updating state, too soon since last update :" << now - m_last_update << std::endl;
//    }

    return m_state;
}

KalmanFilter::KFState
KalmanFilter::get_state()
{
    return m_state;
}

void KalmanFilter::initializeMatrices() {
    F = {
            {1, 0, m_dt, 0, 0.5 * m_dt * m_dt, 0},
            {0, 1, 0, m_dt, 0, 0.5 * m_dt * m_dt},
            {0, 0, 1, 0, m_dt, 0},
            {0, 0, 0, 1, 0, m_dt},
            {0, 0, 0, 0, 1, 0},
            {0, 0, 0, 0, 0, 1}
    };

    H = {
            {1, 0, 0, 0, 0, 0},
            {0, 1, 0, 0, 0, 0}
    };

    R = {
            {50, 0},
            {0, 50}
    };

    double noise_ax = 0.8;
    double noise_ay = 0.8;
    Q = {
            {m_dt * m_dt * m_dt * m_dt / 4 * noise_ax, 0, m_dt * m_dt * m_dt / 2 * noise_ax, 0, m_dt * m_dt * noise_ax, 0},
            {0, m_dt * m_dt * m_dt * m_dt / 4 * noise_ay, 0, m_dt * m_dt * m_dt / 2 * noise_ay, 0, m_dt * m_dt * noise_ay},
            {m_dt * m_dt * m_dt / 2 * noise_ax, 0, m_dt * m_dt * noise_ax, 0, m_dt * noise_ax, 0},
            {0, m_dt * m_dt * m_dt / 2 * noise_ay, 0, m_dt * m_dt * noise_ay, 0, m_dt * noise_ay},
            {m_dt * m_dt * noise_ax, 0, m_dt * noise_ax, 0, 1, 0},
            {0, m_dt * m_dt * noise_ay, 0, m_dt * noise_ay, 0, 1}
    };

    P = identity(6);
    for (size_t i = 0; i < P.size(); ++i) {
        P[i][i] *= 1e-4;
    }

    x = std::vector<double>(6, 0);
}

std::vector<std::vector<double>> KalmanFilter::matrixMultiply(const std::vector<std::vector<double>>& A, const std::vector<std::vector<double>>& B) {
    assert(A[0].size() == B.size());
    std::vector<std::vector<double>> C(A.size(), std::vector<double>(B[0].size(), 0));
    for (size_t i = 0; i < A.size(); ++i) {
        for (size_t j = 0; j < B[0].size(); ++j) {
            for (size_t k = 0; k < B.size(); ++k) {
                C[i][j] += A[i][k] * B[k][j];
            }
        }
    }
    return C;
}

std::vector<double> KalmanFilter::matrixVectorMultiply(const std::vector<std::vector<double>>& A, const std::vector<double>& B) {
    assert(A[0].size() == B.size());
    std::vector<double> C(A.size(), 0);
    for (size_t i = 0; i < A.size(); ++i) {
        for (size_t j = 0; j < B.size(); ++j) {
            C[i] += A[i][j] * B[j];
        }
    }
    return C;
}

std::vector<std::vector<double>> KalmanFilter::transpose(const std::vector<std::vector<double>>& A) {
    std::vector<std::vector<double>> AT(A[0].size(), std::vector<double>(A.size()));
    for (size_t i = 0; i < A.size(); ++i) {
        for (size_t j = 0; j < A[0].size(); ++j) {
            AT[j][i] = A[i][j];
        }
    }
    return AT;
}

std::vector<std::vector<double>> KalmanFilter::inverse(const std::vector<std::vector<double>>& A) {
    assert(A.size() == 2 && A[0].size() == 2);
    double det = A[0][0] * A[1][1] - A[0][1] * A[1][0];
    assert(det != 0);
    std::vector<std::vector<double>> invA = {
            {A[1][1] / det, -A[0][1] / det},
            {-A[1][0] / det, A[0][0] / det}
    };
    return invA;
}

std::vector<std::vector<double>> KalmanFilter::identity(size_t size) {
    std::vector<std::vector<double>> I(size, std::vector<double>(size, 0));
    for (size_t i = 0; i < size; ++i) {
        I[i][i] = 1;
    }
    return I;
}




