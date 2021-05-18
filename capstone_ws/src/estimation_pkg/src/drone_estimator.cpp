#pragma once
#include "../include/estimation_pkg/drone_estimator.hpp"

KF_drone_estimator::KF_drone_estimator()
{
    A.resize(6,6);
    F.resize(6,6);
    H.resize(6,6);
    Q.resize(6,6);
    R.resize(3,3);
    x_hat.resize(6,1);
    x_tilde.resize(6,1);
    x_hat_t.resize(6,1);
    P_hat.resize(6,6);
    P_tilde.resize(6,6);
    K.resize(6,3);
    mat_temp.resize(3,3);
    z.resize(3,1);

    result_position.resize(3);

    ////////////////////////// Parameter Initialize //////////////////////////////
    m_dt = 0.;
    m_dt_old = 0.;
    m_xf = 0.;
    m_yf = 0.;
    m_zf = 0.;

    cnt_ = 0;
    old_cnt_ = 0;

    call_kalman = false;
    thread_join = false;

    A << 1, m_dt, 0, 0,0,0,
        0, 1, 0, 0,0,0,
        0, 0, 1, m_dt,0,0,
        0, 0, 0, 1,0,0,
        0, 0, 0, 0, 1, m_dt,
        0, 0, 0, 0, 0, 1;
    F << 1, 0.01, 0, 0,0,0,
        0, 1, 0, 0,0,0,
        0, 0, 1, 0.01,0,0,
        0, 0, 0, 1,0,0,
        0, 0, 0, 0, 1, 0.01,
        0, 0, 0, 0, 0, 1;

    H << 1, 0, 0, 0, 0, 0,
        0, 0, 1, 0, 0, 0,
        0, 0, 0, 0, 1, 0;

    Q << (double)(0.00000001/4), (double)(0.000001/2), 0, 0, 0, 0,
        (double)(0.000001/2), 0.0001, 0, 0,0,0,
        0, 0, (double)(0.00000001/4), (double)(0.000001/2),0,0,
        0, 0, (double)(0.000001/2), 0.0001,0,0,
        0,0,0,0,(double)(0.00000001/4), (double)(0.000001/2),
        0,0,0,0,(double)(0.000001/2), 0.0001;

    Q = 70225*Q;

    x_hat << 1, 0, 1, 0, 1, 0;
    x_hat_t = x_hat;

    R << 1, 0, 0, 
        0, 1.1,0,
        0,0,0.9;

    R = 0.5*R;

    P_hat << 1, 0, 0, 0, 0, 0, 
            0, 1, 0, 0, 0, 0,
            0, 0, 1, 0, 0, 0,
            0, 0, 0, 1, 0, 0,
            0, 0, 0, 0, 1, 0,
            0, 0, 0, 0, 0, 1;
    //////////////////////////////////////////////////////////////////////////////////
}

void KF_drone_estimator::AddObservation(Eigen::Vector3d postion)
{
    m_dt = (double)0.05*(cnt_ - old_cnt_);
    old_cnt_ = cnt_;

    z(0) = postion[0];
    z(1) = postion[1];
    z(2) = postion[2];

    A <<1,  m_dt,   0,     0,    0,    0,
        0,  1,      0,     0,    0,    0,
        0,  0,      1,     m_dt, 0,    0,
        0,  0,      0,     1,    0,    0,
        0,  0,      0,     0,    1,    m_dt,
        0,  0,      0,     0,    0,    1;

    x_tilde = A*x_hat;
    P_tilde = A*P_hat*A.transpose() + Q;

    K = P_tilde*H.transpose()*(H*P_tilde*H.transpose() + R).inverse();

    x_hat = x_tilde + K*(z - H*x_tilde);
    P_hat = P_tilde - K*H*P_tilde;

    call_kalman = true;   

}


void KF_drone_estimator::excute_timerThread()
{
    while(!thread_join)
    {
        mutex_.lock();

        cnt_++;

        if(cnt_ > 30000) {
            cnt_ = cnt_ - old_cnt_;
            old_cnt_ = 0;
        }
        if(call_kalman == true) {
            x_hat_t = x_hat;
            call_kalman = false;
        }
        else {
            x_hat_t = F*x_hat_t;
        }

        result_position(0) = x_hat_t(0);
        result_position(1)= x_hat_t(2);
        result_position(2) = x_hat_t(4);

        mutex_.unlock();
        std::this_thread::sleep_for(std::chrono::microseconds(50));
    }
}