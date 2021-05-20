#pragma once
#include "../include/estimation_pkg/drone_estimator.hpp"

// YAMLConfig &yaml_config
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
    T_BO_result_position.resize(3);

    ////////////////////////// Parameter Initialize //////////////////////////////
    m_dt = 0.;
    m_dt_old = 0.;
    m_xf = 0.;
    m_yf = 0.;
    m_zf = 0.;

    target_Y = 0.0;
    target_P = 0.0;
    target_tilt = 0.0;

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

    //////// HardWare Param Init ////////////////////////////
    //// RBDL 설정시 대체할 예정  ////////////////////////////

    base_frame_link_.resize(4,4);
    rail_frame_link_.resize(4,4);
    pitch_frame_link_.resize(4,4);
    camera_pitch_frame_link_.resize(4,4);
    camera_frame_link_.resize(4,4);

    Eigen::Vector3f p(0,0,0);
    Eigen::Matrix3f R = Eigen::Quaternionf(1,0,0,0).toRotationMatrix();
    base_frame_link_ <<  R, p,
                         0,0,0 ,1;

    p = Eigen::Vector3f(-0.39365,0.050581,0.3829);
    R = Eigen::Quaternionf(-0.0302588,-0.0302562,0.706459,0.706459).toRotationMatrix();
    rail_frame_link_ <<  R, p,
                        0,0,0 ,1;

    p = Eigen::Vector3f(-0.39135,0.023178,0.4224);
    R = Eigen::Quaternionf(0.478147,0.478149,0.520937,0.520935).toRotationMatrix();
    pitch_frame_link_ <<  R, p,
                        0,0,0 ,1;

    p = Eigen::Vector3f(-0.39147,0.024476,0.6755);
    R = Eigen::Quaternionf(0.520934,-0.520936,0.47815,-0.478149).toRotationMatrix();
    camera_pitch_frame_link_ <<  R, p,
                        0,0,0 ,1;

    p = Eigen::Vector3f(-0.39059,0.065702,0.6765);
    R = Eigen::Quaternionf(-0.0302549,-0.0302523,0.706459,0.706459).toRotationMatrix();
    camera_frame_link_ <<  R, p,
                        0,0,0 ,1;

    ///////////////////////////////////////////////////////////////////////////////////////
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

// void KF_drone_estimator::initModel()
// {
//     std::string urdf_absolute_path;
// 	std::string mod_url = config_.urdf_path;

//     if (config_.urdf_path.find("package://") == 0)
// 	{
// 		mod_url.erase(0, strlen("package://"));
// 		size_t pos = mod_url.find("/");
// 		if (pos == std::string::npos)
// 		{
// 			std::cout << "Could not parse package:// format into file:// format" << std::endl;;
// 		}
// 		std::string package = mod_url.substr(0, pos);
// 		mod_url.erase(0, pos);
// 		std::string package_path = ros::package::getPath(package);

// 		if (package_path.empty())
// 		{
// 			std::cout << "Package does not exist" << std::endl;;
// 		}

// 		urdf_absolute_path =  package_path + mod_url;
// 	}

//     RigidBodyDynamics::Addons::URDFReadFromFile(urdf_absolute_path.c_str(), &rbdl_model_, false, false);
//     rail_frame_end_id_ = rbdl_model_.GetBodyId((config_.chain_end).c_str());
//     base_frame_start_id_ = rbdl_model_.GetBodyId((config_.chain_start).c_str());
// }


void KF_drone_estimator::excute_timerThread()
{
    while(!thread_join)
    {
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
        
        T_BO_result_position = rail_frame_link_*result_position;

        Eigen::Vector3f rail_pos(rail_frame_link_(0,3), rail_frame_link_(1,3), rail_frame_link_(2,3));
        Eigen::Vector3f relative_pitch_pos(2.6947e-05,-0.00030225,0.156);

        double a =  sqrt( pow(T_BO_result_position(0),2) + pow(T_BO_result_position(2),2) );
        double b =  sqrt( pow(T_BO_result_position(0)-relative_pitch_pos(0),2) + pow(T_BO_result_position(2)-relative_pitch_pos(2),2) );
        double c =  sqrt( pow(relative_pitch_pos(0),2) + pow(relative_pitch_pos(2),2) );
        double gamma = acos( ( (b*b)+(c*c)-(a*a) )/2*b );
        double beta = atan2(result_position(2), sqrt(result_position(0)*result_position(0)+result_position(1)*result_position(1)));
        
        mutex_.lock();
        target_Y = atan2(T_BO_result_position(0),T_BO_result_position(1));
        target_P = gamma;
        target_tilt = beta;
        mutex_.unlock();
        std::this_thread::sleep_for(std::chrono::microseconds(500));
    }
}