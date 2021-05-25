#pragma once
#include "../include/estimation_pkg/drone_estimator.hpp"

// YAMLConfig &yaml_config
KF_drone_estimator::KF_drone_estimator()
{
    A.resize(6,6);
    A_hat.resize(6,6);
    H.resize(3,6);
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

    T_BC.resize(4,4);
    
    result_position.resize(4);
    result_position(0) = 1;
    result_position(1) = 0;
    result_position(2) = 0;
    result_position(3) = 1;
    
    T_BO_result_position.resize(4);

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
    lost_cnt = 0;

    call_kalman = false;
    call_lost = false;
    thread_join = false;

    A << 1, m_dt, 0, 0, 0, 0,
        0,  1,    0, 0, 0, 0,
        0, 0, 1, m_dt,0,0,
        0, 0, 0, 1,0,0,
        0, 0, 0, 0, 1, m_dt,
        0, 0, 0, 0, 0, 1;

    A_hat << 1, 0.05, 0, 0, 0, 0,
            0,  1,    0, 0, 0, 0,
            0, 0, 1, 0.05,0,0,
            0, 0, 0, 1,0,0,
            0, 0, 0, 0, 1, 0.05,
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

    Q = 62500*Q;

    x_hat << 1, 0, 1, 0, 1, 0;
    x_hat_t = x_hat;

    R << 1, 0, 0, 
        0, 1,0,
        0,0,0.9;

    R = 0.3*R;

    P_hat << 1, 0, 0, 0, 0, 0, 
            0, 1, 0, 0, 0, 0,
            0, 0, 1, 0, 0, 0,
            0, 0, 0, 1, 0, 0,
            0, 0, 0, 0, 1, 0,
            0, 0, 0, 0, 0, 1;

    //////// HardWare Param Init ////////////////////////////
    //// RBDL 설정시 대체할 예정  ////////////////////////////

    /////// kinematic chain set/////////////////////////////////////
    cam_chain_ = KDL::Chain();
    rail_chain_ = KDL::Chain();

    base_frame_.p(0) = 0.;
    base_frame_.p(1) = 0.;
    base_frame_.p(2) = 0.;
    Eigen::MatrixXf rot = Eigen::Quaternionf(1,0,0,0).toRotationMatrix();
    rot_.data[0] = rot(0, 0);
	rot_.data[1] = rot(0, 1);
	rot_.data[2] = rot(0, 2);
	rot_.data[3] = rot(1, 0);
	rot_.data[4] = rot(1, 1);
	rot_.data[5] = rot(1, 2);
	rot_.data[6] = rot(2, 0);
	rot_.data[7] = rot(2, 1);
	rot_.data[8] = rot(2, 2);
    base_frame_.M = rot_;

    yaw_frame_.p(0) = 0.00087547;
    yaw_frame_.p(1) = -0.010086;
    yaw_frame_.p(2) = 0.2234;
    rot = Eigen::Quaternionf(0.999964,0,0,0.0085224).toRotationMatrix();
    rot_.data[0] = rot(0, 0);
	rot_.data[1] = rot(0, 1);
	rot_.data[2] = rot(0, 2);
	rot_.data[3] = rot(1, 0);
	rot_.data[4] = rot(1, 1);
	rot_.data[5] = rot(1, 2);
	rot_.data[6] = rot(2, 0);
	rot_.data[7] = rot(2, 1);
	rot_.data[8] = rot(2, 2);
    yaw_frame_.M = rot_;

    rail_frame_.p(0) = 0.0;
    rail_frame_.p(1) = 0.0;
    rail_frame_.p(2) = -0.0395;
    rot = Eigen::Quaternionf(1,0,0,0).toRotationMatrix();
    rot_.data[0] = rot(0, 0);
	rot_.data[1] = rot(0, 1);
	rot_.data[2] = rot(0, 2);
	rot_.data[3] = rot(1, 0);
	rot_.data[4] = rot(1, 1);
	rot_.data[5] = rot(1, 2);
	rot_.data[6] = rot(2, 0);
	rot_.data[7] = rot(2, 1);
	rot_.data[8] = rot(2, 2);
    rail_frame_.M = rot_;

    pitch_frame_.p(0) = -0.00030345;
    pitch_frame_.p(1) = 0.0;
    pitch_frame_.p(2) = 0.156;
    rot = Eigen::Quaternionf(1,0,0,0).toRotationMatrix();
    rot_.data[0] = rot(0, 0);
	rot_.data[1] = rot(0, 1);
	rot_.data[2] = rot(0, 2);
	rot_.data[3] = rot(1, 0);
	rot_.data[4] = rot(1, 1);
	rot_.data[5] = rot(1, 2);
	rot_.data[6] = rot(2, 0);
	rot_.data[7] = rot(2, 1);
	rot_.data[8] = rot(2, 2);
    pitch_frame_.M = rot_;

    cam_pitch_frame_.p(0) = 0.001; 
    cam_pitch_frame_.p(1) = 0.0;
    cam_pitch_frame_.p(2) = 0.4091;
    rot = Eigen::Quaternionf(1,0,0,0).toRotationMatrix();
    rot_.data[0] = rot(0, 0);
	rot_.data[1] = rot(0, 1);
	rot_.data[2] = rot(0, 2);
	rot_.data[3] = rot(1, 0);
	rot_.data[4] = rot(1, 1);
	rot_.data[5] = rot(1, 2);
	rot_.data[6] = rot(2, 0);
	rot_.data[7] = rot(2, 1);
	rot_.data[8] = rot(2, 2);
    cam_pitch_frame_.M = rot_;

    cam_frame_.p(0) = 0.041; 
    cam_frame_.p(1) = -0.0043929;
    cam_frame_.p(2) = 0.0010012;
    rot = Eigen::Quaternionf(1,0,0,0).toRotationMatrix();
    rot_.data[0] = rot(0, 0);
	rot_.data[1] = rot(0, 1);
	rot_.data[2] = rot(0, 2);
	rot_.data[3] = rot(1, 0);
	rot_.data[4] = rot(1, 1);
	rot_.data[5] = rot(1, 2);
	rot_.data[6] = rot(2, 0);
	rot_.data[7] = rot(2, 1);
	rot_.data[8] = rot(2, 2);
    cam_frame_.M = rot_;

    rail_chain_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None),KDL::Frame(base_frame_)));
    rail_chain_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame(yaw_frame_)));
    rail_chain_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY),KDL::Frame(pitch_frame_)));
    rail_chain_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None),KDL::Frame(rail_frame_)));

    cam_chain_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None),KDL::Frame(base_frame_)));
    cam_chain_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame(yaw_frame_)));
    cam_chain_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY),KDL::Frame(cam_pitch_frame_)));
    cam_chain_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None),KDL::Frame(cam_frame_)));

    cam_joint_val_ = KDL::JntArray(cam_chain_.getNrOfJoints());
    rail_joint_val_ =  KDL::JntArray(rail_chain_.getNrOfJoints());

    // std::cout<<"joint number"<<cam_chain_.getNrOfJoints()<<std::endl;

    for(unsigned int i = 0; i <cam_chain_.getNrOfJoints(); i++ )
        cam_joint_val_(i) = 0.0;

    for(unsigned int i = 0; i <rail_chain_.getNrOfJoints(); i++ )
        rail_joint_val_(i) = 0.0;
    ///////////////////////////////////////////////////////////////////////////////////////
}

void KF_drone_estimator::AddObservation(Eigen::Vector4f postion)
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
    //call_lost = false;
    //lost_cnt = 0;
    //std::cout<< "found drone" << std::endl;
}

Eigen::Matrix4f KF_drone_estimator::Frame2Eigen(KDL::Frame &frame)
{
    Eigen::Matrix4f H_trans;
    H_trans.resize(4,4);
    double d[16] = {0,};
    frame.Make4x4(d);

    H_trans(0,0) = d[0];
    H_trans(0,1) = d[1];
    H_trans(0,2) = d[2];
    H_trans(0,3) = d[3];
    H_trans(1,0) = d[4];
    H_trans(1,1) = d[5];
    H_trans(1,2) = d[6];
    H_trans(1,3) = d[7];
    H_trans(2,0) = d[8];
    H_trans(2,1) = d[9];
    H_trans(2,2) = d[10];
    H_trans(2,3) = d[11];
    H_trans(3,0) = d[12];
    H_trans(3,1) = d[13];
    H_trans(3,2) = d[14];
    H_trans(3,3) = d[15];

    return H_trans;
}

void KF_drone_estimator::UpdateKinematics()
{
    KDL::ChainFkSolverPos_recursive camFksolver = KDL::ChainFkSolverPos_recursive(cam_chain_);
    KDL::ChainFkSolverPos_recursive railFksolver = KDL::ChainFkSolverPos_recursive(rail_chain_);

    KDL::Frame T_BC_frame;
    KDL::Frame T_BP_frame;
    KDL::Frame T_BR_frame;

    if(camFksolver.JntToCart(cam_joint_val_,T_BC_frame)<0)
        std::cerr<<"Update Kinematics Failed"<<std::endl;

    T_BC = Frame2Eigen(T_BC_frame);
}
void KF_drone_estimator::excute_timerThread()
{
    while(!thread_join)
    {
        cnt_++;

        if(cnt_ > 30000) {
            cnt_ = cnt_ - old_cnt_;
            old_cnt_ = 0;
        }
        /*if(lost_cnt*0.05 > 0.5) {
            call_lost = true;
            old_cnt_ = cnt_;
            //std::cout<< "lost drone" << std::endl;
            if(lost_cnt > 20000) lost_cnt = 10000;
        }*/
        if(call_kalman == true) {
            x_hat_t = x_hat;
            call_kalman = false;
        }
        else{
            x_hat_t = A_hat * x_hat_t;
        }

        result_position(0) = x_hat_t(0);
        result_position(1)= x_hat_t(2);
        result_position(2) = x_hat_t(4);
        result_position(3) = 1;

        KDL::ChainFkSolverPos_recursive camFksolver = KDL::ChainFkSolverPos_recursive(cam_chain_);
        KDL::ChainFkSolverPos_recursive railFksolver = KDL::ChainFkSolverPos_recursive(rail_chain_);

        KDL::Frame T_BC_frame;
        KDL::Frame T_BP_frame;
        KDL::Frame T_BR_frame;
        
        // std::cout<<"cam_joint_Val"<<cam_joint_val_.data.transpose()<<std::endl;
        // std::cout<<"rail_joint_Val"<<rail_joint_val_.data.transpose()<<std::endl;

        /////////////////////////////Base to Drone

        //std::cout<<"mat T_BC \n"<<T_BC<<std::endl;

        T_BO_result_position = result_position; //4*4 matrix

        // std::cout<<"T_BC :"<<T_BC<<std::endl;
        ////////////////////////////Rail to Drone
        if(camFksolver.JntToCart(rail_joint_val_,T_BR_frame) < 0)
           std::cerr<<"Fk no solution exist"<<std::endl;
        
        Eigen::Matrix4f T_BR = Frame2Eigen(T_BR_frame);

        Eigen::Matrix4f T_RB;
        T_RB.block(0,0,3,3) = T_BR.block(0,0,3,3).inverse();
        T_RB.block(0,3,3,1) = -T_BR.block(0,0,3,3).inverse()* T_BR.block(0,3,3,1);
        T_RB(3,0) = 0; T_RB(3,1) = 0; T_RB(3,2) = 0; T_RB(3,3) = 1;

        Eigen::VectorXf T_RO_result_position;
        
        T_RO_result_position = T_RB*T_BO_result_position;

        // std::cout<<"result Pose(T_RO) :"<<T_RO_result_position.transpose()<<std::endl;
        // std::cout<<"result Pose(T_CO) :"<<result_position.transpose()<<std::endl;

        ///////////////////////////Base to Pitch

        if(camFksolver.JntToCart(rail_joint_val_,T_BP_frame,3)<0)
           std::cerr<<"Fk no solution exist"<<std::endl;
        
        Eigen::Matrix4f T_BP = Frame2Eigen(T_BP_frame);

        ///////////////////////////Pitch to Rail
        Eigen::Matrix4f T_PB;
        T_PB << T_BP.block(0,0,3,3).inverse(), 
               -T_BP.block(0,0,3,3).inverse()* T_BP.block(0,3,3,1)
               ,0,0,0,1;

        Eigen::Matrix4f T_PR = T_PB* T_BR;

        Eigen::VectorXf T_PO_result_position =  T_PR * T_RO_result_position;

        double a =  sqrt( pow(T_RO_result_position(0),2) + pow(T_RO_result_position(2),2) );
        double b =  sqrt( pow(T_RO_result_position(0)-T_PR(0,3),2) + pow(T_RO_result_position(2)-T_PR(2,3),2) );
        double c =  sqrt( pow(T_PO_result_position(2),2) + pow(T_PO_result_position(0),2));

        double alpha = atan2(T_BO_result_position(1),T_BO_result_position(0));
        double gamma = acos( ( (b*b)+(c*c)-(a*a) )/2*b );
        double beta = atan2(result_position(2), sqrt(result_position(0)*result_position(0)+result_position(1)*result_position(1)));
        
        // std::cout<< "Y  : "<<alpha<<std::endl;
        // std::cout<< "P  : "<<gamma<<std::endl;
        // std::cout<< "tilt  : "<<beta<<std::endl;

        mutex_.lock();

        target_Y = alpha;
        target_P = gamma;
        target_tilt = beta;

        mutex_.unlock();
        //std::cout << "Target_Y_in droneestimate = " << target_Y*180/M_PI << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}