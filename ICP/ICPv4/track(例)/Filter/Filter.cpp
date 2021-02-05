#include "Filter.h"
#include <iostream>
#include <unsupported/Eigen/MatrixFunctions>

namespace VTracker
{
    Filter::Filter()
    {
        bInit_ = false;
        prev_ts_ = -1.0;

        Eigen::Matrix<float, Q_SIZE, 1> Q_vars;
        Q_vars << std::pow(1e-2 * M_PI / 180.0, 2), std::pow(2e-4, 2), std::pow(2e-4, 2), //omega, linear velocity x, linear velocity y
                  std::pow(1.0 * M_PI / 180.0, 2), std::pow(2e-2, 2), std::pow(2e-2, 2), //omega, linear velocity x, linear velocity y
                  std::pow(2.0, 2), std::pow(2.0, 2); //linear velocity x, linear velocity y
        noise_params_.Q_ = Q_vars.asDiagonal();

        Eigen::Matrix<float, STATE_SIZE, 1> P_vars;
        P_vars << 1e-2,         //R
                  100.0, 100.0,     //P
                  1e-4,         //omega
                  1e0, 1e0,   //v
                  1e1, 1e1,   //a
                  1e-2;         //width
        noise_params_.init_var = P_vars.asDiagonal();

        noise_params_.u_var = std::pow(1.0/1458.0, 2); //visual var in pixel
        noise_params_.w_var = std::pow(10.0/1458.0, 2); //width var in pixel
        noise_params_.center_var = std::pow(10.0/1458, 2);
        noise_params_.dist_err_ratio = 0.3;
        return;
    }

    void Filter::initFilterState(const VState& initState, const Obs& obs)
    {
        state_ = initState;
        state_covar_ = noise_params_.init_var;
        augmentState();

        for(const auto& it : obs.us)
        {
            feature_map_[it.first] = it.second;
        }
        prev_ts_ = obs.ts;

        bInit_ = true;
        return;
    }

    void Filter::obsCallBack(const Obs& obs)
    {
        if(!bInit_)
        {
            std::cerr << "check filter initialization before update" << std::endl;
            return;
        }
        double dt = obs.ts - prev_ts_;

        propagate(dt);

        visualUpdate(obs);

        augmentState();

        prev_ts_ = obs.ts;
        return;
    }

    void Filter::propagate(const double& dt)
    {
        VState predict_state = predict(state_, dt);

        calcF();
        calcG();

	    // F * dt
	    F_ *= dt;

	    // Matrix exponential
	    Phi_ = F_.exp();

	    Eigen::Matrix<float, STATE_SIZE, STATE_SIZE> covar_prop =
	    	Phi_ * (state_covar_ + G_ * noise_params_.Q_ * G_.transpose() * dt) * Phi_.transpose();

	    // Apply updates directly
	    state_ = predict_state;
    	state_covar_ = (covar_prop + covar_prop.transpose()) / 2.0;
    	combine_covar_ = Phi_ * combine_covar_;

        return;
    }

    VState Filter::predict(const VState state_k, const double& dt)
    {
        VState predict_state = state_k;

        predict_state.v_V += (state_k.a_V * dt);
        predict_state.p_V += (state_k.v_V * dt);
        predict_state.theta += (state_k.omega * dt);
        return predict_state;
    }


    void Filter::calcF()
    {
        F_.setZero();
        F_.block<1,1>(0, 3).setIdentity(); //theta_omega
        F_.block<2,2>(1, 4).setIdentity(); //P_V
        F_.block<2,2>(4, 6).setIdentity(); //P_V
        return;
    }

    void Filter::calcG()
    {
        G_.setZero();

        G_.block<1,1>(0,0).setIdentity();
        G_.block<2,2>(1,1).setIdentity();

        G_.block<1,1>(3,3).setIdentity();
        G_.block<2,2>(4,4).setIdentity();

        G_.block<2,2>(6,6).setIdentity();

        return;
    }

    void Filter::augmentState()
    {
        prev_state_ = state_;

        Eigen::MatrixXf J = Eigen::MatrixXf::Zero(3, STATE_SIZE);
        J.block<1,1>(0, 0).setIdentity();
        J.block<2,2>(1,1).setIdentity();

    	Eigen::MatrixXf tempMat = Eigen::MatrixXf(STATE_SIZE + 3, STATE_SIZE);
	    tempMat.topRows(STATE_SIZE).setIdentity();
	    tempMat.bottomRows(3) = J;

        Eigen::MatrixXf P = state_covar_;
	    Eigen::MatrixXf P_aug = tempMat * P * tempMat.transpose();
	    Eigen::MatrixXf P_aug_sym = (P_aug + P_aug.transpose()) * 0.5;
	    P_aug = P_aug_sym;

        state_covar_ = P_aug.block<STATE_SIZE, STATE_SIZE>(0,0);
        prev_covar_ = P_aug.bottomRightCorner(3,3);
        combine_covar_ = P_aug.topRightCorner(STATE_SIZE,3);
        return;
    }


    void Filter::visualUpdate(const Obs& obs)
    {
            
        Eigen::Vector2f prev_oc = - (rotMatrix(prev_state_.theta).transpose() * prev_state_.p_V);

        std::vector<float> rs;
        std::vector<Eigen::MatrixXf> Hs_prev, Hs_curr;
        std::vector<Eigen::MatrixXf> Rs;

        std::unordered_map<int, float> tmp_feature_map;

        for (int i = 0; i < obs.us.size(); i++)
        {
            const std::pair<int, float>& it = obs.us[i];
            tmp_feature_map[it.first] = it.second;

            if (!feature_map_.count(it.first))
            {
                continue;
            }

            float prev_u = feature_map_[it.first];
            float u = it.second;

            Eigen::Vector2f prev_v_tmp(1.0, prev_u);
            Eigen::Vector2f prev_v = rotMatrix(prev_state_.theta).transpose() * prev_v_tmp;

            float a = prev_v(1) / prev_v(0);
            Eigen::Vector2f pc(0, prev_oc(1) - a * prev_oc(0));
            Eigen::Vector2f vc = rotMatrix(state_.theta) * pc;
            Eigen::Vector2f curr_pw = vc + state_.p_V;
            float est_u = curr_pw(1) / curr_pw(0);

            float err = u - est_u;

            Eigen::MatrixXf g0(1,2);
            g0 << - curr_pw(1) / (curr_pw(0) * curr_pw(0)), 1.0 / curr_pw(0);


            //prevH
            Eigen::MatrixXf H_prev(1,3);
            H_prev.setZero();

            Eigen::MatrixXf g1 = g0 * rotMatrix(state_.theta);

            //pc _ oc
            Eigen::MatrixXf G_pc_oc(2,2);
            G_pc_oc.row(0).setZero();
            G_pc_oc(1,0) = -a;
            G_pc_oc(1,1) = 1;

            //pc _ a
            Eigen::MatrixXf G_pc_a(2,1);
            G_pc_a << 0, - prev_oc(0);

            // prev_oc _ prev_pv
            Eigen::MatrixXf G_oc_pv = - rotMatrix(prev_state_.theta).transpose();
            // prev_oc _ prev_rv
            Eigen::MatrixXf G_oc_rv(2,1);
            G_oc_rv << prev_oc(1), -prev_oc(0);

            // a _ prev_v
            Eigen::MatrixXf G_a_v(1,2); 
            G_a_v << - prev_v(1) / (prev_v(0) * prev_v(0)), 1.0 / prev_v(0);
            //prev_v _ prev_rv
            Eigen::MatrixXf G_prevv_rv(2,1);
            G_prevv_rv << prev_v(1), - prev_v(0);

            H_prev.block<1,1>(0,0) = g1 * (G_pc_oc * G_oc_rv + G_pc_a * G_a_v * G_prevv_rv);
            H_prev.block<1,2>(0,1) = g1 * (G_pc_oc * G_oc_pv); 

            //currH
            Eigen::MatrixXf H_curr(1, STATE_SIZE);
            H_curr.setZero();

            //vc_rv
            Eigen::MatrixXf G_vc_rv(2,1);
            G_vc_rv = rotMatrix(state_.theta) * Eigen::Vector2f(-pc(1), pc(0));

            H_curr.block<1,1>(0,0) = g0 * G_vc_rv;
            H_curr.block<1,2>(0,1) = g0;

            Eigen::MatrixXf R(1,1);
            R(0,0) = noise_params_.u_var;

            rs.push_back(err);
            Hs_prev.push_back(H_prev);
            Hs_curr.push_back(H_curr);
            Rs.push_back(R);
        }

        if (obs.wv_img > 0)
        {
            Eigen::Vector2f pl(0, -0.5 * state_.w_V);
            Eigen::Vector2f pr(0,  0.5 * state_.w_V);

            Eigen::Vector2f pwl = rotMatrix(state_.theta) * pl + state_.p_V;
            Eigen::Vector2f pwr = rotMatrix(state_.theta) * pr + state_.p_V;

            Eigen::Vector2f pcl = pwl / pwl(0);
            Eigen::Vector2f pcr = pwr / pwr(0);

            float w = pcr(1) - pcl(1);

            float err = obs.wv_img - w;
            // std::cout << obs.wv_img << ", " << w << ", " << err << std::endl;

            Eigen::MatrixXf H_curr(1, STATE_SIZE);
            H_curr.setZero();

            Eigen::MatrixXf Gr(1,2);
            Gr << - pwr(1) / (pwr(0) * pwr(0)), 1.0 / pwr(0);

            Eigen::MatrixXf Gl(1,2);
            Gl <<  pwl(1) / (pwl(0) * pwl(0)), -1.0 / pwl(0);

            // pwr _ Rv
            Eigen::MatrixXf G_pwr_rv = rotMatrix(state_.theta) * Eigen::Vector2f(-pr(1), pr(0));

            // pwl _ Rv
            Eigen::MatrixXf G_pwl_rv = rotMatrix(state_.theta) * Eigen::Vector2f(-pl(1), pl(0));

            //pwr _ pv
            Eigen::MatrixXf G_pwr_pv(2,2);
            G_pwr_pv.setIdentity();

            //pwl _ pv
            Eigen::MatrixXf G_pwl_pv(2,2);
            G_pwl_pv.setIdentity();

            //pwr _ p
            Eigen::MatrixXf G_pw_p = rotMatrix(state_.theta);

            //pr _ w
            Eigen::MatrixXf G_pr_w(2,1);
            G_pr_w << 0, 0.5;

            //pl _ w
            Eigen::MatrixXf G_pl_w(2,1);
            G_pl_w << 0, -0.5;

            H_curr.block<1,1>(0,0) = Gr * G_pwr_rv + Gl * G_pwl_rv;
            H_curr.block<1,2>(0,1) = Gr * G_pwr_pv + Gl * G_pwl_pv;
            H_curr.block<1,1>(0,8) = Gr * G_pw_p * G_pr_w + Gl * G_pw_p * G_pl_w;

            // std::cout << G_pr_w << ", " << G_pl_w << std::endl;

            Eigen::MatrixXf R(1,1);
            R(0,0) = noise_params_.w_var;

            rs.push_back(err);
            Hs_curr.push_back(H_curr);
            Rs.push_back(R);
            
            // std::cout << state_.w_V << ", " <<  w << ", " << obs.wv_img << std::endl;
        }

        if (obs.dist > 0)
        {
            float err = obs.dist - state_.p_V(0);

            Eigen::MatrixXf H_curr(1, STATE_SIZE);
            H_curr.setZero();

            H_curr(0, 1) = 1.0;

            Eigen::MatrixXf R(1,1);
            R(0,0) = std::pow(obs.dist * noise_params_.dist_err_ratio, 2);

            rs.push_back(err);
            Hs_curr.push_back(H_curr);
            Rs.push_back(R);
        }

        if (obs.ur > -0.5 && obs.ur < 0.5 && obs.ul > -0.5 && obs.ul < 0.5)
        {
            const Eigen::Vector2f& pv = state_.p_V;
            float mid = 0.5 * (obs.ur + obs.ul);
            float err = mid - pv(1) / pv(0);

            Eigen::MatrixXf H_curr(1, STATE_SIZE);
            H_curr.setZero();
            H_curr.block<1,2>(0, 1) << -pv(1) / (pv(0) * pv(0)), 1.0 / pv(0);

            Eigen::MatrixXf R(1,1);
            R(0, 0) = noise_params_.center_var;

            rs.push_back(err);
            Hs_curr.push_back(H_curr);
            Rs.push_back(R);
        }
        

        if (!rs.empty())
        {
            Eigen::VectorXf r(rs.size());
            r.setZero();
            Eigen::MatrixXf H(rs.size(), STATE_SIZE + 3);
            H.setZero();
            Eigen::MatrixXf R(rs.size(), rs.size());
            R.setIdentity();

            for (int i = 0; i < rs.size(); i++)
            {
                r(i) = rs[i];
                if(i < Hs_prev.size())
                {
                    H.block<1,3>(i, STATE_SIZE)= Hs_prev[i];
                }
                H.block<1,STATE_SIZE>(i, 0) = Hs_curr[i];
                R.block<1,1>(i,i) = Rs[i];
            }

            measurementUpdate(r, H, R);
        }

        feature_map_ = tmp_feature_map;
        return;
    }

	void Filter::measurementUpdate(const Eigen::VectorXf &r, Eigen::MatrixXf &H, const Eigen::MatrixXf &R, const std::string &fun_name)
    {
	    if (0 == r.size()) 
        {
	    	return;
	    }

	    Eigen::MatrixXf P;
	    {
	    	P = Eigen::MatrixXf::Zero(STATE_SIZE + 3, STATE_SIZE + 3);

	    	P.block<STATE_SIZE, STATE_SIZE>(0, 0) = state_covar_;
	    	P.block(0, STATE_SIZE, STATE_SIZE, 3) = combine_covar_;
	    	P.block(STATE_SIZE, 0, 3, STATE_SIZE) = combine_covar_.transpose();
	    	P.block(STATE_SIZE, STATE_SIZE, 3, 3) = prev_covar_;
	    }

	    Eigen::MatrixXf temp0, temp1, K;
	    Eigen::VectorXf deltaX;

        {
		    temp0 = P * H.transpose();
		    temp1 = H * temp0 + R;
		    K = temp0 * temp1.inverse();
            deltaX = K * r;
        }

        state_.theta += deltaX(0);
        state_.p_V += deltaX.segment<2>(1);
        state_.omega += deltaX(3);
        state_.v_V += deltaX.segment<2>(4);
        state_.a_V += deltaX.segment<2>(6);
        state_.w_V += deltaX(8);


	    Eigen::MatrixXf tempMat ;
	    {
	    	tempMat=Eigen::MatrixXf::Identity(STATE_SIZE + 3, STATE_SIZE + 3) -
	    			K * H;
	    }

	    Eigen::MatrixXf P_corrected, P_corrected_transpose;
	    {
	    	P_corrected = tempMat * P * tempMat.transpose() + K * R * K.transpose();
            P_corrected_transpose = P_corrected.transpose();
	        P_corrected += P_corrected_transpose;
            P_corrected *= 0.5;
	    }

    	state_covar_ = P_corrected.block<STATE_SIZE, STATE_SIZE>(0, 0);

    	prev_covar_ = P_corrected.block(STATE_SIZE, STATE_SIZE, P_corrected.rows() - STATE_SIZE,
    								   P_corrected.cols() - STATE_SIZE);
    	combine_covar_ = P_corrected.block(0, STATE_SIZE, STATE_SIZE, P_corrected.cols() - STATE_SIZE);

        return;
    }

    VState Filter::getState()
    {
        return state_;
    }

    Eigen::Matrix2f rotMatrix(float theta)
    {
        Eigen::Matrix2f R;
        R << std::cos(theta), -std::sin(theta),
            std::sin(theta), std::cos(theta);
        return R;
    }

}



