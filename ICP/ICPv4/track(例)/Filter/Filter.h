#ifndef FILTER_H
#define FILTER_H

#include <vector>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <unordered_map>

#define STATE_SIZE  9
#define Q_SIZE 8

namespace VTracker
{
    Eigen::Matrix2f rotMatrix(float theta);

    struct Obs
    {
        std::vector<std::pair<int, float>> us;

        float wv_img = -1.0;

        float dist = -1.0;

        float ul = -100.0;

        float ur = 100.0;

        double ts = -1.0;
    };

    struct VState
    {
        float theta = 0;                                        //0, 1
        Eigen::Vector2f p_V = Eigen::Vector2f(10, 0);           //1, 2
        float omega = 0;                                        //3, 1
        Eigen::Vector2f v_V = Eigen::Vector2f(0, 0);            //4, 2
        Eigen::Vector2f a_V = Eigen::Vector2f(0, 0);            //6, 2
        float w_V = 1.5;                                        //8, 1
        double ts = -1.0;
    };

    struct noiseParams
    {
    	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    	float u_var = 1e-6;
    	float w_var = 1e-2;
        float center_var = 1e-6;
        float dist_err_ratio = 0.2;
    	Eigen::Matrix<float, Q_SIZE, Q_SIZE> Q_ = Eigen::Matrix<float, Q_SIZE, Q_SIZE>::Identity();
    	Eigen::Matrix<float, STATE_SIZE, STATE_SIZE> init_var =
    		Eigen::Matrix<float, STATE_SIZE, STATE_SIZE>::Identity();
    };


    class Filter
    {
    public:
        Filter();

        void obsCallBack(const Obs& obs);

        void initFilterState(const VState& initState, const Obs& obs);

        VState getState();

    private:
        void propagate(const double& dt);

        VState predict(const VState state_k, const double& dt);

        void calcF();

        void calcG();

        void augmentState();

        void visualUpdate(const Obs& obs);

        void measurementUpdate(const Eigen::VectorXf &r, Eigen::MatrixXf &H, const Eigen::MatrixXf &R,
				const std::string &fun_name="default");


        bool bInit_;

        noiseParams noise_params_;

        VState state_, prev_state_;

        double prev_ts_;

        std::unordered_map<int, float> feature_map_;

		Eigen::Matrix<float, STATE_SIZE, STATE_SIZE> state_covar_;
		Eigen::Matrix<float, STATE_SIZE, STATE_SIZE> F_;
		Eigen::Matrix<float, STATE_SIZE, STATE_SIZE> Phi_;
		Eigen::Matrix<float, STATE_SIZE, Q_SIZE> G_;

        Eigen::Matrix<float, 3, 3> prev_covar_;
		Eigen::Matrix<float, STATE_SIZE, 3> combine_covar_;
    };

};

#endif