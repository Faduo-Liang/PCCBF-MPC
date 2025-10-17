#ifndef MPC_H
#define MPC_H

#include <vector>
#include <eigen3/Eigen/Dense>
#include "tf2/utils.h"
#include <casadi/casadi.hpp>
#include <iostream>
#include <chrono>
#include <ros/ros.h>

using namespace std;
using namespace casadi;

class Mpc
{
private:
    //mpc params
    int N_;  //horizon
    double dt_;  //step
    //constrains
    double u_max_, u_min_;
    double w_max_, w_min_;
    
    //weights
    DM Q_, R_, W_, Q_N;

    // OBS
    vector<double> obst_states = vector<double> (64, 0); // number gamma x y (static obs)
    vector<double> dyn_obst_states ; // dynamic obs
    vector<double> obst_collision ; // x y
    vector<double> predict_states = vector<double> (62, 0);
    double safe_dist, obs_flag;

    MX X;
    MX U;
    
    Function kinematic_equation_;
    unique_ptr<casadi::OptiSol> solution_;
    bool mpc_success = true;
    

public:
    Mpc();
    ~Mpc();

    Function setKinematicEquation();
    void setWeights(vector<double> weights);
    bool solve(Eigen::VectorXd current_states, vector<double> desired_states);
    vector<double> getFirstU();
    vector<double> getAllU();
    vector<double> getPredictX();
    bool is_target(Eigen::VectorXd cur, vector<double> goal);
    MX h(MX states, const double &obs_x, const double &obs_y);
    MX h_dyn(MX states, const double &obs_x, const double &obs_y);
    vector<double> getObst(vector<double> obs_state, vector<double> dyn_obs_states);
    vector<double> getObst_collision();
};

#endif