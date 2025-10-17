#include "local_planner/mpc.h"

Mpc::Mpc() {
    N_ = 30;
    dt_ = 0.1;
    u_max_ = 1.2; // max: 0.3 TB4 / 1.2 jackal
    w_max_ = 1.2; // max: 1.89
    vector<double> weights = {1,1,0.02,1.2,0,1.2,1,1,2,2,0.04}; //Q, R，W, Q_N   
    u_min_ = - u_max_;
    w_min_ = - w_max_;
    
    Q_ = DM::zeros(3,3);
    R_ = DM::zeros(3,3);
    W_ = DM::zeros(2,2);
    Q_N = DM::zeros(3,3);
    
    // obs
    obst_states[0] = 1;
    obst_states[1] = 0.7;
    obst_states[2] = 10;
    obst_states[3] = 10;

    safe_dist = 0.333 ;
    mpc_success = true;

    setWeights(weights);
    kinematic_equation_ = setKinematicEquation();
}
Mpc::~Mpc() {}

vector<double> Mpc::getObst(vector<double> obs_state, vector<double> dyn_obs_states) {
    for (int i = 0; i < N_+1; ++i){
        obst_states[2+2*i] = obs_state[2*i] ; // x
        obst_states[3+2*i] = obs_state[2*i+1] ; // y
    }
    dyn_obst_states = dyn_obs_states;
    return obst_states;
}

// 
vector<double> Mpc::getObst_collision() {
    return obst_collision;
}

void Mpc::setWeights(vector<double> weights) {
    Q_(0, 0) = weights[0];
    Q_(1, 1) = weights[1];
    Q_(2, 2) = weights[2];
    R_(0, 0) = weights[3];
    R_(1, 1) = weights[4];
    R_(2, 2) = weights[5];
    W_(0, 0) = weights[6];
    W_(1, 1) = weights[7];
    Q_N(0, 0) = weights[8];
    Q_N(1, 1) = weights[9];
    Q_N(2, 2) = weights[10];
}

Function Mpc::setKinematicEquation() {
    MX x = MX::sym("x");
    MX y = MX::sym("y");
    MX theta = MX::sym("theta");
    MX state_vars = MX::vertcat({x, y, theta});

    MX u = MX::sym("u");
    MX v = MX::sym("v");
    MX w = MX::sym("w");
    MX control_vars = MX::vertcat({u, v, w});

    MX rhs = u * MX::cos(theta);
    rhs = MX::vertcat({rhs, u * MX::sin(theta), w});
    return Function("kinematic_equation", {state_vars, control_vars}, {rhs});
}


bool Mpc::solve(Eigen::VectorXd current_states, vector<double> desired_states) {
    Opti opti = Opti();

    Slice all;

    MX cost = 0;
    X = opti.variable(3, N_ + 1);
    U = opti.variable(3, N_);
    MX x = X(0, all);
    MX y = X(1, all);
    MX theta = X(2, all);
    MX u = U(0, all);
    MX v = U(1, all);
    MX w = U(2, all);

    MX X_ref = opti.parameter(3, 1);
    MX X_cur = opti.parameter(3);
    MX U_cur = opti.parameter(3);
    DM x_tmp1 = {current_states[0], current_states[1], current_states[2]};
    DM u_tmp1 = {current_states[3], current_states[4], current_states[5]};

    opti.set_value(X_cur, x_tmp1);
    opti.set_value(U_cur, u_tmp1);

    DM X_ref_d(desired_states);
    X_ref = MX::reshape(X_ref_d, 3, 1);

    MX X_pre = opti.parameter(2, N_);
    for (int i = 0; i < N_; ++i){
        DM x_pre1 = {predict_states[2*i+2], predict_states[(2*i+1)+2]};
        opti.set_value(X_pre(all,i), x_pre1);
    }

    //set cost function
    //stage cost
    for (int i = 0; i < N_; ++i) {
        MX X_err = X(all, i) - X_ref(all, 0); 
        MX U_0 = U(all, i);
        MX x_err_pre = X(0, i) - X_pre(0, i);
        MX y_err_pre = X(1, i) - X_pre(1, i); 

        cost += MX::mtimes({X_err.T(), Q_, X_err});
        cost += MX::mtimes({U_0.T(), R_, U_0});
        cost += MX::mtimes({x_err_pre.T(), W_(0, 0), x_err_pre});
        cost += MX::mtimes({y_err_pre.T(), W_(1, 1), y_err_pre});
    }
    //terminal cost
    cost += MX::mtimes({(X(all, N_) - X_ref(all, 0)).T(), Q_N,
                        X(all, N_) - X_ref(all, 0)});
    opti.minimize(cost);

    //kinematic constrains
    for (int i = 0; i < N_; ++i) {
        vector<MX> input(2);
        input[0] = X(all, i);
        input[1] = U(all, i);
        MX X_next = kinematic_equation_(input)[0] * dt_ + X(all, i);
        opti.subject_to(X_next == X(all, i + 1));
    }

    //init value
    opti.subject_to(X(all, 0) == X_cur);

    //speed angle_speed limit
    opti.subject_to(0 <= u <= u_max_);
    opti.subject_to(0 <= v <= 0);
    opti.subject_to(w_min_ <= w <= w_max_);

    // CBF constrain(dynamic obs)
    for (int j = 0; j < (dyn_obst_states.size()/N_/2); ++j){
        for (int i = 0; i < N_-1; ++i) {
            if (dyn_obst_states[j*N_*2+2*i] != 0.171){
                opti.subject_to(h_dyn(X(all, i+1), dyn_obst_states[j*N_*2+2*(i+1)], dyn_obst_states[j*N_*2+2*(i+1)+1]) >= (1 - obst_states[1]) * h_dyn(X(all, i), dyn_obst_states[j*N_*2+2*i], dyn_obst_states[j*N_*2+2*i+1]) );
            }
        }
    }
    dyn_obst_states.clear();
    // CBF constrain(static obs) 
    for (int i = 0; i < N_; ++i) {
        if (i != 1 && (obst_states[2+2*i]!=0.0 && obst_states[3+2*i]!=0.0)){
            obst_collision.push_back(obst_states[2+2*i]);
            obst_collision.push_back(obst_states[3+2*i]);
        }else if(obst_states[2+2*1] > current_states[0] + 0.3 && obst_states[2+2*1]!=100){
            opti.subject_to(h(X(all, i+1), obst_states[2+2*1], obst_states[3+2*1]) >= (1 - obst_states[1]) * h(X(all, i), obst_states[2+2*1], obst_states[3+2*1]) );
        }

        if (obst_collision.size()){
            for(int j = 0; j < obst_collision.size()/2; ++j){
                if(obst_collision[2*j] != 100 && obst_collision[2*j] < current_states[0] + 0.3 ){
                    obst_collision[2*j] = 100;
                    obst_collision[2*j+1] = 100;
                }
                if (obst_collision[2*j] != 100){
                    opti.subject_to(h(X(all, i+1), obst_collision[2*j], obst_collision[2*j+1]) >= (1 - obst_states[1]) * h(X(all, i), obst_collision[2*j], obst_collision[2*j+1]) );
                }
            }
        }
    }  
    // Clean up irrelevant historical risk points
    for (int i=0; i<obst_collision.size(); ){
        if(obst_collision[i]==100){
            obst_collision.erase(obst_collision.begin() + i);
        }else{
            i++;
        }
    }

    //set solver
    casadi::Dict solver_opts;
    solver_opts["expand"] = true;
    solver_opts["ipopt.max_iter"] = 2000;
    solver_opts["ipopt.print_level"] = 0;
    solver_opts["print_time"] = 0;
    solver_opts["ipopt.acceptable_tol"] = 1e-3;
    solver_opts["ipopt.acceptable_obj_change_tol"] = 1e-3;

    try{
        opti.solver("ipopt", solver_opts);
        solution_ = std::make_unique<casadi::OptiSol>(opti.solve());
        mpc_success = true;
    }
    catch(const std::exception& e) {
        ROS_ERROR("Infeasible Solution: %s", e.what());
        mpc_success = false;
    }

    return mpc_success;
}

vector<double> Mpc::getFirstU() {
    vector<double> res;
    auto first_u =  solution_->value(U)(0, 0);
    auto first_v =  solution_->value(U)(1, 0);
    auto first_w = solution_->value(U)(2, 0);

    res.push_back(static_cast<double>(first_u));
    res.push_back(static_cast<double>(first_v));
    res.push_back(static_cast<double>(first_w));
    return res;
}

vector<double> Mpc::getAllU() {
    vector<double> res;
    for (int i = 0; i < N_; ++i) {
        auto u =  solution_->value(U)(0, i);
        auto v =  solution_->value(U)(1, i);
        auto w = solution_->value(U)(2, i);

        res.push_back(static_cast<double>(u));
        res.push_back(static_cast<double>(v));
        res.push_back(static_cast<double>(w));
    }
    return res;
}

vector<double> Mpc::getPredictX() {
    predict_states.clear();
    auto predict_x = solution_->value(X);

    for (int i = 0; i <= N_; ++i) {
        predict_states.push_back(static_cast<double>(predict_x(0, i)));
        predict_states.push_back(static_cast<double>(predict_x(1, i)));
    }
    return predict_states;
}

bool Mpc::is_target(Eigen::VectorXd cur, vector<double> goal)
{
	if(abs(cur[0] - goal[0]) < 0.05 && abs(cur[1] - goal[1]) < 0.05)
	{
		return true;
	}
	else return false;
}

MX Mpc::h(MX states, const double &obs_x, const double &obs_y)
{
	MX result;
    result = sqrt((states(0, 0)-obs_x)*(states(0, 0)-obs_x) + (states(1, 0)-obs_y)*(states(1, 0)-obs_y)) - safe_dist ;
	return result;
}

MX Mpc::h_dyn(MX states, const double &obs_x, const double &obs_y)
{
	MX result;
    result = sqrt((states(0, 0)-obs_x)*(states(0, 0)-obs_x) + (states(1, 0)-obs_y)*(states(1, 0)-obs_y)) - safe_dist - 0.2 ;
	return result;
}
