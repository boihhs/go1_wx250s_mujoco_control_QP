#include "MPC_QP.h"
#include "inertial_params.h"
#include <iostream>
#include <limits> // Required for std::numeric_limits
#include "utils.h"

// Global Variables (as per your MATLAB code)
double dt_MPC = 0.04;
double m_Object = 0.032492;

Eigen::VectorXd MPC_QP(const Eigen::VectorXd& xin, const Eigen::VectorXd& PosVel,
                     const Eigen::VectorXd& qArm, const Eigen::Vector3d& F_Tatile, double t) {
    // Initialization
    const int nx = 19; //r3 x3 w3 v3 1 po3 vo3
    const int nu = 15; //GRF12 Fo3
    const int p = 10;  // number of horizons
    
    // Reference Generation
    //Eigen::VectorXd xref = Traj_Command_01M(t, xin)
    Eigen::VectorXd xref = Traj_Command_01M(t);

    // Call the function to compute COM and MOI
    //std::pair<Eigen::Vector3d, Eigen::Matrix3d> result = inertial_params(qArm);

    // Extract the results
    //Eigen::Vector3d COM_b = result.first;
    //Eigen::Matrix3d I_b = result.second;
    
    // Continuous Time Dynamics
    double g = 9.81;
    double m = 12.03 + 0.538736 + 0.480879 + 0.430811 + 0.234589 + 0.220991 + 0.084957 + 0.110084;
    Eigen::Matrix3d R_r = eul2rotm(xin.segment(0,3).reverse());
    
    Eigen::Vector3d p_COM = xin.segment(3, 3) + R_r * COM_b;
    
    Eigen::Matrix3d I_r = R_r * I_b * R_r.transpose();
   
    Eigen::MatrixXd Ac = Eigen::MatrixXd::Zero(nx, nx);
    Ac.block<3, 3>(0, 6) = R_r.transpose(); // R_z
    Ac.block<3, 3>(3, 9) = Eigen::Matrix3d::Identity();
    Ac(11, 12) = -g;
    Ac.block<3, 3>(13, 13) = Eigen::Matrix3d::Identity();
    Ac(18, 12) = -g;

    Eigen::Matrix3d omega_coeff;
    omega_coeff <<  0                    ,R_r.row(2)*COM_b  ,-R_r.row(1)*COM_b,
                  -R_r.row(2)*COM_b    ,0                    ,R_r.row(0)*COM_b,
                   R_r.row(1)*COM_b    ,-R_r.row(0)*COM_b  ,0;

    Eigen::MatrixXd Lambda = Eigen::MatrixXd::Identity(6, 6);
    Lambda.block<3,3>(3,0) = omega_coeff;
    Eigen::MatrixXd B_Lambda(6,15);
    B_Lambda.setZero();
    B_Lambda.block<3,3>(0,0) = I_r.inverse() * skew(-p_COM + PosVel.segment(0,3));
    B_Lambda.block<3,3>(0,3) = I_r.inverse() * skew(-p_COM + PosVel.segment(6,3));
    B_Lambda.block<3,3>(0,6) = I_r.inverse() * skew(-p_COM + PosVel.segment(12,3));
    B_Lambda.block<3,3>(0,9) = I_r.inverse() * skew(-p_COM + PosVel.segment(18,3));
    B_Lambda.block<3,3>(0,12) = I_r.inverse() * skew(-p_COM + PosVel.segment(24,3));
    B_Lambda.block<3,3>(3,0) = Eigen::Matrix3d::Identity() / m;
    B_Lambda.block<3,3>(3,3) = Eigen::Matrix3d::Identity() / m;
    B_Lambda.block<3,3>(3,6) = Eigen::Matrix3d::Identity() / m;
    B_Lambda.block<3,3>(3,9) = Eigen::Matrix3d::Identity() / m;
    B_Lambda.block<3,3>(3,12) = Eigen::Matrix3d::Identity() / m;

    Eigen::MatrixXd Bc = Eigen::MatrixXd::Zero(nx, nu);
    Bc.block(6, 0, 6, 15) = Lambda.inverse() * B_Lambda;
    Bc.block(16, 12, 3, 3) = -Eigen::Matrix3d::Identity() / m_Object;
    
    //std::cout << PosVel;
    
    // Discrete Time Dynamics
    Eigen::MatrixXd A_d = Eigen::MatrixXd::Identity(nx, nx) + Ac * dt_MPC;

    
    Eigen::MatrixXd A_qp = Eigen::MatrixXd::Zero(nx * p, nx);
    for (int ind = 0; ind < p; ++ind) {
        Eigen::MatrixXd A_d_pow = Eigen::MatrixXd::Identity(nx, nx); // Initialize as identity matrix
        for (int k = 0; k <= ind; ++k) {
            A_d_pow *= A_d; // Compute A_d^(ind+1)
        }
        A_qp.block(ind * nx, 0, nx, nx) = A_d_pow; // Assign the computed block
    }
    


    Eigen::MatrixXd B_d = Bc * dt_MPC;
    
    Eigen::MatrixXd B_qp = Eigen::MatrixXd::Zero(p * nx, p * nu);
    for (int ind = 0; ind < p; ++ind) {
        for (int j = 0; j <= ind; ++j) {
            Eigen::MatrixXd A_d_pow_partial = Eigen::MatrixXd::Identity(nx, nx); // Initialize as identity matrix
            for (int k = 0; k < (ind - j); ++k) {
                A_d_pow_partial *= A_d; // Compute A_d^(ind-j)
            }
            B_qp.block(ind * nx, j * nu, nx, nu) = A_d_pow_partial * B_d; // Assign the computed block
        }
    }

    
    // Gait
    int i = std::ceil(t / dt_MPC);
    if (t < dt_MPC) {
        i = 1;
    }
     int segment = std::to_string(i).back() - '0';

    Eigen::MatrixXd A_eq = Eigen::MatrixXd::Zero(2 * p, nu * p);
    Eigen::VectorXd b_eq = Eigen::VectorXd::Zero(2 * p);
    
    if (t <= time_stop){
        for(int ind = 0; ind < p; ++ind) {
            int phase = segment + ind ;
                if(phase >= 10){
                phase = phase - 10;
            }
                if (phase < 6 && phase != 0) {
                A_eq.block(ind * 2, ind * nu, 2, nu) <<  0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                                        0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0;
                }
                else{
                 A_eq.block(ind * 2, ind * nu, 2, nu) <<  0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                                        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0;
               }
        }
    }
    else {
         A_eq = Eigen::MatrixXd::Zero(0,0);
         b_eq = Eigen::VectorXd::Zero(0);
    }

    // Inequality Constraints
    double GRFz_max = 500;
    double GRFz_min = 0;
    double mu_pyramid = 0.5;
    
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(24 * p, nu * p);

    Eigen::VectorXd b(24*p);
    b.setZero();
    for(int ind=0; ind <p; ++ind){
       b.segment(ind*24,24) << 0, 0, 0, 0, GRFz_max, GRFz_min,
                            0, 0, 0, 0, GRFz_max, GRFz_min,
                            0, 0, 0, 0, GRFz_max, GRFz_min,
                            0, 0, 0, 0, GRFz_max, GRFz_min;
    }

    Eigen::MatrixXd A_mid(24, nu);
    A_mid.setZero();
    A_mid << 1, 0, -mu_pyramid, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            -1, 0, -mu_pyramid, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 1, -mu_pyramid, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, -1, -mu_pyramid, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
           0, 0, 0, 1, 0, -mu_pyramid, 0, 0, 0, 0, 0, 0, 0, 0, 0,
           0, 0, 0, -1, 0, -mu_pyramid, 0, 0, 0, 0, 0, 0, 0, 0, 0,
           0, 0, 0, 0, 1, -mu_pyramid, 0, 0, 0, 0, 0, 0, 0, 0, 0,
           0, 0, 0, 0, -1, -mu_pyramid, 0, 0, 0, 0, 0, 0, 0, 0, 0,
           0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
           0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
           0, 0, 0, 0, 0, 0, 1, 0, -mu_pyramid, 0, 0, 0, 0, 0, 0,
           0, 0, 0, 0, 0, 0, -1, 0, -mu_pyramid, 0, 0, 0, 0, 0, 0,
           0, 0, 0, 0, 0, 0, 0, 1, -mu_pyramid, 0, 0, 0, 0, 0, 0,
           0, 0, 0, 0, 0, 0, 0, -1, -mu_pyramid, 0, 0, 0, 0, 0, 0,
           0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
           0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0,
           0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, -mu_pyramid, 0, 0, 0,
           0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, -mu_pyramid, 0, 0, 0,
           0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -mu_pyramid, 0, 0, 0,
           0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, -mu_pyramid, 0, 0, 0,
           0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
           0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0;
           
    for (int ind = 0; ind < p; ++ind) {
        A.block(ind * 24, ind * nu, 24, nu) = A_mid;
    }

   


    // Cost Function
    Eigen::VectorXd Q_diag(nx);
    Q_diag << 2000, 2000, 2000, 500, 1000, 2000, 120, 50, 50, 10, 10, 50, 0, 150, 150, 200, 10, 10, 50;
    Eigen::MatrixXd Q = Q_diag.asDiagonal();

    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(nu, nu);
    double alpha = 1e-3;

    // QP setup
    Eigen::MatrixXd L = Eigen::MatrixXd::Zero(nx * p, nx * p);
    for (int ind = 0; ind < p; ++ind) {
        L.block(ind * nx, ind * nx, nx, nx) = Q;
    }

    Eigen::MatrixXd K = Eigen::MatrixXd::Zero(nu * p, nu * p);
    for (int ind = 0; ind < p; ++ind) {
        K.block(ind * nu, ind * nu, nu, nu) = R * alpha;
    }
    

    Eigen::VectorXd x0 = xin;
    Eigen::MatrixXd H = 2 * (B_qp.transpose() * L * B_qp + K);
    
    Eigen::VectorXd f = 2 * B_qp.transpose() * L * (A_qp * x0 - xref);

    Eigen::VectorXd F_c(15);

    if (A_eq.sum() != 0){
        int total_constraints = A_eq.rows() + A.rows();

        // Combine A_eq and A into a dense matrix
        Eigen::MatrixXd combined_A(total_constraints, nu * p);
        combined_A.topRows(A_eq.rows()) = A_eq;           // First A_eq rows
        combined_A.bottomRows(A.rows()) = A;             // Following A rows

        // Combine lower and upper bounds
        Eigen::VectorXd combined_lower_bound(total_constraints);
        Eigen::VectorXd combined_upper_bound(total_constraints);

        combined_lower_bound.head(b_eq.rows()) = b_eq;   // Lower bound for equality constraints
        combined_upper_bound.head(b_eq.rows()) = b_eq;   // Upper bound for equality constraints

        combined_lower_bound.tail(b.rows()) = Eigen::VectorXd::Constant(b.rows(), -std::numeric_limits<double>::infinity());
        combined_upper_bound.tail(b.rows()) = b;         // Upper bound for inequality constraints

        // Set up the OSQP solver
        OsqpEigen::Solver solver;
        solver.clearSolver();
        
        solver.settings()->setVerbosity(false);
        solver.settings()->setWarmStart(false);
        solver.data()->setNumberOfVariables(nu * p);
        solver.data()->setNumberOfConstraints(total_constraints);

        // Convert dense matrices to sparse matrices
        Eigen::SparseMatrix<double> H_sparse = H.sparseView();
        Eigen::SparseMatrix<double> A_sparse = combined_A.sparseView();

        // Set data for the solver
        solver.data()->setHessianMatrix(H_sparse);
        solver.data()->setGradient(f);
        solver.data()->setLinearConstraintsMatrix(A_sparse);
        solver.data()->setLowerBound(combined_lower_bound);
        solver.data()->setUpperBound(combined_upper_bound);
        // Initialize and solve the problem
        solver.initSolver();
        solver.solveProblem();

        // Extract the solution
        Eigen::VectorXd U_opt = solver.getSolution();
        F_c.head(12) = U_opt.head(12);
        F_c.tail(3) = U_opt.segment(12, 3);
    }else{
        int total_constraints = A.rows();

        // Combine lower and upper bounds
        Eigen::VectorXd combined_lower_bound(total_constraints);
        Eigen::VectorXd combined_upper_bound(total_constraints);

        combined_lower_bound = Eigen::VectorXd::Constant(b.rows(), -std::numeric_limits<double>::infinity());
        combined_upper_bound = b;         // Upper bound for inequality constraints

        // Set up the OSQP solver
        OsqpEigen::Solver solver;
        solver.clearSolver();
        
        solver.settings()->setVerbosity(false);
        solver.settings()->setWarmStart(false);
        solver.data()->setNumberOfVariables(nu * p);
        solver.data()->setNumberOfConstraints(total_constraints);

        // Convert dense matrices to sparse matrices
        Eigen::SparseMatrix<double> H_sparse = H.sparseView();
        Eigen::SparseMatrix<double> A_sparse = A.sparseView();

        // Set data for the solver
        solver.data()->setHessianMatrix(H_sparse);
        solver.data()->setGradient(f);
        solver.data()->setLinearConstraintsMatrix(A_sparse);
        solver.data()->setLowerBound(combined_lower_bound);
        solver.data()->setUpperBound(combined_upper_bound);
        // Initialize and solve the problem
        solver.initSolver();
        solver.solveProblem();

        // Extract the solution
        Eigen::VectorXd U_opt = solver.getSolution();
        F_c.head(12) = U_opt.head(12);
        F_c.tail(3) = U_opt.segment(12, 3);
    }


    return F_c;

}