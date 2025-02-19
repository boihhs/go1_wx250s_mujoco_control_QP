#include "Traj_Command_Goal.h"

// Function to generate the trajectory
VectorXd Traj_Command_Goal(const double t, const VectorXd& xin, const Vector3d& goal, const Vector3d& prev) {
    const double dt_MPC = 0.04; // MPC time step
    const int nx = 19;          // Number of states
    const int p = 10;           // Number of horizons

    // Initialize trajectory matrix (nx x (p+1))
    MatrixXd traj = MatrixXd::Zero(nx, p + 1);

    // Default values
    Vector3d rpy_def(0, 0, 0); // Roll, Pitch, Yaw defaults
    double z_def = 0.3;        // Default body z-position

    // Set default values for roll, pitch, yaw, and body z
    traj.row(0) = VectorXd::Constant(p + 1, rpy_def(0)); // Roll
    traj.row(1) = VectorXd::Constant(p + 1, rpy_def(1)); // Pitch
    traj.row(2) = VectorXd::Constant(p + 1, rpy_def(2)); // Yaw
    traj.row(5) = VectorXd::Constant(p + 1, z_def);      // Body z
    traj.row(12) = VectorXd::Constant(p + 1, 1.0);       // Gravity (g)

    // Initial offsets for positions
    double xo0 = 0.4507, yo0 = 0.0, zo0 = 0.7187;

    // Trajectory parameters
    double vx = 0, vy = 0.0, w_yaw = 0.0;
    double vxo = 0, vyo = 0, vzo = 0;


    // Trajectory definition
    if (t > time_stop) {
        // If time exceeds stop time, set velocities and accelerations to zero
        traj.row(8).setZero();  // w_yaw
        traj.row(9).setZero(); // vx
        traj.row(10).setZero(); // vy
        traj.row(16).setZero(); // vzo
        traj.row(17).setZero(); // vxo
        traj.row(18).setZero(); // vyo

        // Set positions and orientations to final values
        traj.row(2) = VectorXd::Constant(p + 1, w_yaw * time_stop); // Final yaw
        traj.row(3) = VectorXd::Constant(p + 1, vx * time_stop);    // Final x
        traj.row(4) = VectorXd::Constant(p + 1, vy * time_stop);    // Final y
        traj.row(13) = VectorXd::Constant(p + 1, xo0 + vxo * time_stop);
        traj.row(14) = VectorXd::Constant(p + 1, yo0 + vyo * time_stop);
        traj.row(15) = VectorXd::Constant(p + 1, zo0 + vzo * time_stop);

        if (traj(14,0)-traj(4,0) > .5){
            traj.row(14) = traj.row(4).array() + .5;
        }
        if (traj(15,0)-traj(5,0) < -.15){
            traj.row(15) = traj.row(5).array() - .15;
        }
    } else {
        // Iterate through each horizon step
        for (int ind = 0; ind <= p; ++ind) {
            double p = 1-pow(1-((t / 10) - std::floor(t / 10)), 1);
    
          
            // Assign velocity values
            traj(8, ind) = w_yaw;  // Angular velocity
            traj(9, ind) = vx;    // Linear velocity in x
            traj(10, ind) = vy;    // Linear velocity in y
            traj(16, ind) = vxo;   // x velocity offset
            traj(17, ind) = vyo;   // y velocity offset
            traj(18, ind) = vzo;   // z velocity offset
            //std::cout << (1 - ((t / 5) - std::floor(t /5)))*xin(3) + (((t / 5) - std::floor(t /5)))*1 << std::endl;
            if (ind < 2) {
                // First step: Update using current time
                traj(2, ind) = w_yaw * t + dt_MPC * traj(8, ind);
                traj(3, ind) = (1 - p)*(prev(0) - .3) + ((p))*(goal(0) - .3);
                traj(4, ind) = (1 - (p))*(prev(1)) + ((p))*(goal(1));
                traj(13, ind) = (1 - (p))*prev(0)  + ((p))*(goal(0));
                traj(14, ind) = (1 - (p))*prev(1)  + ((p))*(goal(1));
                traj(15, ind) = (1 - (p))*prev(2)  + ((p))*(goal(2));
                /*
                // Uncomment this to use xin for initial state-based updates
                traj(2, ind) = xin(2) + dt_MPC * traj(8, ind);
                traj(3, ind) = xin(3) + dt_MPC * traj(9, ind);
                traj(4, ind) = xin(4) + dt_MPC * traj(10, ind);
                traj(13, ind) = xin(13) + dt_MPC * traj(16, ind);
                traj(14, ind) = xin(14) + dt_MPC * traj(17, ind);
                traj(15, ind) = xin(15) + dt_MPC * traj(18, ind);
                */
            } else {
                /*
                traj(2, ind) = traj(3, ind - 1) + dt_MPC * traj(8, ind - 1);
                traj(3, ind) = traj(4, ind - 1) + dt_MPC * traj(9, ind - 1);
                traj(4, ind) = traj(5, ind - 1) + dt_MPC * traj(10, ind - 1);
                traj(13, ind) = traj(13, ind - 1) + dt_MPC * traj(16, ind - 1);
                traj(14, ind) = traj(14, ind - 1) + dt_MPC * traj(17, ind - 1);
                traj(15, ind) = traj(15, ind - 1) + dt_MPC * traj(18, ind - 1);
                */
                traj(2, ind) = w_yaw * t + dt_MPC * traj(8, ind - 1);
                traj(3, ind) = (1 - p)*(prev(0) - .3) + ((p))*(goal(0) - .3);
                traj(4, ind) = (1 - (p))*(prev(1)) + ((p))*(goal(1));
                traj(13, ind) = (1 - (p))*prev(0)  + ((p))*(goal(0));
                traj(14, ind) = (1 - (p))*prev(1)  + ((p))*(goal(1));
                traj(15, ind) = (1 - (p))*prev(2)  + ((p))*(goal(2));
            }

            // Apply constraints
            if (traj(14, ind) - traj(4, ind) > 0.5) {
                traj(14, ind) = traj(4, ind) + 0.5;
            }
            if (traj(15, ind) - traj(5, ind) < -0.15) {
                traj(15, ind) = traj(5, ind) - 0.15;
            }
        }
    }

    // Remove the first column and reshape
    MatrixXd traj_final = traj.block(0, 1, nx, p); // Exclude the first column
    
    VectorXd traj_reshaped = Map<VectorXd>(traj_final.data(), nx * p);
    return traj_reshaped;
}


VectorXd Traj_Command_Goal2(const double t, const VectorXd& xin, const Vector3d& goal, const Vector3d& prev) {
    const double dt_MPC = 0.04; // MPC time step
    const int nx = 19;          // Number of states
    const int p = 10;           // Number of horizons

    // Initialize trajectory matrix (nx x (p+1))
    MatrixXd traj = MatrixXd::Zero(nx, p + 1);

    // Default values
    Vector3d rpy_def(0, 0, 0); // Roll, Pitch, Yaw defaults
    double z_def = 0.3;        // Default body z-position

    // Set default values for roll, pitch, yaw, and body z
    traj.row(0) = VectorXd::Constant(p + 1, rpy_def(0)); // Roll
    traj.row(1) = VectorXd::Constant(p + 1, rpy_def(1)); // Pitch
    traj.row(2) = VectorXd::Constant(p + 1, rpy_def(2)); // Yaw
    traj.row(5) = VectorXd::Constant(p + 1, z_def);      // Body z
    traj.row(12) = VectorXd::Constant(p + 1, 1.0);       // Gravity (g)

    // Initial offsets for positions
    double xo0 = 0.4507, yo0 = 0.0, zo0 = 0.7187;

    // Trajectory parameters
    double vx = 1, vy = 0, w_yaw = 0.0;
    double vxo = 0, vyo = 0, vzo = 0;


    // Trajectory definition
    if (t > time_stop) {
        // If time exceeds stop time, set velocities and accelerations to zero
        traj.row(8).setZero();  // w_yaw
        traj.row(9).setZero(); // vx
        traj.row(10).setZero(); // vy
        traj.row(16).setZero(); // vzo
        traj.row(17).setZero(); // vxo
        traj.row(18).setZero(); // vyo

        // Set positions and orientations to final values
        traj.row(2) = VectorXd::Constant(p + 1, w_yaw * time_stop); // Final yaw
        traj.row(3) = VectorXd::Constant(p + 1, vx * time_stop);    // Final x
        traj.row(4) = VectorXd::Constant(p + 1, vy * time_stop);    // Final y
        traj.row(13) = VectorXd::Constant(p + 1, xo0 + vxo * time_stop);
        traj.row(14) = VectorXd::Constant(p + 1, yo0 + vyo * time_stop);
        traj.row(15) = VectorXd::Constant(p + 1, zo0 + vzo * time_stop);

        if (traj(14,0)-traj(4,0) > .5){
            traj.row(14) = traj.row(4).array() + .5;
        }
        if (traj(15,0)-traj(5,0) < -.15){
            traj.row(15) = traj.row(5).array() - .15;
        }
    } else {
        // Iterate through each horizon step
        for (int ind = 0; ind <= p; ++ind) {
            double p = 1 - pow(2*((t / 30) - std::floor(t / 30)) - 1, 2);
            
          
            // Assign velocity values
            traj(8, ind) = w_yaw;  // Angular velocity
            traj(9, ind) = vx;    // Linear velocity in x
            traj(10, ind) = vy;    // Linear velocity in y
            traj(16, ind) = vxo;   // x velocity offset
            traj(17, ind) = vyo;   // y velocity offset
            traj(18, ind) = vzo;   // z velocity offset
            //std::cout << (1 - ((t / 5) - std::floor(t /5)))*xin(3) + (((t / 5) - std::floor(t /5)))*1 << std::endl;
            if (ind < 2) {
                // First step: Update using current time
                traj(2, ind) = w_yaw * t + dt_MPC * traj(8, ind);
                traj(3, ind) = t*(dt_MPC*vx);
                traj(4, ind) = 0;
                traj(13, ind) = (1 - (p))*(traj(3, ind)+xo0)  + ((p))*(xo0);
                traj(14, ind) = (1 - (p))*traj(4, ind) + ((p))*(.3);
                traj(15, ind) = (1 - (p))*(traj(5, ind)+0.7187 - .3)  + ((p))*(traj(5, ind)-.2);
                /*
                // Uncomment this to use xin for initial state-based updates
                traj(2, ind) = xin(2) + dt_MPC * traj(8, ind);
                traj(3, ind) = xin(3) + dt_MPC * traj(9, ind);
                traj(4, ind) = xin(4) + dt_MPC * traj(10, ind);
                traj(13, ind) = xin(13) + dt_MPC * traj(16, ind);
                traj(14, ind) = xin(14) + dt_MPC * traj(17, ind);
                traj(15, ind) = xin(15) + dt_MPC * traj(18, ind);
                */
            } else {
                /*
                traj(2, ind) = traj(3, ind - 1) + dt_MPC * traj(8, ind - 1);
                traj(3, ind) = traj(4, ind - 1) + dt_MPC * traj(9, ind - 1);
                traj(4, ind) = traj(5, ind - 1) + dt_MPC * traj(10, ind - 1);
                traj(13, ind) = traj(13, ind - 1) + dt_MPC * traj(16, ind - 1);
                traj(14, ind) = traj(14, ind - 1) + dt_MPC * traj(17, ind - 1);
                traj(15, ind) = traj(15, ind - 1) + dt_MPC * traj(18, ind - 1);
                */
                traj(2, ind) = w_yaw * t + dt_MPC * traj(8, ind - 1);
                traj(3, ind) = t*(dt_MPC*vx);
                traj(4, ind) = 0;
                traj(13, ind) = (1 - (p))*(traj(3, ind)+xo0)  + ((p))*(xo0);
                traj(14, ind) = (1 - (p))*traj(4, ind) + ((p))*(.3);
                traj(15, ind) = (1 - (p))*(traj(5, ind)+0.7187 - .3)  + ((p))*(traj(5, ind)-.2);
            }

            // Apply constraints
            if (traj(14, ind) - traj(4, ind) > 0.5) {
                traj(14, ind) = traj(4, ind) + 0.5;
            }
            if (traj(15, ind) - traj(5, ind) < -0.15) {
                traj(15, ind) = traj(5, ind) - 0.15;
            }
        }
    }

    // Remove the first column and reshape
    MatrixXd traj_final = traj.block(0, 1, nx, p); // Exclude the first column
    
    VectorXd traj_reshaped = Map<VectorXd>(traj_final.data(), nx * p);
    return traj_reshaped;
}
