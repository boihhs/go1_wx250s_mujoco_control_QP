#ifndef CARTESIAN_PD_END_H
#define CARTESIAN_PD_END_H

#include <Eigen/Dense>
#include <vector>
#include "Traj_Command_01M.h"


Eigen::VectorXd Cartesian_PD_End(const Eigen::VectorXd& x, double t);


#endif 
