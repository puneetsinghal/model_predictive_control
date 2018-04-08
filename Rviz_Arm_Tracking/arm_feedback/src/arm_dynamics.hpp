/**
    File: arm_dynamics.hpp
    Description: Calculate the dynamics equations of a 5-DOF HEBI manipulator
    arm used on the Biorobotics Lab Intelligent Mobile Robot (IMR) project
    Source File: arm_dynamics.cpp

    @author Ky Woodard
    @version 1.0 2/12/2017

    Functions:
		gravityComp
			Description: Calculate the necessary torque required to cancel
						 gravitational forces on the joints
			Input:
				theta  - current angles of the five joints [rad]
			Output:
				torque - computed torque on the 5 joints [N-m]

		inverseDynamics
			Description: Calculate the inverse dynamics of the arm, which
					 	 includes inertia, corriolis, gravitational, and
					 	 friction effect
			Inputs:
				theta  - current angle of the 5 joints [rad]
				omega  - current angular velocity of the 5 joints [rad/s]
				alpha  - desired angular acceleration of the 5 joints [rad/s^2]
			Output:
				torque - computed torque to produce desired acceleration
						 at the 5 joints [N-m]

		gravity
			Description: Calculate the gravity torque of the inverse dynamics
						 based on the current angle of the joints
			Inputs:
				theta  - current angle of the 5 joints
			Output:
				torque - computed torque on the 5 joints [N-m]

		inertia
			Description: Calculate the inertia torque of the inverse dynamics
						 based on the current angle of the joints
			Inputs:
				theta  - current angle of the 5 joints [rad]
			Output:
				Inertia - inertia matrix (5x5)

		corriolis
			Description: Calculate the corriolis torque of the inverse dynamics
						 based on the current angle and angular velocity of the
						 joints
			Inputs:
				theta  - current angle of the 5 joints [rad]
				omega  	- current angle of the 5 joints [rad/s]
			Output:
				Corriolis - Corriolis matrix (5x5)
*/

#ifndef ARM_DYNAMICS_H
#define ARM_DYNAMICS_H

#include <eigen3/Eigen/Eigen>

void gravityComp(const std::vector<double> &, std::vector<double> &);
void inverseDynamics(const Eigen::VectorXd &,const Eigen::VectorXd &,
					  const Eigen::VectorXd &,std::vector<double> &);
void inverseDynamics(const Eigen::VectorXd &theta,const Eigen::VectorXd &omega,
					const Eigen::VectorXd &alpha, Eigen::VectorXd &torque);


void forwardDynamics(const Eigen::VectorXd &theta, const Eigen::VectorXd &omega, 
						const Eigen::VectorXd &torque, std::vector<double> &alpha);
void forwardDynamics(const Eigen::VectorXd &theta, const Eigen::VectorXd &omega, 
						const Eigen::VectorXd &torque, Eigen::VectorXd &alpha);

void gravity(const Eigen::VectorXd &,Eigen::VectorXd &);
void inertia(const Eigen::VectorXd &,Eigen::MatrixXd &);
void corriolis(const Eigen::VectorXd &,const Eigen::VectorXd &,Eigen::MatrixXd &);



#endif