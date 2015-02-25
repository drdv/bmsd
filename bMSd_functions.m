%
% ------------------------------------------------------
% | Basic Multibody Simulator Derived (Matlab toolbox) |
% ------------------------------------------------------
%
% List of functions & scripts:
%

% Main directory (./)
% --------------------
% setup_bMSd(case_flag)         - Add (remove) bMSd Toolbox to (from) Matlab path
% struct_def                    - Definition of system parameters
%

% General purpose (./general_purpose)
% ------------------------------------
% SV = System_Variables(SP)     - Manipulator system variables initialization
% joints = joint_tree(SP,bN)    - Returns joints between link 1 and link "bN"
% Draw_System(SP, SV, ...)      - Plots the manipulator system
% DrawCylinder(pos, az, ...)    - Draw a cylinder (used to display the manipulator joints)
% n = cross(u,v)                - Cross product of two vectors
% S = tilde(w)                  - Returns a skew-symmetric matrix from a 3D vector      
% draw_frame(R,p)               - Draws a coordinate frame with orientation R at position p
% write2file                    - Writes data to file 
% [q,dq,ddq] = poly3(qi,qf,...) - Using 3rd order polynomial generate position, velocity ...
% P1 = trajC(x,y,z,t_f,...)     - Generation of Cartesian trajectory           
%

% Kinematics (./kinematics)
% -----------------------
% SV = calc_pos(SP,SV)          - Updates SV.L(2:SP.n+1).R and SV.L(2:SP.n+1).p
% SV = calc_pos_from(...)       - like calc_pos.m but starts the recursion from a given base_link 
% SV = calc_vel(SP,SV)          - Updates SV.L(2:SP.n+1).w and SV.L(2:SP.n+1).v
% SV = calc_acc(SP,SV)          - Updates SV.L(2:SP.n+1).dw and SV.L(2:SP.n+1).dv
% [p,R]=fk_e(SP,SV,bN,bP)       - Calculates the position and orientation of an end-effector point
% p = fk_j(SP,SV,joints)        - Calculates the position of n manipulator joints
% Je = calc_Je(SP,SV,bN,bP)     - Calculates the Jacobian matrix of an end-effector point ... 
% Je = calc_Je_from(...)        - like calc_Je.m but uses user given base_link 
% dJe = calc_dJe(SP,SV,bN,bP)   - Calculates the derivative of Je
% Jmm = calc_Jmm(SP,SV)         - Calculates an augmented Jacobian matrix Jmm 
% dJmm = calc_dJmm(SP,SV)       - Calculates the derivative of Jmm
% [q, iter] = ik_e(SP,SV,...)   - Inverse kinematics
%

% Dynamics (./dynamics)
% ----------------------
% HH = calc_hh(SP,SV)           - Calculation of the inertia matrix HH
% [HH,R,M] = calc_hh_a(SP,SV)   - Calculation of the inertia matrix HH (alternative 1)
% T = f2t(SV,bN,bP,bF)  TO TEST - Computes torque resulting from a force bF applied at a point bP
% c = r_ne_a(SP,SV,R,Gravity)   - recursive Newton-Euler method (inverse dynamics)
% c = r_ne(SP,SV,Gravity)       - recursive Newton-Euler method (inverse dynamics)
% c = r_ne1_a(SP,SV,R,...)      - recursive Newton-Euler method (inverse dynamics) with desired accelerations
% c = r_ne2_a(SP,SV,R,Gravity)  - recursive Newton-Euler method - no forces and gravity
% SV = f_dyn(SP,SV,Gravity)     - forward dynamics
% [SV,dy] = f_dyn_Q(SP,SV,...)  - forward dynamics (updating the Quaternion of the base)
% G = calc_G(SP,R,Gravity)      - Computes the generalized forces/torques as a result of Gravity
% Rg = calc_CoM(SP,SV)          - Computes the Center of Mass of the system
%

% Rotations (./rotation)
% ----------------------
% q = aa2q(an,ax)               - Converts axis angle representation to unit quaternion
% R = aa2R(an,ax)               - Converts axis angle representation to rotation matrix 
% [an,ax] = R2aa(R)             - Converts a rotation matrix to axis-angle representation
% R = q2R(s,x,y,z)              - Converts a unit quaternion to rotation matrix 
% rpy = q2rpy(q)                - Convert a unit quaternion to X-Y-Z rotation sequence (around the new axis) 
% LM = qLM(q)                   - Returns an orthogonal matrix LM for quaternion multiplication
% RM = qRM(q)                   - Returns an orthogonal matrix RM for quaternion multiplication
% qi = quat_inv(q)              - Computes an inverse of a quaternion q
% qm = quat_mult(q1,q2)         - Multiplication of two quaternions
% qm = quat_mult(q1,q2,q3)      - Multiplication of three quaternions
% dq = qw2dq(q,w)               - Time derivative of a quaternion when we know the angular velocity w
% q = R2q(R)                    - Converts a rotation matrix to unit quaternion
% q = R2q_rt(R)                 - Converts a rotation matrix to unit quaternion
% rpy = R2rpy(R)                - Convert a rotation matrix to X-Y-Z rotation sequence (around the new axis) 
% q = rpy2q(r)                  - Convert a X-Y-Z rotation sequence (around the new axis) to unit quaternion  
% R = rpy2R(r)                  - Convert a X-Y-Z rotation sequence (around the new axis) to rotation matrix 
% Rx = rx(A)                    - Returns a rotation matrix (X axis)
% Ry = ry(A)                    - Returns a rotation matrix (Y axis)
% Rz = rz(A)                    - Returns a rotation matrix (Z axis)
% qm = slerp(qi,qn,t,eps)       - Spherical Linear Interpolation (Slerp) using quaternions  
% e = R_err(R1,R2)              - Forms an orientation error between two rotation matrices
% w = qq2w(q1,q2,d_time)        - Compute angular velocity w given two quaternions q1,q2
% w = RR2w(R1,R2,d_time)        - Compute angular velocity w given two rotation matrices R1,R2
%

% Solvers (./solvers)
% ----------------------
% SV = int_euler(SV,d_time)     - Euler integration (implicit or explicit)
% SV = int_rk4(SP,SV,...)       - Runge-Kutta 4th order method
%

% Models (./models)
% ----------------------
% model_system_example          - model from the documentation
% model_system_0                - only the base
% model_system_2                - two links (planar system)
% model_system_3                - three links (3D system)
% model_ABB                     - ABB IRB140 manipulator (6 DOF RRRRRR)
% model_rand(NL)                - creates a random system with NL links
% model_7dof                    - 7DoF manipulator + 3 reaction wheels
% model_NAO                     - model of NAO robot 
% model_test                    - this model is for testing purposes
%

% Examples (./examples)
% ----------------------
% test_rot_seq                  - Demonstrate alternative sequences of Euler angles
% test_slerp                    - Test the slerp function
% test_trajC                    - Cartesian trajectory generation 
% example_1                     - using bMSd
% example_ID                    - solving the inverse dynamics problem
% example_JT                    - Jacobian transpose
% example_dJ                    - derivative w.r.t. time of Jacobians
% example_IK_ABB                - inverse kinematics for ABB IRB 140
%
