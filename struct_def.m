% ------------------------------------------------------
% | Basic Multibody Simulator Derived (Matlab toolbox) |
% ------------------------------------------------------
%
% For more detailed and up-to-date description see Lecture 9
%
%-------------------
% General structure:
%-------------------
%
% CoM - Center of Mass
%
% CF  - Coordinate Frame
%
% WF  - World frame. This is an inertial frame fixed in the environment 
%
% DoF - Degree of Freedom
%
% PL  - Parent Link. PL of a link is the link immediately below it in the kinematic tree 
%                    (the link that "supports" it). Every link has only one PL 
%                    (the PL for the base is the "environment"). A link can be a PL of 
%                    zero, one or many links. 
%
% IJ  - Input joint. This is the joint connecting a link and its PL. 
%                    Only two types of joints can be defined (Prismatic and Revolute). Note that,
%                    any other joint could be modeled as a combination of these two.
%                    The base could be either "rigidly" connected to the environment (0 DoF) 
%                    or can be free-floating (6 DoF) - see SP.mode
%
% Numbering of links and joints:
%                    The environment has number 0
%                    The base link has number 1
%                    The next link has number 2 etc.
%                    Joint 1 is the IJ for link 2 (connecting link 1 and link 2)
%                    In general joint i is an IJ for link i+1
%
% We define a CF fixed in the input joint of each link. The CF of link 1 is fixed in its CoM
%
%-------------------
% System Parameters:
%-------------------
%
% SP.C             -: connectivity vector. SP.C(i) is the PL of link "i".
% {integer}           Example:
%                     --------
%                     SP.C = [0 1 2 2 1 4] means:
%                     PL for link 1 (the base) is the environment 
%                     PL for 2 is link 1
%                     PL for 3 is link 2
%                     PL for 4 is link 2
%                     PL for 5 is link 1 (the base)
%                     PL for 6 is link 4
%          
%                     Note that, link 2 is a PL for links 3 and 4
%
% SP.n             -: number of joints (number of links - 1)
% {integer}
%
% SP.mode          -: if SP.mode ~= 0 -> fixed base manipulator
% {integer}           if SP.mode == 0 -> free-floating system
%
% SP.J(i).t        -: vector from the CoM of link SP.C(i+1) to joint i expressed in the CF of link SP.C(i+1)
%
% SP.J(i).f        -: vector from joint i to the CoM of link i+1 expressed in the CF of link i+1
% [3x1] {double}      
%
% SP.J(i).rpy      -: orientation of the CF of link i relative to the CF of link SP.C(i)
% [3x1] {double}      expressed as X-Y-Z Euler angles (current axis).
%
% SP.J(i).type     -: type of joint i ('R' for revolute, 'P' for prismatic). The local z axis of the
% [1x1] {string}      CF of link i+1 is the axis of rotation/translation of joint i.
%
% SP.L(i).m        -: mass of link i
% [1x1] {double}
%
% SP.L(i).I        -: inertia of link i expressed in the CF of link i (computed about the CoM)
% [3x3] {double}
% 
% SP.bN            -: links with end-effectors
% [1xm] {integer}     Example :
%                     --------
%                     bN = [5 2 1] - end-effectors on links 5, 2 and link 1 (the base) 
%        
% SP.bP            -: bP(:,i) is vector from the CoM of link bN(i) to end-effector "i".  
% [3xm] {double}      Example:
%                     ---------
%                     bN = [3 3 7] and bP = [0.5    1    0;
%                                              0    0    1;
%                                              0  0.2  0.3];
%                     means that on link 3 there are two end-effectors    
%                     and on link 7 there is one bP(:,3)
%
%-------------------
% System Variables
%-------------------
%
% SV.q             -: SV.q(i) is the angle of joint "i" 
% [SP.nx1] {double}   user input for i=1:SP.n
%
% SV.dq            -: SV.dq(i) is the velocity of joint "i" 
% [SP.nx1] {double}   user input for i=1:SP.n
%
% SV.ddq           -: SV.ddq(i) is the acceleration of joint "i" 
% [SP.nx1] {double}   
%
% SV.tau           -: SV.tau(i) is the torque applied by the motor in joint "i" 
% [SP.nx1] {double}   user input for i=1:SP.n
%
% SV.L(i).R        -: rotation matrix representing the orientation of link i with respect to the WF 
% [3x3] {double}      user input for i=1 (this can be used to set the initial base attitude)
%                     Example:
%                     --------
%                     v_WF = SV.L(i).R * v_i
%                     v_i  - is a vector expressed in CF of link i
%                     v_WF - is vector v_i expressed in the WF
%
%                     v_i = SV.L(i).R' * v_WF can be used to transform vectors from the WF
%                     to the link-fixed frames
%
% SV.L(i).Q        -: quaternion describing the orientation of CF of link i with respect to the WF 
% [4x1] {double}      user input for i=1 (this can be used to set the initial base attitude)
%                     Note: 
%                     -----
%                     this is just another way to parametrize orientation
%
% SV.L(i).p        -: position of the CoM of link i expressed in the WF. 
% [3x1] {double}      user input for i=1 (this can be used to set the initial position of the base)
%                 
%
% SV.L(i).v        -: linear velocity of the CoM of link i expressed in the WF (the derivative is taken in the WF). 
% [3x1] {double}      user input for i=1 (this can be used to set the initial linear velocity of the base)
%                     Note: 
%                     -----                     
%
% SV.L(i).dv       -: linear acceleration of the CoM of link i expressed in the WF (the derivative is taken in the WF).
% [3x1] {double}      
%                     
%  
% SV.L(i).w        -: angular velocity of link i expressed in the WF (the derivative is taken in the WF). 
% [3x1] {double}      user input for i=1 (this can be used to set the initial angular velocity of the base)
%                     
%
% SV.L(i).dw       -: angular acceleration of link i expressed in the WF (the derivative is taken in the WF). 
% [3x1] {double}      
%                     
%
% SV.L(i).T        -: external torque applied to link i (expressed in the WF). 
% [3x1] {double}      user input for i=1:SP.n
%
% SV.L(i).F        -: external force applied to the CoM of link i (expressed in the WF). 
% [3x1] {double}      user input for i=1:SP.n
%

%%%EOF