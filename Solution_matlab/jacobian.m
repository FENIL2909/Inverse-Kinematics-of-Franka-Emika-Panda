% RBE 501 - Robot Dynamics - Fall 2021
% Midterm Exam
% Worcester Polytechnic Institute
%
% Student: ***<FENIL DESAI>***
clear, clc, close all
addpath('utils');

plotOn = true;
%% *** ENTER THE SECOND DIGIT OF YOUR WPI ID BELOW: ***
digit = 4;

%% Create the manipulator

%robot link lengths
L0 = 0.0880;
L1 = 0.3330;
L2 = 0.3160;
L3 = 0.3840;
L4 = 0.1070;
L5 = 0.0825;

robot = make_robot(digit);
n = robot.n;
qlim = robot.qlim;


%% Calculate the Jacobian matrix in the home configuration

% Let us calculate the screw axis for each joint
% Put all the axes into a 6xn matrix S, where n is the number of joints
S = [0 0 1 0 0 0; 0 1 0 -L1 0 0; 0 0 1 0 0 0; 0 -1 0 (L1+L2) 0 -L5; 0 0 1 0 0 0; 0 -1 0 (L1+L2+L3) 0 0; 0 0 -1 0 L0 0]';

% Let us also calculate the homogeneous transformation matrix M for the
% home configuration
M = [1 0 0 0; 0 -1 0 0; 0 0 -1 0; L0 0 (L1 + L2 + L3 - L4) 1]';

q = zeros(1,7);

% Calculating the Forward Kinematics
T = fkine(S,M,q);

J = jacob0(S,q)
     
if plotOn
    robot.teach(q);
    title('Differential Kinematics Test');
end
    
% Testing the correctness of the Jacob0 function dveeloped by me
Jcoords = [-skew(T(1:3,4))*J(1:3,:)+J(4:6,:); J(1:3,:)];
assert(all(all(abs(double(robot.jacob0(q)) - Jcoords) < 1e-10)));

fprintf('\nTest passed successfully.\n');