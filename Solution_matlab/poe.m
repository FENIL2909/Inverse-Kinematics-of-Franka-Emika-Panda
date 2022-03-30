% RBE 501 - Robot Dynamics - Fall 2021
% Midterm Exam
% Worcester Polytechnic Institute
%
% Student: ***<FENIL DESAI>***
clear, clc, close all
addpath('utils');

plotOn = true; %toggle to false to switch off the robot visualization
ploterrorOn = true;  %toggle to false to switch off the error plots in ik.m
nTests = 20; % number of random test configurations

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


%% Calculate the forward kinematics using the Product of Exponentials
% Let us calculate the screw axis for each joint
% Put all the axes into a 6xn matrix S, where n is the number of joints
S = [0 0 1 0 0 0; 0 1 0 -L1 0 0; 0 0 1 0 0 0; 0 -1 0 (L1+L2) 0 -L5; 0 0 1 0 0 0; 0 -1 0 (L1+L2+L3) 0 0; 0 0 -1 0 L0 0]';

% Let us also calculate the homogeneous transformation matrix M for the
% home configuration
M = [1 0 0 0; 0 -1 0 0; 0 0 -1 0; L0 0 (L1 + L2 + L3 - L4) 1]';


fprintf('---------------------Forward Kinematics Test---------------------\n');
fprintf(['Testing ' num2str(nTests) ' random configurations.\n']);
fprintf('Progress: ');
nbytes = fprintf('0%%'); 
 
% Testing the forward kinematics for 20 random sets of joint variables
for ii = 1 : nTests
    fprintf(repmat('\b',1,nbytes));
    nbytes = fprintf('%0.f%%', ceil(ii/nTests*100));
    
    %Generating a random configuration
    q = [qlim(1,1) + (qlim(1,2) - qlim(1,1)) * rand(), ...
         qlim(2,1) + (qlim(2,2) - qlim(2,1)) * rand(), ...
         qlim(3,1) + (qlim(3,2) - qlim(3,1)) * rand(), ...
         qlim(4,1) + (qlim(4,2) - qlim(4,1)) * rand(), ...
         qlim(5,1) + (qlim(5,2) - qlim(5,1)) * rand(), ...
         qlim(6,1) + (qlim(6,2) - qlim(6,1)) * rand(), ...
         qlim(7,1) + (qlim(7,2) - qlim(7,1)) * rand()];
    
    %Calculating the forward kinematics
    T = fkine(S,M,q);
    
    if plotOn
        robot.teach(q);
        title('Forward Kinematics Test');
    end
    
    %Checking the correctness of the forward kinematics developed by me
    assert(all(all(abs(double(robot.fkine(q)) - T) < 1e-10)));
end
 
fprintf('\nTest passed successfully.\n');