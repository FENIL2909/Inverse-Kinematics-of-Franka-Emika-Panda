function robot = make_robot(digit)
%MAKE_ROBOT Creates the kinematic structure of the robot used in the exam
%   This is a factory method that creates the robot needed for the exam.
%
%   Inputs: digit [int] - may be used to tweak the robot model
%   Output: robot - the robot structure, created using Peter Corke's
%   robotics toolbox
%
%   Author: L. Fichera <lfichera@wpi.edu>
%   Last modified: 10/8/2021

%% Create the manipulator
mdl_panda            % load the model
panda.tool = eye(4); % take the tool off the end effector
qlim = panda.qlim;   % read the joint limits


% If the digit is an odd number, change the direction of some of the axes
if mod(digit, 2) == 1
    panda = SerialLink([RevoluteMDH('d', 0.333, 'a',  0,      'alpha', 0), ...
                        RevoluteMDH('d',     0, 'a',  0,      'alpha', pi/2), ...
                        RevoluteMDH('d', 0.316, 'a',  0,      'alpha', -pi/2), ...
                        RevoluteMDH('d',     0, 'a',  0.0825, 'alpha', -pi/2), ...
                        RevoluteMDH('d', 0.384, 'a', -0.0825, 'alpha', pi/2), ...
                        RevoluteMDH('d',     0, 'a',  0,      'alpha', -pi/2), ...
                        RevoluteMDH('d', 0.107, 'a',  0.088,  'alpha', -pi/2)], ...
                        'name', 'PANDA', 'manufacturer', 'Franka-Emika');
    
    % Need to update joint limits as well
    panda.qlim = qlim;
    panda.qlim(4,:) = [0.0698 3.0718];
    panda.qlim(6,:) = [-3.7525 0.0175];
    
end

% Return the robot
robot = panda;

end

