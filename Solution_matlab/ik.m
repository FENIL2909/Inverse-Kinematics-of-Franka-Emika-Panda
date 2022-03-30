% RBE 501 - Robot Dynamics - Fall 2021
% Midterm Exam
% Worcester Polytechnic Institute
%
% Student: ***<FENIL DESAI>***

clear, clc, close all
addpath('utils');


% First, execute poe.m to load the S and M matrices
poe
close all

% Generate and display the path that the robot has to trace
t = linspace(-pi, pi, 36);
x = 0.5  * ones(1,36);
a = 0.4;
y = (16 * (sin(t)).^3)./50;
z = (13 * cos(t) - 5 * cos(2*t) - 2 * cos(3*t) - cos(4*t))./50 + 0.75;
path = [x; y; z];

scatter3(path(1,:), path(2,:), path(3,:), 'filled');


% Convert Cartesian coordinates into twists
targetPose = zeros(6,size(path,2)); % each column of this matrix is a target pose represented by a twist

for ii = 1 : size(path,2)
    % First calculate the homogeneous transformation matrix representing
    % each target pose
    R = [0 0 -1; 0 1 0; 1 0 0]';
    T = [R path(:,ii); 
         0 0 0 1];
     
    % Then perform the matrix logarithm operation to convert transformation
    % matrices into 4x4 elements of se(3)
    t = MatrixLog6(T);
    
    % Finally, "unpack" this matrix (i.e., perform the inverse of the
    % bracket operator)
    targetPose(:,ii) = [t(3,2) t(1,3) t(2,1) t(1:3,4)']';
end


%% Calculate the inverse kinematics 
fprintf('----------------------Inverse Kinematics Test--------------------\n');
fprintf('Progress: ');
nbytes = fprintf('0%%');

% Calculating the twist representing the robot's home pose
currentPose = MatrixLog6(M);
currentPose = [currentPose(3,2) currentPose(1,3) currentPose(2,1) currentPose(1:3,4)']';

% Initializing the current joint variables
currentQ = zeros(1,7);


%inverse kinematics
tic  % start timer
for ii = 1 : size(targetPose,2)
    fprintf(repmat('\b',1,nbytes));
    nbytes = fprintf('%0.f%%', ceil(ii/size(targetPose,2)*100));

    iterations = 0;
    x_graph = [];
    y_graph = [];

    % Inverse Kinematics
    while (norm(targetPose(:,ii) - currentPose) > 1e-3)
        J = jacob0(S,currentQ);         % calculatinf using currentQ
        error  = norm(targetPose(:,ii) - currentPose);

        %Newton-Raphson
        %deltaQ = pinv(J)*(targetPose(:,ii) - currentPose);  
        
        % Damped-Least Square method
        lambda = 0.001;
        J_star = J'*pinv(J*J' + (lambda^2)*eye(6));
        %deltaQ = J_star*(targetPose(:,ii) - currentPose);

        % gradient method
        alpha = 0.35;
        %deltaQ = alpha*transpose(J)*(targetPose(:,ii) - currentPose);

        
        %combined method
        if error <= 0.1
            deltaQ = J_star*(targetPose(:,ii) - currentPose);
        else 
            deltaQ = alpha*transpose(J)*(targetPose(:,ii) - currentPose);
        end


        % for error plots
        iterations = iterations + 1;
        x_graph(iterations) = iterations; 
        y_graph(iterations) = (norm(targetPose(:,ii) - currentPose));
        
        if ploterrorOn
            figure(2)
            plot(x_graph,y_graph)
            title(sprintf('Gradient + Damped Least Sqaure for point: %d', ii))
            xlabel('iterations')
            ylabel('norm(target-pose - current-pose)')
            hold on
        end
        
        %updating the current joint variables
        currentQ = currentQ + deltaQ';
        
        T = fkine(S,M,currentQ);
        currentPose = MatrixLog6(T);
        currentPose = [currentPose(3,2) ...
                       currentPose(1,3) ...
                       currentPose(2,1) ...
                       currentPose(1:3,4)']';
    end
    %generating the list of joint varibales to reach the given target points
    qList(:,ii) = currentQ;

    if ploterrorOn
        pause(1)
        clf(figure(2));
    end
end
toc  % stop timer


qList = qList'   % printing list of joint varibales to move the end-effector in a closed circle (taking a heart-shape)

figure(1)
robot.plot(qList, 'trail', {'r', 'LineWidth', 5});  %visualizing the robot movement
fprintf('\nTest passed successfully.\n');


