%creating kinova robot model
gen3 = loadrobot("kinovaGen3");
gen3.DataFormat = 'column';
q_home = [0 15 180 -130 0 55 90]'*pi/180;
eeName = 'EndEffector_Link';
T_home = getTransform(gen3, q_home, eeName); 

%viewing the robot in matlab
show(gen3,q_home);
axis auto;
view([60,10]);

%creating Inverse Kinematics solver
ik = inverseKinematics('RigidBodyTree', gen3);
ik.SolverParameters.AllowRandomRestart = false;
weights = [1, 1, 1, 1, 1, 1];
q_init = q_home;

%creating waypoints 
%pick things up

dt = 0.25;
t_up = (0:dt:5)';
t_middle_1 = (5.25:dt:20)';
t_middle_2 = (20.25:dt:35)';
t_down = (35.25:dt:40)';
middle=[0.5 -0.5 0.5];
start=[0.5 -0.5 0.1];
points1 = start + [zeros(size(t_up)) zeros(size(t_up)) (t_up/t_up(end))*0.4];
theta = (t_middle_1-t_middle_1(1))*(pi/(t_middle_1(end)-t_middle_1(1)));
points2 = middle + [zeros(size(theta)) (t_middle_1-t_middle_1(1))*(0.5/(t_middle_1(end)-t_middle_1(1))) 0.2*sin(theta)];
gamma = -((t_middle_2-t_middle_2(1))*(pi/(t_middle_2(end)-t_middle_2(1))));
points3 = points2(end,:) + [zeros(size(theta)) (t_middle_2-t_middle_2(1))*(0.5/(t_middle_2(end)-t_middle_2(1))) 0.2*sin(gamma)];
points4=points3(end,:)+[zeros(size(t_down)) zeros(size(t_down)) -((t_down-t_down(1))/(t_down(end)-t_down(1)))*0.4];
points=[points1;points2;points3;points4];





%diplaying the waypoints
hold on;
plot3(points(:,1),points(:,2),points(:,3),'-*g', 'LineWidth', 1.5);
xlabel('x');
ylabel('y');
zlabel('z');
axis auto;
view([60,10]);
grid('minor');

%Inverse Kinematics for each waypoints

numJoints = size(q_home,1);   
numWaypoints1 = size(points1,1);
numWaypoints2 = size(points2,1);
numWaypoints3 = size(points3,1);
numWaypoints4 = size(points4,1);
qs1 = zeros(numWaypoints1,numJoints);
qs2 = zeros(numWaypoints2,numJoints);
qs3 = zeros(numWaypoints3,numJoints);
qs4 = zeros(numWaypoints4,numJoints);
for i = 1:numWaypoints1
    T_des = T_home;
    T_des(1:3,4) = points1(i,:)';
    [q_sol, q_info] = ik(eeName, T_des, weights, q_init);
    
    % Display status of ik result
    %disp(q_info.Status);
    
    % Store the configuration
    qs1(i,:) = q_sol(1:numJoints); 
    
    % Start from prior solution
    q_init = q_sol;
end
for i = 1:numWaypoints2
    T_des = T_home;
    T_des(1:3,4) = points2(i,:)';
    [q_sol, q_info] = ik(eeName, T_des, weights, q_init);
    
    % Display status of ik result
    %disp(q_info.Status);
    
    % Store the configuration
    qs2(i,:) = q_sol(1:numJoints); 
    
    % Start from prior solution
    q_init = q_sol;
end
for i = 1:numWaypoints3
    T_des = T_home;
    T_des(1:3,4) = points3(i,:)';
    [q_sol, q_info] = ik(eeName, T_des, weights, q_init);
    
    % Display status of ik result
    %disp(q_info.Status);
    
    % Store the configuration
    qs3(i,:) = q_sol(1:numJoints); 
    
    % Start from prior solution
    q_init = q_sol;
end
for i = 1:numWaypoints4
    T_des = T_home;
    T_des(1:3,4) = points4(i,:)';
    [q_sol, q_info] = ik(eeName, T_des, weights, q_init);
    
    % Display status of ik result
    %disp(q_info.Status);
    
    % Store the configuration
    qs4(i,:) = q_sol(1:numJoints); 
    
    % Start from prior solution
    q_init = q_sol;
end
%animate the solution
figure; set(gcf,'Visible','on');
ax = show(gen3,qs1(1,:)');
ax.CameraPositionMode='auto';
hold on;
 
% Plot waypoints
plot3(points(:,1),points(:,2),points(:,3),'-g','LineWidth',2);
axis auto;
view([60,10]);
grid('minor');
hold on;
 
title('Simulated Movement of the Robot');
% Animate
framesPerSecond = 30;
r = robotics.Rate(framesPerSecond);
for i = 1:numWaypoints1
    show(gen3, qs1(i,:)','PreservePlot',false);
    drawnow;
    waitfor(r);
end
for i = 1:numWaypoints2
    show(gen3, qs2(i,:)','PreservePlot',false);
    drawnow;
    waitfor(r);
end
for i = 1:numWaypoints3
    show(gen3, qs3(i,:)','PreservePlot',false);
    drawnow;
    waitfor(r);
end
for i = 1:numWaypoints4
    show(gen3, qs4(i,:)','PreservePlot',false);
    drawnow;
    waitfor(r);
end
%calculating joint velocity and acceleration at each waypoints
%motion 1
qs_deg1 = qs1*180/pi;
vel1 = diff(qs_deg1)/dt;
vel1(1,:) = 0;
vel1(end+1,:) = 0;
acc1 = diff(vel1)/dt;
acc1(1,:) = 0;
acc1(end+1,:) = 0;

%motion 2
qs_deg2 = qs2*180/pi;
vel2 = diff(qs_deg2)/dt;
vel2(1,:) = 0;
vel2(end+1,:) =0;
acc2 = diff(vel2)/dt;
acc2(1,:) = 0;
acc2(end+1,:) = 0;

%motion 3
qs_deg3 = qs3*180/pi;
vel3 = diff(qs_deg3)/dt;
vel3(1,:) = 0;
vel3(end+1,:) = 0;
acc3 = diff(vel3)/dt;
acc3(1,:) = 0;
acc3(end+1,:) = 0;

%motion 4
qs_deg4 = qs4*180/pi;
vel4 = diff(qs_deg4)/dt;
vel4(1,:) = 0;
vel4(end+1,:) = 0;
acc4 = diff(vel4)/dt;
acc4(1,:) = 0;
acc4(end+1,:) = 0;

% Interpolate the joint position, velocity and acceleration to ensure the 0.001 seconds time step between two trajectory points

timestamp1 = 0:0.001:t_up(end);
qs_deg1 = interp1(t_up,qs_deg1,timestamp1);
vel1 = interp1(t_up,vel1,timestamp1);
acc1 = interp1(t_up,acc1,timestamp1);

timestamp2 = 0:0.001:(t_middle_1(end)-t_middle_1(1));%t_middle(1):0.001:t_middle(end);
qs_deg2 = interp1(t_middle_1-t_middle_1(1),qs_deg2,timestamp2);
vel2 = interp1(t_middle_1-t_middle_1(1),vel2,timestamp2);
acc2 = interp1(t_middle_1-t_middle_1(1),acc2,timestamp2);

timestamp3 = 0:0.001:(t_middle_2(end)-t_middle_2(1));%t_middle(1):0.001:t_middle(end);
qs_deg3 = interp1(t_middle_2-t_middle_2(1),qs_deg3,timestamp3);
vel3 = interp1(t_middle_2-t_middle_2(1),vel3,timestamp3);
%vel3(1:3,1:end);
%vel3(end-3:end,1:end)
acc3 = interp1(t_middle_2-t_middle_2(1),acc3,timestamp3);

timestamp4 = 0:0.001:(t_down(end)-t_down(1));
qs_deg4 = interp1(t_down-t_down(1),qs_deg4,timestamp4);
vel4 = interp1(t_down-t_down(1),vel4,timestamp4);
acc4 = interp1(t_down-t_down(1),acc4,timestamp4);



% robot connection
Simulink.importExternalCTypes(which('kortex_wrapper_data.h'));
gen3Kinova = kortex;
gen3Kinova.ip_address = '192.168.1.10';
 
isOk = gen3Kinova.CreateRobotApisWrapper();
if isOk
   disp('You are connected to the robot!'); 
else
   error('Failed to establish a valid connection!'); 
end

%visualize the actual movement of robot
title('Actual Movement of the Robot');
[~,~, actuatorFb, ~] = gen3Kinova.SendRefreshFeedback();
show(gen3, ((actuatorFb.position)*pi/180)','PreservePlot',false);
drawnow;

%send robot to starting point of the trajectory
jointCmd = wrapTo360(qs_deg1(1,:));
constraintType = int32(0);
speed = 0;
duration = 0;

isOk = gen3Kinova.SendJointAngles(jointCmd, constraintType, speed, duration);
if isOk
    disp('success');
else
    disp('SendJointAngles cmd error');
    return;
end

%check if the robot has reached the starting position
while 1
    [isOk,~, actuatorFb, ~] = gen3Kinova.SendRefreshFeedback();
    show(gen3, ((actuatorFb.position)*pi/180)','PreservePlot',false);
    drawnow;
    if isOk
        if max(abs(wrapTo360(qs_deg1(1,:))-actuatorFb.position)) < 0.1
            disp('Starting point reached.')
            break;
        end 
    else
        error('SendRefreshFeedback error')
    end
end

%closing grip
toolCommand = int32(3);    % position control mode
toolDuration = 0;
toolCmd = 0.85; 
isOk = gen3Kinova.SendToolCommand(toolCommand, toolDuration, toolCmd);
if isOk
    disp('Command sent to the gripper. Wait for the gripper to close.')
else
    error('Command Error.');
end
pause(3);

%send pre-computed trajectory
isOk = gen3Kinova.SendPreComputedTrajectory(qs_deg1.', vel1.', acc1.', timestamp1, size(timestamp1,2));
if isOk
    disp('Send1stPreComputedTrajectory success');
else
    disp('Send1stPreComputedTrajectory command error');
end

while 1
    [isOk,~, actuatorFb, ~] = gen3Kinova.SendRefreshFeedback();
    show(gen3, ((actuatorFb.position)*pi/180)','PreservePlot',false);
    drawnow;
    if isOk
        if max(abs(wrapTo360(qs_deg1(end,:))-actuatorFb.position)) < 0.1
            disp('1stEnd Point reached.')
            break;
        end 
    else
        error('SendRefreshFeedback error')
    end
end

isOk = gen3Kinova.SendPreComputedTrajectory(qs_deg2.', vel2.', acc2.', timestamp2, size(timestamp2,2));
if isOk
    disp('Send2ndPreComputedTrajectory success');
else
    disp('Send2ndPreComputedTrajectory command error');
end

while 1
    [isOk,~, actuatorFb, ~] = gen3Kinova.SendRefreshFeedback();
    show(gen3, ((actuatorFb.position)*pi/180)','PreservePlot',false);
    drawnow;
    if isOk
        if max(abs(wrapTo360(qs_deg2(end,:))-actuatorFb.position)) < 0.1
            disp('2ndEnd Point reached.')
            break;
        end 
    else
        error('SendRefreshFeedback error')
    end
end



isOk = gen3Kinova.SendPreComputedTrajectory(qs_deg3.', vel3.', acc3.', timestamp3, size(timestamp3,2));
if isOk
    disp('Send3rdPreComputedTrajectory success');
else
    disp('Send3rdPreComputedTrajectory command error');
end



%check if the robot has reach the end position
while 1
    [isOk,~, actuatorFb, ~] = gen3Kinova.SendRefreshFeedback();
    show(gen3, ((actuatorFb.position)*pi/180)','PreservePlot',false);
    drawnow;
    if isOk
        if max(abs(wrapTo360(qs_deg3(end,:))-actuatorFb.position)) < 0.1
            disp('End Point reached.')
            break;
        end 
    else
        error('SendRefreshFeedback error')
    end
end

isOk = gen3Kinova.SendPreComputedTrajectory(qs_deg4.', vel4.', acc4.', timestamp4, size(timestamp4,2));
if isOk
    disp('Send3rdPreComputedTrajectory success');
else
    disp('Send3rdPreComputedTrajectory command error');
end



%check if the robot has reach the end position
while 1
    [isOk,~, actuatorFb, ~] = gen3Kinova.SendRefreshFeedback();
    show(gen3, ((actuatorFb.position)*pi/180)','PreservePlot',false);
    drawnow;
    if isOk
        if max(abs(wrapTo360(qs_deg4(end,:))-actuatorFb.position)) < 0.1
            disp('End Point reached.')
            break;
        end 
    else
        error('SendRefreshFeedback error')
    end
end

%opening grip
toolCommand = int32(3);    % position control mode
toolDuration = 0;
toolCmd = 0; 
isOk = gen3Kinova.SendToolCommand(toolCommand, toolDuration, toolCmd);
if isOk
    disp('Command sent to the gripper. Wait for the gripper to open.')
else
    error('Command Error.');
end
pause(3);
%disconnect the robot
isOk = gen3Kinova.DestroyRobotApisWrapper();