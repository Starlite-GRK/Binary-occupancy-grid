clear all;
clc;
close all;
x_max=25;
y_max=25;
x_s=2;
y_s=1;
x_t=9;
y_t=9;
walls= zeros(x_max,y_max);
walls(1,:)=1;
walls(:,1)=1;
walls(end,:)=1;
walls(:,end)=1;
obstacle_s1=[5 5];
obstacle_l1=[2 2];
for i=obstacle_s1(1):obstacle_s1(1)+obstacle_l1(1)
    for j=obstacle_s1(2):obstacle_l1(2)+obstacle_s1(2)
    walls(i,j)=1;
    end
end
obstacle_s2=[10 10];
obstacle_l2=[2 2];
for i=obstacle_s2(1):obstacle_s2(1)+obstacle_l2(1)
    for j=obstacle_s2(2):obstacle_l2(2)+obstacle_s2(2)
    walls(i,j)=1;
    end
end
walls(8:end,4)=1;
MAP = binaryOccupancyMap(walls,1);
show(MAP)

prm= mobileRobotPRM;
prm.NumNodes=300;
prm.ConnectionDistance=3;
prm.Map=MAP;
show(prm);
disp('click start position');
startlocation=ginput(1);
disp('click end position');
endlocation=ginput(1);
path=findpath(prm,startlocation,endlocation)
while isempty(path)
    prm.NumNodes=prm.NumNodes+5;
    update(prm);
    path=findpath(prm,startlocation,endlocation)
    show(prm)
    pause(10);
end
show(prm)

%robot
robot = differentialDriveKinematics("TrackWidth", 1, "VehicleInputs", "VehicleSpeedHeadingRate");
frameSize = robot.TrackWidth/0.8;
%controller
controller = controllerPurePursuit;
controller.Waypoints = path;
controller.DesiredLinearVelocity = 0.6;
controller.MaxAngularVelocity = 2;
controller.LookaheadDistance = 0.3;
%SIMULATION
robotInitialLocation = path(1,:);
robotGoal = path(end,:);
initialOrientation = 0;
robotCurrentPose = [robotInitialLocation initialOrientation]';
distanceToGoal = norm(robotInitialLocation - robotGoal);
goalRadius = 0.1;
sampleTime = 0.1;
vizRate = rateControl(1/sampleTime);

% Initialize the figure
figure

while( distanceToGoal > goalRadius )
    
    % Compute the controller outputs, i.e., the inputs to the robot
    [v, omega] = controller(robotCurrentPose);
    
    % Get the robot's velocity using controller inputs
    vel = derivative(robot, robotCurrentPose, [v omega]);
    
    % Update the current pose
    robotCurrentPose = robotCurrentPose + vel*sampleTime; 
    
    % Re-compute the distance to the goal
    distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal(:))
    % Update the plot
    hold off
    show(MAP);
    hold all

    % Plot path each instance so that it stays persistent while robot mesh
    % moves
    plot(path(:,1), path(:,2),"k--d")
    
    % Plot the path of the robot as a set of transforms
    plotTrVec = [robotCurrentPose(1:2); 0];
    plotRot = axang2quat([0 0 1 robotCurrentPose(3)]);
    plotTransforms(plotTrVec', plotRot, 'MeshFilePath', 'groundvehicle.stl', 'Parent', gca, "View","2D", "FrameSize", frameSize);
    waitfor(vizRate);
end
