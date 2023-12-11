
clear;
nos = ["twentyfive"];

for ni=1:length(nos)
    no = nos(ni);
    %waypoints = load(sprintf("field_6/%s_stops.txt",no));
    waypoints = [ 6 0; 6 10;0 10; 0 0; 5 5; 0 0 ];
    %spots = load(sprintf("field_6/%s_spots.txt",no));
    %robot= [-6.30 4.78 -97.85];
    robot = [0 0 0];
    initial = robot;
    sortedwaypoints =  waypoints; %GoalSequence(waypoints,initial);
    x_initial = initial(1,1);
    y_initial = initial(1,2);
    yaw_initial = toRadian(initial(1,3));
    % Goal position [x(m),y(m)]
    % Obstacle list [x(m) y(m)]
    obstacle = load('obstacle.txt'); % You can load the obstacle from text file
    obstacleR=0.5;% Radius of obstacle for collision detection
    global dt; dt=0.1; % Stepping time [s]
    % %%%%Robotic Kinematic Variable%%%%%%%
    % Maximum Linear Velocity [m / s]
    % Maximum Angular Velocity [rad / s], 
    % Maximum acceleration / deceleration [m / s^2], 
    % Maximum Angular acceleration / deceleration [rad / s^2],
    % Velocity resolution [m / s]
    % Angular velocity resolution [rad / s]]
    Kinematic=[1.0,1.0,5.0,3.2,0.1,toRadian(1)];
    % Evaluation function parameters [distance, heading,obsCost,velocity,predictDT]
    evalParam=[24,32,0,0.1,3.0]; % alpha, beta, gamma, evaldt
    area=[min(waypoints(:,1))-4 max(waypoints(:,1))+4 min(waypoints(:,2))-4 max(waypoints(:,2))+4];% The size of the simulation area [xmin xmax ymin ymax]
    % simulation result
    result.x=[]; % Accumulate and store the state value of the trajectory points traversed
    result.waypoints=[];
    result.time = [];
    result.u = [];
    result.path = [];
 
    tic; % Start of estimating program running time
    %movcount=0;
    % Main loop
    % Robot initial state [x(m),y(m),yaw(Rad),v(m/s),?(rad/s)]
    x = [x_initial y_initial yaw_initial 0 0];
        path = 1;
        for io=1:length(sortedwaypoints)
            x_goal = sortedwaypoints(io,1);
            y_goal = sortedwaypoints(io,2);

            goal = [x_goal,y_goal];
            result.waypoints = [result.waypoints;goal];
            
            initial = x(1:2);
            for i=1:5000 % Cycle operation 5000 times, guide to reach the destination or end of 5000 times operation
            % Calculation of input value by DWA
                [u,traj]=DynamicWindowApproach(x,Kinematic,goal,evalParam,obstacle,obstacleR);
            %Stop Simulation
               
                x=f(x,u);%Movement by motion model
            %Saving simulation results
                result.u = [result.u;u'];
                result.x=[result.x; x]; % The latest result is added to result.x as a column
            %Goal judgment
                path = path + 1;
                if norm(x(1:2)-goal)<0.5
                    elapsed_time = toc;
                    result.time = [result.time;i*dt];
                    result.path = [result.path;path];
                    break;
                end
                %x
           %====Animation====
                hold off;
                ArrowLength=1.0;%Arrow length
                %robot
                
                quiver(x(1),x(2),ArrowLength*cos(x(3)),ArrowLength*sin(x(3)),'r');hold on;
                plot(result.x(:,1),result.x(:,2),'-b');hold on;
                plot(result.waypoints(:,1),result.waypoints(:,2),'or');hold on;
                plot(obstacle(:,1),obstacle(:,2),'*k');hold on;
                %plot(spots(:,1),spots(:,2),'*b');hold on;
                % Search trajectory display
                if ~isempty(traj)
                    for it=1:length(traj(:,1))/5
                        ind=1+(it-1)*5;
                        %plot(traj(ind,:),traj(ind+1,:),'-g');hold on;
                    end
                end
                axis(area);
                grid on;
                drawnow;
                
           
            end
        end
        
    
    result.distances = [];
    start = 1;
    for i=1:length(result.path)
        current = result.path(i) - 1;
        route = result.x(start:current,1:2);
        total_distance=0;
        for j=1:length(route)-1
            total_distance = total_distance + norm(route(j,:) - route(j+1,:))
        end
        result.distances = [result.distances;total_distance]
        start = current;
    end
    spot_to_spot_distance = sum(result.distances(2:length(result.distances)-1));
    spot_to_spot_time = sum(result.time(2:length(result.time)-1));
    figure;
    hold off;
    %scatter(spots(:,1),spots(:,2),"*r");hold on;
    scatter(waypoints(:,1),waypoints(:,2),"ob");hold on;
    plot(waypoints(:,1),waypoints(:,2),"-b");hold on;
    scatter(robot(:,1),robot(:,2),"^b");hold on;
    plot(result.x(:,1),result.x(:,2),"-r");hold on;
    %text(5,15,sprintf("distance travelled: %.2f m",spot_to_spot_distance));
    %text(5,10,sprintf("time taken: %.2f minutes",spot_to_spot_time/60));
    title(sprintf("Simulated robot's trajectory; field 6; %s waypoints.",5));
    lgd = legend(["Stops","Ideal path","Start","Actual path"],'FontSize',7);
    lgd.NumColumns = 2;
    legend('boxoff');
    hold off;
    
    
end 