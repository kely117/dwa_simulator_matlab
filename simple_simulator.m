function [result]=simple_simulator(waypoints,robot,model)
result = [];
    result.time = [] ;
    result.distance = [];
    result.angles = [];
    linear_speed = model(1);
    angular_speed = model(2);
    sorted_waypoints = GoalSequence(waypoints,robot);
    start = robot;
    for io=1:length(sorted_waypoints)
        goal = sorted_waypoints(io,:);
        linear_distance =  ((start(1) - goal(1))^2 + (start(2) - goal(2))^2)^0.5 ;
        target_angle = atan2d(goal(2)-start(2),goal(1) - start(1));
        angular_distance = target_angle - start(3);
        total_time = linear_distance/linear_speed + angular_distance/angular_speed;
        result.time=[result.time;total_time];
        result.distance=[result.distance;linear_distance];
        start(3)= target_angle;
        start(1) = goal(1);
        start(2) = goal(2);
        result.angles = [result.angles;target_angle];
    end
    
    result.waypoints = sorted_waypoints;
end




