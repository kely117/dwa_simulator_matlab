function [sortedWaypoints]=GoalSequence(waypoints,robot_start)
% Function to create trajectory data
% Calculate distances from startLocation to all waypoints
    len = length(waypoints);
    startLocation = robot_start;
    distances = sqrt((waypoints(:,1) - ones(len,1).*startLocation(1)).^2 + (waypoints(:,2) - ones(len,1).*startLocation(2)).^2);
    
    % Sort waypoints based on distances
    [~, sortedIndices] = sort(distances);
    
    % Determine the index of the start location in the sorted order
    startLocationIndex = sortedIndices(1);
    
    % Reorder waypoints based on the TSP solution
    sortedWaypoints = waypoints([startLocationIndex:end, 1:startLocationIndex-1], :);
    sortedWaypoints = [sortedWaypoints; robot_start(1:2)];
end