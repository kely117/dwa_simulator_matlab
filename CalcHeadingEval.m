function heading=CalcHeadingEval(x,goal)
%A function that calculates the evaluation function of heading

theta=toDegree(x(3));%Robot orientation
if theta < 0
    theta = theta + 360;
end

goalTheta=toDegree(atan2(goal(2)-x(2),goal(1)-x(1)));%Goal direction

if goalTheta<0
    goalTheta = goalTheta + 360;
end
targetTheta = goalTheta - theta ;
heading = abs(min(targetTheta, 360 - targetTheta));