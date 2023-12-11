function heading=CalcDistanceEval(x,goal)
%A function that calculates the evaluation function of heading

heading = norm(x(1:2) - goal(1:2));