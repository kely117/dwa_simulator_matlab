function cost=CalcPathEval(xt,x,goal)
%A function that calculates the evaluation function of heading
x1 = x(1);
y1 = x(2);
x2 = goal(1);
y2 = goal(2);
m = (y2 - y1)/(x2 - x1);
A = m ;
B = -1 ;
C = y1 - m*x1 ;
cost = abs(A*xt(1) + B*xt(2) + C)/sqrt(A^2 + B^2);

