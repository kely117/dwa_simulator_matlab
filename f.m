function x = f(x, u)
% Motion Model
global dt;
 
% F = [1 0 0 0 0
    % 0 1 0 0 0
     %0 0 1 0 0
     %0 0 0 0 0
     %0 0 0 0 0];
 
% B = [dt*cos(x(3)) 0
    %dt*sin(x(3)) 0
    %0 dt
    %1 0
    %0 1];

% x= F*x+B*u;
x(3) = x(3) + u(2)*dt;
x(1) =x(1) + u(1)*cos(x(3))*dt;
x(2) = x(2) + u(1)*sin(x(3))*dt;
x(4) = u(1);
x(5) = u(2);