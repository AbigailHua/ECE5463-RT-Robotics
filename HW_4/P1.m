%% Problem 1
%% Solve the differential equation
% The second parameter specifies the range of t and the third parameter
% specifies the initial value of x' and x''.
[t, x] = ode45(@myDiffEqu, [0 5], [1; -1]);

%% Plot
plot(t,x(:,1),'-o',t,x(:,2),'-o')
title('Solution of the Second Order Differential Equation with ODE45');
xlabel('Time t');
ylabel('Solution');
legend('$x$', '$\dot{x}$', 'Interpreter','latex')

%% Function
function dxdt = myDiffEqu(t, x)
    m = 1;
    c = 8;
    k = 4;
    dxdt = [x(2); -(c*x(2)+k*x(1))/m];
end

%% Reference
% 1. ode45 Documentation from Mathworks: https://www.mathworks.com/help/matlab/ref/ode45.html?lang=en#d123e975456
%
% 2. How to use LaTex in legend: https://www.mathworks.com/matlabcentral/answers/21984-tex-latex-math-mode-symbols-in-legends-and-labels-in-matlab-figures