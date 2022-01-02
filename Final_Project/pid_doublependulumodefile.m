function dstatevar = pid_doublependulumodefile(t,statevar,param)
    %DOUBLEPENDULUMODEFILE This is the ODE file for a double pendulum with
    %massless links, and point-masses at the end of each link (see text book for 
    % the equations).

    m1 = param.m1; m2 = param.m2; 
    L1 = param.L1; L2 = param.L2; g = param.g;
    I1 = param.I1; I2 = param.I2;
    r1 = param.L1; r2 = param.L2;

    % t

    theta1 = statevar(1); 
    dtheta1 = statevar(2); 
    theta2 = statevar(3); 
    dtheta2 = statevar(4);
    interror1 = statevar(5);
    interror2 = statevar(6);

%     PID controller
    kp = 100; kv = 80; ki = 0.1;
    tau1 = -kp*(theta1-param.td1)-ki*interror1-kv*dtheta1;
    tau2 = -kp*(theta2-param.td2)-ki*interror2-kv*dtheta2;

    % if we wanted, we can add springs and dampers ...
    % c = 0.1; k = 10;
    % tau1 = -c*dtheta1-k*theta1;
    % tau2 = -c*dtheta2-k*theta2;

    E = zeros(2,1); % assuming tau1 and tau2 = 0
    E(1) = L1*m2*r2*sin(theta2)*dtheta1^2 - tau2 + g*m2*r2*cos(theta1 + theta2);
    E(2) = -L1*m2*r2*sin(theta2)*dtheta2^2-2*L1*dtheta1*m2*r2*sin(theta2)*dtheta2 - tau1 + g*m2*r2*cos(theta1 + theta2) + L1*g*m2*cos(theta1) + g*m1*r1*cos(theta1);

    M = zeros(2,2);
    M(1,1) = -m2*r2^2-L1*m2*cos(theta2)*r2-I2; 
    M(1,2) = -m2*r2^2-I2;
    M(2,1) = -m2*L1^2-2*m2*cos(theta2)*L1*r2-m1*r1^2-m2*r2^2-I1-I2;
    M(2,2) = -m2*r2^2-L1*m2*cos(theta2)*r2-I2;

    C = M\E; % or equivalently inv(M)*E. M\E is better
    % C = inv(M)*E;

    ddtheta1 = C(1);
    ddtheta2 = C(2);
    
    dinterror1 = theta1-param.td1;
    dinterror2 = theta2-param.td2;
    dstatevar = [dtheta1; ddtheta1; dtheta2; ddtheta2; dinterror1; dinterror2];

end