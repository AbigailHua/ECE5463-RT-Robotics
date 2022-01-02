function pid_main()
    clear
    clc
    % basic settings
    param.g = 9.81; param.n = 5;
    param.m1 = 0.2; param.m2 = 0.2;
    param.L1 = 0.25; param.L2 = 0.25;
    param.r1 = param.L1; param.r2 = param.L2;
    param.I1 = 1/12; param.I2 = 1/12;
    
    % initial conditions    
    Numtimelist = 1000;
    tspan = linspace(0,20,Numtimelist)'; % simulation duration
    statevar0 = [-pi/4; 0; -pi/4; 0; 0; 0]; % initial conditions for theta1 theta1dot, theta2, theta2dot
    options = odeset('reltol',1e-9,'abstol',1e-9); % specifies accuracy of simulation
    
    % program start
    hold on
    % draw workspace
    drawWorkSpace(param.L1, param.L2);
    
    A = [param.L1*cos(statevar0(1)) param.L1*sin(statevar0(1))];
    B = [param.L1*cos(statevar0(1))+param.L2*cos(statevar0(1)+statevar0(3)) param.L1*sin(statevar0(1))+param.L2*sin(statevar0(1)+statevar0(3))];
    plot(0,0,'ko'); hold on;
    hOA = plot([0 A(1)],[0 A(2)],'r', 'LineWidth',2);
    hold on
    hAB = plot([A(1) B(1)], [A(2) B(2)], 'b', 'LineWidth',2);
    for i=1:param.n
        figure(1);
        point = ginput(1);
        while norm(point) < abs(param.L1-param.L2) || norm(point) > param.L1+param.L2
            title(sprintf('Invalid point! It must be within the workspace!'));
            point = ginput(1);
        end
        plot(point(1), point(2), 'r*', 'LineWidth', 1);
        [param.td1, param.td2] = solveInverse(point, param.L1, param.L2);
        statevar0(5) = statevar0(1) - param.td1;
        statevar0(6) = statevar0(2) - param.td2;
        
        [tlist1,statevarlist1] = ode45(@pid_doublependulumodefile,tspan,statevar0,options,param);
        theta1list = statevarlist1(:,1);
        dtheta1list = statevarlist1(:,2);
        theta2list = statevarlist1(:,3);
        dtheta2list = statevarlist1(:,4);
        
        figure(1);
        L1 = param.L1; L2 = param.L2;
        plot(0,0,'ko');
        
        for j = 1:1:Numtimelist
            xA = L1*cos(theta1list(j));
            yA = L1*sin(theta1list(j));
            xB = xA + L2*cos(theta1list(j)+theta2list(j));
            yB = yA + L2*sin(theta1list(j)+theta2list(j));
            if mod(j, 100) == 0 && (xB-point(1))^2 + (yB-point(2))^2 < 0.0001
                break
            end
            set(hOA,'xdata',[0 xA],'ydata',[0 yA],'LineWidth',2);
            set(hAB,'xdata',[xA xB],'ydata',[yA yB],'LineWidth',2);
            pause(0.01);
        end
        theta1list(j)
        theta2list(j)
        
        figure(2);
        subplot(221);
        plot(tlist1,theta1list);
        ylabel('$\theta_1$','interpreter','latex', 'FontWeight','bold');
        xlabel('t'); hold on;
        subplot(222);
        plot(tlist1,theta2list);
        ylabel('$\theta_2$','interpreter','latex', 'FontWeight','bold');
        xlabel('t'); hold on;
        subplot(223);
        plot(tlist1,dtheta1list);
        ylabel('$\dot{\theta_1}$','interpreter','latex', 'FontWeight','bold');
        xlabel('t'); hold on;
        subplot(224);
        plot(tlist1,dtheta2list);
        ylabel('$\dot{\theta_2}$','interpreter','latex', 'FontWeight','bold');
        xlabel('t'); hold on;
        
        statevar0 = [param.td1; 0; param.td2; 0; 0; 0];
    end
    
    hold off
end

function drawCircle(r)
    th = 0:pi/50:2*pi;
    xunit = r * cos(th);
    yunit = r * sin(th);
    plot(xunit, yunit, 'LineWidth', 2);
end

function drawWorkSpace(L1, L2)
    bound = L1+L2;
    % outer circle
    drawCircle(bound);
    % inner circle
    drawCircle(abs(L1-L2));
    xlim([-bound bound]);
    ylim([-bound bound]);
    pbaspect([1 1 1]);
end

function [th1_right, th2_right] = solveInverse(point, l1, l2)
    x = point(1); y = point(2);
    hypotenuse = x*x + y*y;
    alpha = real(acos((l1*l1 + l2*l2 - hypotenuse)/2/l1/l2));
    beta = real(acos((l1*l1 + hypotenuse - l2*l2)/2/l1/sqrt(hypotenuse)));
    atg = atan2(y, x);
    th1_right = mod(atg - beta, 2*pi);
    th2_right = mod(pi - alpha, 2*pi);
end