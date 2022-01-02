function main()
    % basic configuration
    num = 100;
    interval = 2/num;
    l1 = 1;
    l2 = 0.5;
    % Record reference:
    % https://www.mathworks.com/matlabcentral/answers/455886-how-to-save-animated-plots
    recorder = VideoWriter('simulation.mp4'); % open video file
    recorder.FrameRate = 15;  % can adjust this, 5 - 10 works well for me
    open(recorder);
    
    n = input('Please enter n:\n');
    while (~isnumeric(n) || ~isequal(size(n), [1,1]) || n <= 1)
        fprintf("Invalid input!");
        n = input("Please enter n:\n");
    end
    hold on
    % draw outer circle
    plotCircle(l1+l2);
    % draw inner circle
    plotCircle(l1-l2);
    
    pause(interval);
    frame = getframe(gcf); % get frame
    writeVideo(recorder, frame);
    
    title(sprintf('Click on %d points', n));
%     [interpX, vq] = getInterp(n, num, l1, l2);
%     [interpX, vq] = getBezierPoints(n, l1, l2);
    [interpX, vq] = getCirclePoints(n, l1, l2);
    
    pause(interval);
    frame = getframe(gcf); % get frame
    writeVideo(recorder, frame);
    
    title(sprintf('Success!'));
    plot(interpX, vq);
    
    pause(interval);
    frame = getframe(gcf); % get frame
    writeVideo(recorder, frame);
    
    % simulate moving process
    [th1_left, th2_left, th1_right, th2_right] = solveInverse(interpX(1), vq(1), l1, l2);
    theta_buffer = [th1_right th2_right];
    [a1, a2] = drawArms(th1_right, th2_right, l1, l2);
    pause(interval);
    
    frame = getframe(gcf); %get frame
    writeVideo(recorder, frame);
    
    for i=2:size(interpX, 1)
        [th1_left, th2_left, th1_right, th2_right] = solveInverse(interpX(i), vq(i), l1, l2);
        diff_left = angdiff(th1_left, theta_buffer(1));
        diff_right = angdiff(th1_right, theta_buffer(1));
        
        set(a1, 'Visible', 'off');
        set(a2, 'Visible', 'off');
        if diff_left < diff_right
            [a1, a2] = drawArms(th1_left, th2_left, l1, l2);
            theta_buffer = [th1_left th2_left];
        else
            [a1, a2] = drawArms(th1_right, th2_right, l1, l2);
            theta_buffer = [th1_right th2_right];
        end
        pause(interval);
        
        frame = getframe(gcf); %get frame
        writeVideo(recorder, frame);
    end
    hold off
    close(recorder);
end

function h = plotCircle(r)
    % Reference:
    % https://www.mathworks.com/matlabcentral/answers/98665-how-do-i-plot-a-circle-with-a-given-radius-and-center
    th = 0:pi/50:2*pi;
    xunit = r * cos(th);
    yunit = r * sin(th);
    h = plot(xunit, yunit, 'LineWidth', 2);
    xlim([-1.5 1.5]);
    ylim([-1.5 1.5]);
    pbaspect([1 1 1]);
end

function [cx, cy] = getCirclePoints(n, l1, l2)
    interpNum = 20;
    h_l = zeros(n-1, 1);
    h_p = zeros(n, 1);
    points = zeros(n, 2);
    points(1, :) = ginput(1);
    while norm(points(1, :)) < l1-l2 || norm(points(1, :)) > l1+l2
        title(sprintf('Invalid point! It must be within the area between the two circles'));
        points(1, :) = ginput(1);
    end
    title(sprintf('Click on %d more points', n-1));
    h_p(1) = plot(points(1, 1), points(1, 2), 'r.', 'LineWidth', 2);
    for i = 2:n
        % let user select the i-th point and line between the (i-1)-th 
        % point and the i-th point
        points(i, :) = ginput(1);
        while norm(points(i, :)) < l1-l2 || norm(points(i, :)) > l1+l2
            title(sprintf('Invalid point! It must be within the area between the two circles'));
            points(i, :) = ginput(1);
        end
        if i ~= n
            title(sprintf('Click on %d more points', n-i));
        end
        h_p(i) = plot(points(i, 1), points(i, 2), 'r.');
        h_l(i-1) = plot(points(i-1:i, 1), points(i-1:i, 2), 'black', 'LineWidth', 0.5);
    end
    if n == 2
        % n=2: return bezier curve
        midNorm = (norm(points(1, :)) + norm(points(2, :)))/2;
        ts = [atan2(points(1, 2), points(1, 1)) atan2(points(2, 2), points(2, 1))];
        tm = mod((ts(1)+ts(2))/2, 2*pi);
        [bezcurve, ~] = bezier_([
            points(1, 1), points(1, 2); 
            midNorm*cos(tm) midNorm*sin(tm); 
            points(2, 1), points(2, 2)
            ], interpNum*3);
        cx = bezcurve(:, 1);
        cy = bezcurve(:, 2);
    else
        cx = zeros((interpNum-1)*(n-1)+1, 1);
        cy = zeros((interpNum-1)*(n-1)+1, 1);
        [r, cn] = circumcircle(points(1:3, :)');
        if r ~= 0
            th = atan2(points(1:3, 2)-cn(2), points(1:3, 1)-cn(1));
            diff12 = angdiff(th(2), th(1));
            diff13 = angdiff(th(3), th(1));
            dir = ~xor(diff12 < 0, diff12*diff13 < 0 || abs(diff12) < abs(diff13));
            [cx(1:interpNum, 1), cy(1:interpNum, 1)] = interpCirclePoints(interpNum, th(1:2, :), r, cn, dir);
            [cx(interpNum:2*interpNum-1, 1), cy(interpNum:2*interpNum-1, 1)] = interpCirclePoints(interpNum, th(2:3, :), r, cn, dir);
        else
            cx(1:2*interpNum-1, 1) = linspace(points(1, 1), points(3, 1), 2*interpNum-1)';
            cy(1:2*interpNum-1, 1) = linspace(points(1, 2), points(3, 2), 2*interpNum-1)';
        end
        
        for i=4:n
            [r, cn] = circumcircle(points(i-2:i, :)');
            if r ~= 0
                th = atan2(points(i-2:i, 2)-cn(2), points(i-2:i, 1)-cn(1));
                [cx(1+(i-2)*(interpNum-1):1+(i-1)*(interpNum-1), 1), cy(1+(i-2)*(interpNum-1):1+(i-1)*(interpNum-1), 1)] = interpCirclePoints(interpNum, th(2:3, :), r, cn, dir);
            else
                cx(1+(i-2)*(interpNum-1):1+(i-1)*(interpNum-1), 1) = linspace(points(i-1, 1), points(i, 1), interpNum)';
                cy(1+(i-2)*(interpNum-1):1+(i-1)*(interpNum-1), 1) = linspace(points(i-1, 2), points(i, 2), interpNum)';
            end
        end
    end
    % hide line segments between inputed points
    for i = 1:n-1
        set(h_l(i), 'Visible', 'off');
    end
end

function [ix, iy] = interpCirclePoints(n, th, r, cn, clockwise)
    if clockwise
        if th(1) < th(2)
            th(1) = th(1) + 2*pi;
        end
    else
        if th(2) < th(1)
            th(2) = th(2) + 2*pi;
        end
    end
    interpTh = linspace(th(1), th(2), n)';
    res = [r*cos(interpTh) r*sin(interpTh)] + cn';
    ix = res(:, 1);
    iy = res(:, 2);
end

function [bx, by] = getBezierPoints(n, l1, l2)
    bezier_num = 20;
    points = zeros(n, 2);
    h_l = zeros(n-1, 1);
    h_p = zeros(n, 1);
    bx = zeros(bezier_num*(n-1), 1);
    by = zeros(bezier_num*(n-1), 1);
    
    points(1, :) = ginput(1);
    while norm(points(1, :)) < l1-l2 || norm(points(1, :)) > l1+l2
        title(sprintf('Invalid point! It must be within the area between the two circles'));
        points(1, :) = ginput(1);
    end
    title(sprintf('Click on %d more points', n-1));
    h_p(1) = plot(points(1, 1), points(1, 2), 'r.', 'LineWidth', 2);
    for i = 2:n
        % let user select the i-th point and line between the (i-1)-th 
        % point and the i-th point
        points(i, :) = ginput(1);
        while norm(points(i, :)) < l1-l2 || norm(points(i, :)) > l1+l2
            title(sprintf('Invalid point! It must be within the area between the two circles'));
            points(i, :) = ginput(1);
        end
        if i ~= n
            title(sprintf('Click on %d more points', n-i));
        end
        x = [points(i-1, 1) points(i, 1)];
        v = [points(i-1, 2) points(i, 2)];
        ts = [atan2(points(i-1, 2), points(i-1, 1)) atan2(points(i, 2), points(i, 1))];
        tm = mod((ts(1)+ts(2))/2, 2*pi);
        magnitude = 1.2*min([norm(points(i-1, :)) norm(points(i, :))]);
        midNorm = (norm(points(i-1, :)) + norm(points(i, :)));
        [bezcurve, ~] = bezier_([
            points(i-1, 1), points(i-1, 2); 
            midNorm*cos(tm) midNorm*sin(tm); 
            points(i, 1), points(i, 2)
            ], bezier_num);
        bx(bezier_num*(i-2)+1:bezier_num*(i-1)) = bezcurve(:, 1);
        by(bezier_num*(i-2)+1:bezier_num*(i-1)) = bezcurve(:, 2);
        h_p(i) = plot(points(i, 1), points(i, 2), 'r.');
        h_l(i-1) = plot(x, v, 'black', 'LineWidth', 0.5);
    end
    
    % hide line segments between inputed points
    for i = 1:n-1
        set(h_l(i), 'Visible', 'off');
    end
end

function [interpX, vq] = getInterp(n, l1, l2)
    points = zeros(n, 2);
    h_l = zeros(n-1, 1);
    h_p = zeros(n, 1);
    % let user select the 1st point
    points(1, :) = ginput(1);
    while norm(points(1, :)) < l1-l2 || norm(points(1, :)) > l1+l2
        title(sprintf('Invalid point! It must be within the area between the two circles'));
        points(1, :) = ginput(1);
    end
    title(sprintf('Click on %d more points', n-1));
    h_p(1) = plot(points(1, 1), points(1, 2), 'r.', 'LineWidth', 2);
    for i = 2:n
        % let user select the i-th point and line between the (i-1)-th 
        % point and the i-th point
        points(i, :) = ginput(1);
        while norm(points(i, :)) < l1-l2 || norm(points(i, :)) > l1+l2
            title(sprintf('Invalid point! It must be within the area between the two circles'));
            points(i, :) = ginput(1);
        end
        if i ~= n
            title(sprintf('Click on %d more points', n-i));
        end
        x = [points(i-1, 1) points(i, 1)];
        v = [points(i-1, 2) points(i, 2)];
        h_p(i) = plot(points(i, 1), points(i, 2), 'r.');
        h_l(i-1) = plot(x, v, 'black', 'LineWidth', 0.5);
    end
    x = points(:, 1);
    v = points(:, 2);
    interpX = zeros((n-1)*num+1, 1);
    for i = 1:n-1
        interpX((i-1)*num+1: i*num+1) = linspace(x(i), x(i+1), num+1)';
    end
    vq = interp1(x, v, interpX, 'spline');
    
    % hide line segments between inputed points
    for i = 1:n-1
        set(h_l(i), 'Visible', 'off');
    end
end

function [th1_left, th2_left, th1_right, th2_right] = solveInverse(x, y, l1, l2)
    hypotenuse = x*x + y*y;
    alpha = real(acos((l1*l1 + l2*l2 - hypotenuse)/2/l1/l2));
    beta = real(acos((l1*l1 + hypotenuse - l2*l2)/2/l1/sqrt(hypotenuse)));
    atg = atan2(y, x);
    th1_left = mod(atg + beta, 2*pi);
    th1_right = mod(atg - beta, 2*pi);
    th2_left = mod(alpha - pi, 2*pi);
    th2_right = mod(pi - alpha, 2*pi);
end

function [a1, a2] = drawArms(th1, th2, l1, l2)
    % end point of the 1st arm
    x1 = l1 * cos(th1);
    y1 = l1 * sin(th1);
    % end point of the 2nd arm
    x2 = x1 + l2*cos(th1+th2);
    y2 = y1 + l2*sin(th1+th2);
    % plot 2 arms
    a1 = plot([0;x1], [0;y1], 'm', 'LineWidth', 2);
    a2 = plot([x1;x2], [y1;y2], 'c', 'LineWidth', 2);
end

function d = angdiff(th1, th2)
    % Simplified version of:
    % https://github.com/petercorke/spatialmath-matlab/blob/master/angdiff.m
    d = th1 - th2;
    d = mod(d+pi, 2*pi) - pi;
end