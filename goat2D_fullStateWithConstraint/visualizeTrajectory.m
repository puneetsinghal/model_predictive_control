function visualizeTrajectory(q,l)
%     Input txtheta
    [tsteps,~] = size(q);
    h1 = plot(0,0);
    hold on;
    h2 = plot(0,0);
    h3 = plot(0,0);
    h4 = plot(0,0);
    h5 = plot(0,0);
    h6 = plot(0,0);
    hold off;
    waitforbuttonpress;
    for i=1:1:tsteps
        x1 = zeros(4,2);
        x2 = zeros(4,2);
        x1(2,1) = l(1)*cos(q(i,1));
        x1(3,1) = l(1)*cos(q(i,1)) + l(2)*cos(q(i,1)+q(i,2));
        x1(4,1) = l(1)*cos(q(i,1)) + l(2)*cos(q(i,1)+q(i,2)) + l(3)*cos(q(i,1)+q(i,2)+q(i,3));
        x1(2,2) = l(1)*sin(q(i,1));
        x1(3,2) = l(1)*sin(q(i,1)) + l(2)*sin(q(i,1)+q(i,2));
        x1(4,2) = l(1)*sin(q(i,1)) + l(2)*sin(q(i,1)+q(i,2)) + l(3)*sin(q(i,1)+q(i,2)+q(i,3));
        
        x2(2,1) = l(1)*cos(q(i,4));
        x2(3,1) = l(1)*cos(q(i,4)) + l(2)*cos(q(i,4)+q(i,5));
        x2(4,1) = l(1)*cos(q(i,4)) + l(2)*cos(q(i,4)+q(i,5)) + l(3)*cos(q(i,4)+q(i,5)+q(i,6));
        x2(2,2) = l(1)*sin(q(i,4));
        x2(3,2) = l(1)*sin(q(i,4)) + l(2)*sin(q(i,4)+q(i,5));
        x2(4,2) = l(1)*sin(q(i,4)) + l(2)*sin(q(i,4)+q(i,5)) + l(3)*sin(q(i,4)+q(i,5)+q(i,6));
        
        pause(0.1);
        delete(h1);delete(h2);delete(h3);delete(h4);delete(h5);delete(h6);
        axes;
%         axis image;
        axis([-sum(l)*1.2, sum(l)*1,-sum(l)*1.2, sum(l)*1]);
        hold on;
        h1 = plot(x1(1:3,1),x1(1:3,2),'Color', [0 0 0.7],'LineWidth', 5);
        h2 = plot(x1(3:4,1),x1(3:4,2),'Color', [0 0.7 0],'LineWidth', 5);
        h3 = plot(x1(:,1),x1(:,2),'.', 'MarkerSize', 20, 'Color', [1 0 0]);
        h4 = plot(x2(1:3,1),x2(1:3,2),'Color', [0 0 0.7],'LineWidth', 5);
        h5 = plot(x2(3:4,1),x2(3:4,2),'Color', [0 0.7 0],'LineWidth', 5);
        h6 = plot(x2(:,1),x2(:,2),'.', 'MarkerSize', 20, 'Color', [1 0 0]);
        hold off;
    end
    
end