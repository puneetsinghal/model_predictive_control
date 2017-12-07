load('Acrobot_It1.mat','-mat','xHist')
xtraj = xHist;
load('Acrobot_It2.mat','-mat','xHist','params')
xtraj = [xtraj xHist];
ts = params.Ts;
for i = 1:40
    drawAcrobot(i*ts, xtraj(:,i), params);
%     if (i==1)
%         pause(5);
%     end
    pause(0.01);
end