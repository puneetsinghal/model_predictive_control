function plotDC (TS,optimal_parameters,xf)
[numP,N]= size(optimal_parameters);
t = linspace(0,TS*N,N);
for i=1:numP-2
    subplot(2,(numP-2)/2,i);
    plot(t, xf(i)*ones(1,N), 'k--');
    hold on;
    plot(t,optimal_parameters(i,:),'b');
%     hold off;
end
figure();
hold on
plot(t, optimal_parameters(numP-1,:));
plot(t, optimal_parameters(numP,:));
end