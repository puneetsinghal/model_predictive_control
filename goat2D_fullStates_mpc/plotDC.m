function plotDC (TS,optimal_parameters,xf)
 [numP,N]= size(optimal_parameters);
t = linspace(0,TS*N,N);
figure();
for i=1:numP-2
    subplot(2,(numP-2)/2,i);
    hold on;
    plot(t, xf(1)*ones(1,N), 'k--');
    plot(t,optimal_parameters(i,:),'b');
    hold off;
end
figure();
hold on
plot(t, optimal_parameters(numP-1,:));
plot(t, optimal_parameters(numP,:));
end