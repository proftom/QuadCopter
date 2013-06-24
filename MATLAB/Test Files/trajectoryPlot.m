tData = load('statesamples.txt');
[len,x] = size(tData);
figure;
plot3 (tData(1000:len,1), tData(1000:len,2),tData(1000:len,3));

xlabel('x(m)')
ylabel('y(m)')
zlabel('z(m)')
grid on
axis square
axis equal
set(gca,'XTick',-1.84:0.05:-1.74);