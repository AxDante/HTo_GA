X  = [2 2 5 6 10 8 1];
Y = [8 6 5 3 2 2 1];
Z = [1 3 1 4 1 8 7];

a = [1:7]'; b = num2str(a); c = cellstr(b);
dx = 0.2; dy = 0.2; dz = 0.2; % displacement so the text does not overlay the data points

scatter3(X,Y,Z)
xlabel('Cost')
ylabel('Smooth')
zlabel('Safe')
text(X+dx, Y+dy,Z+dz, c);