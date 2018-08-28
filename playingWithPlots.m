 R = [1     0     0;
         0     0    -1;
         0     1     0];
R = eye(3);
figure(3)
cam = plotCamera('Location',[10 0 20],'Orientation',R,'Opacity',0);
grid on
axis equal
axis manual
xlabel('X (mm)');
ylabel('Y (mm)');
zlabel('Z (mm)');

xlim([-15,20]);
ylim([-15,20]);
zlim([15,25]);