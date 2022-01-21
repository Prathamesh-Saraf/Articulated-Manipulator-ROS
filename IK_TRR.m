close all
clear all
clc
%G0 X100 Y50 Z100
% x_1 = str2double(input('x1: ','s'));
% y_1 = str2double(input('y1: ','s'));
% z_1 = str2double(input('z1: ','s'));
% x_2 = str2double(input('x2: ','s'));
% y_2 = str2double(input('y2: ','s'));
% z_2 = str2double(input('z2: ','s'));
x_31 = 15; y_31 = 0; z_31 = 0;
x_32 = 15; y_32 = 0; z_32 = 15;
% x_11 = 10.5; y_11 = 10.5; z_11 = 0;
% x_12 = 10.5; y_12 = 10.5; z_12 = 5;
for i=1:5
   [theta11, theta12, theta13] = invk(x_31,y_31,z_31);
    pause(1)
   [theta21, theta22, theta23] = invk(x_32,y_32,z_32);
    pause(1)
end
    
function [theta1, theta2, theta3] = invk(x,y,z)

a1 = 12;
a2 = 12; 
a3 = 12;

r1 = sqrt(x^2 + y^2);
r2 = abs(z - a1);
if(r1== 0 && r2>=0)
    phi2 = pi/2;
elseif(r1== 0 && r2<0)
    phi2 = -pi/2;
else
    phi2 = atan(r2/r1);
end

r3 = sqrt(r1^2 + r2^2);
if(r3== 0)
    phi1 = 0;
else
    phi1 = acos((a3^2 - a2^2 - r3^2)/(-2*a2*r3));
end

phi3 = acos((r3^2 - a2^2 - a3^2)/(-2*a2*a3));

if(y== 0)
    theta1 = 0
else
    theta1 = atan(y/x)
end

theta2 = -(phi2 - phi1)
theta3 = -(pi - phi3)

x0 = 0;
y0 = 0;
z0 = 0;

x1 = 0;
y1 = 0;
z1 = a1;

x2 = a2*cos(theta1)*cos(theta2);
y2 = a2*sin(theta1)*cos(theta2) +y1;
z2 = a2*sin(theta2) + z1;

x3 = a3*cos(theta1)*cos(theta2+theta3) + x2;
y3 = a3*sin(theta1)*cos(theta2+theta3) + y2;
z3 = a3*sin(theta2+theta3) + z2;

L1=sqrt((y1-y0)^2+(z1-z0)^2+(x1-x0)^2)
L2=sqrt((y2-y1)^2+(z2-z1)^2+(x2-x1)^2)
L3=sqrt((y3-y2)^2+(z3-z2)^2+(x3-x2)^2)

plot3([x0 x1],[y0 y1],[z0 z1],[x1 x2],[y1 y2],[z1 z2],[x2 x3],[y2 y3],[z2 z3],'linewidth', 2)
axis([-40 40 -40 40 -0.1 40])
xlabel('x');
ylabel('y');
zlabel('z');
end



