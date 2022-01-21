close all
clear 
clc

x_1 = 0;
y_1 = 0;
x_2 = 2;
y_2 = 0;

for i=1:10
    [theta11, theta12] = invk(x_1,y_1);
    pause(1)
    [theta21, theta22] = invk(x_2,y_2);
    pause(1)
end
    
function [theta1, theta2] = invk(x,y)

a1 = 1; 
a2 = 1;

d = sqrt(x^2 + y^2);
phi2 = acosd((d^2 - a1^2 - a2^2)/(-2*a1*a2));
theta2 = 180 - phi2

phi1 = acosd((a2^2 - a1^2 - d^2)/(-2*a1*d));

if(x==0)
    theta1 = 90 - phi1
elseif(y<0 && x<0)
    theta1 = -180 + atand(y/x) - phi1
else
    theta1 = atand(y/x) - phi1
end

x1 = 0;
y1 = 0;

x2 = a1*cosd(theta1);
y2 = a1*sind(theta1);

x3 = a2*cosd(theta1+theta2) + x2;
y3 = a2*sind(theta1+theta2) + y2;

L1=sqrt((y2-y1)^2+(x2-x1)^2)
L2=sqrt((y3-y2)^2+(x3-x2)^2)

plot([x1 x2],[y1 y2],[x2 x3],[y2 y3],'linewidth', 2)
axis([-6 6 -6 6])
xlabel('x');
ylabel('y');
end
