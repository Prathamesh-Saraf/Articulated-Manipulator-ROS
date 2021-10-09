close all
clear all
clc

%% Manipulator specs
global m1 m2 m3 a1 a2 a3 g
%m1 = 2+2 ; m2 = 1.6+1.6; m3 = .5*3 ;
m1 = 0.1+0.1 ; m2 = 0.01+0.01 ; m3 = .01*3 ;
%m1 = 2+3 ; m2 = 1.6+2.5; m3 = .7*3;
a1 = 34; a2 = 40; a3 = 52.6; g = 9.81;
%% Initial and final end-effector coordinates
x_1 = 30; y_1 = 0; z_1 = 25;
x_2 = 15; y_2 = 15; z_2 = 25;

%% Inverse kinematics to get corresponding joint angles
[Theta11, Theta12, Theta13] = invk(x_1,y_1,z_1);
theta11 = real(Theta11), theta12 = real(Theta12), theta13 = real(Theta13)
[Theta21, Theta22, Theta23] = invk(x_2,y_2,z_2);
theta21 = real(Theta21), theta22 = real(Theta22), theta23 = real(Theta23)

%% Set-points
theta1 = theta21; theta2 = theta22; theta3 = theta23;
theta1_dot = abs((theta21 - theta11)/2); theta2_dot = abs((theta22 - theta12)/2); theta3_dot = abs((theta23 - theta13)/2);

%% Jacobian
J = [-sin(theta1)*(a3*cos(theta2+theta3)+a2*cos(theta2)) -cos(theta1)*(a3*sin(theta2+theta3)+a2*sin(theta2)) -a3*sin(theta2+theta3)*cos(theta1);
      cos(theta1)*(a3*cos(theta2+theta3)+a2*cos(theta2)) -sin(theta1)*(a3*sin(theta2+theta3)+a2*sin(theta2)) -a3*sin(theta2+theta3)*sin(theta1);
      0 a3*cos(theta2+theta3)+a2*cos(theta2) a3*cos(theta2+theta3)];
JT = transpose(J);
J_inv = inv(J);
J_dot = [-cos(theta1)*(a3*cos(theta2+theta3)+a2*cos(theta2)) -cos(theta1)*(a3*cos(theta2+theta3)+a2*cos(theta2)) -a3*cos(theta2+theta3)*cos(theta1);
         -sin(theta1)*(a3*cos(theta2+theta3)+a2*cos(theta2)) -sin(theta1)*(a3*cos(theta2+theta3)+a2*cos(theta2)) -a3*cos(theta2+theta3)*sin(theta1);
         0 -a3*sin(theta2+theta3)-a2*sin(theta2) -a3*sin(theta2+theta3)];

%% Lagrange Euler Dynamic model
M = [((m3*(a2^2*cos(2*theta2)+a3^2*cos(2*theta2+2*theta3)+a2^2+a3^2+2*a2*a3*cos(2*theta2+theta3)+2*a2*a3*cos(theta3)))/2+m2*a2*a2*cos(theta2)^2) 0 0;
      0 (a2^2*m2+a2^2*m3+a3^2*m3 + 2*a2*a3*m3*cos(theta3)) (a3^2*m3+a2*a3*m3*cos(theta3));
      0 (m3*a3*a3+m3*a2*a3*cos(theta3)) (m3*a3*a3)]
  
V = [-theta1_dot*((a3^2*m3*sin(2*theta2 + 2*theta3)*(theta2_dot + theta3_dot)) + a2^2*m3*theta2_dot*sin(2*theta2) + a2*a3*m3*sin(2*theta2 + theta3)*(2*theta2_dot + theta3_dot) + 2*a2^2*m2*theta2_dot*cos(theta2)*sin(theta2) + a2*a3*m3*theta3_dot*sin(theta3));
     (a2^2*m2*theta1_dot^2*sin(2*theta2))/2 + (a2^2*m3*theta1_dot^2*sin(2*theta2))/2 + (a3^2*m3*theta1_dot^2*sin(2*theta2 + 2*theta3))/2 - a2*a3*m3*theta3_dot^2*sin(theta3) + a2*a3*m3*theta1_dot^2*sin(2*theta2 + theta3)- 2*a2*a3*m3*theta2_dot*theta3_dot*sin(theta3);
      0.5*m3*a3*a3*theta1_dot^2*sin(2*theta2 + 2*theta3) + 0.5*m3*a3*a2*theta1_dot^2*sin(theta3) + m3*a3*a2*theta2_dot^2*sin(theta3) + 0.5*m3*a3*a2*theta1_dot^2*sin(2*theta2 + theta3) + m3*a2*a3*theta2_dot*theta3_dot*sin(theta3)]

V1 = [0 0.5*(-theta1_dot*(a3^2*m3*sin(2*theta2 + 2*theta3)+a2^2*m3*sin(2*theta2)+2*a2*a3*m3*sin(2*theta2 + theta3)+2*a2^2*m2*cos(theta2)*sin(theta2))) 0.5*(-theta1_dot*(a3^2*m3*sin(2*theta2 + 2*theta3) + a2*a3*m3*sin(2*theta2 + theta3) + a2*a3*m3*sin(theta3)));
      ((a2^2*m2*theta1_dot*sin(2*theta2))/2 + (a2^2*m3*theta1_dot*sin(2*theta2))/2 + (a3^2*m3*theta1_dot*sin(2*theta2 + 2*theta3))/2 + a2*a3*m3*theta1_dot*sin(2*theta2 + theta3)) 0 (-a2*a3*m3*theta3_dot*sin(theta3)-a2*a3*m3*theta2_dot*sin(theta3));
      (0.5*m3*a3*a3*theta1_dot*sin(2*theta2 + 2*theta3) + 0.5*m3*a3*a2*theta1_dot*sin(theta3) + 0.5*m3*a3*a2*theta1_dot*sin(2*theta2 + theta3))  (m3*a2*a3*theta3_dot*sin(theta3)+ m3*a3*a2*theta2_dot*sin(theta3)) 0];

G = [g*m3*cos(theta1)*(a3*cos(theta2 + theta3) + a2*cos(theta2)) + a2*g*m2*cos(theta1)*cos(theta2);
        -g*m3*sin(theta1)*(a3*sin(theta2 + theta3) + a2*sin(theta2)) - a2*g*m2*sin(theta1)*sin(theta2);
        -a3*g*m3*sin(theta2 + theta3)*sin(theta1)];

%% State-space model
A =[0 0 0 1 0 0; 
    0 0 0 0 1 0;  
    0 0 0 0 0 1;   
    0 0 0 (2*(a2^2*m2*theta2_dot*sin(2*theta2) + a2^2*m3*theta2_dot*sin(2*theta2) + a3^2*m3*sin(2*theta2 + 2*theta3)*(theta2_dot + theta3_dot) + a2*a3*m3*sin(2*theta2 + theta3)*(2*theta2_dot + theta3_dot) + a2*a3*m3*theta3_dot*sin(theta3)))/(a2^2*m3 + a3^2*m3 + a2^2*m3*cos(2*theta2) + 2*a2^2*m2*cos(theta2)^2 + a3^2*m3*cos(2*theta2 + 2*theta3) + 2*a2*a3*m3*cos(theta3) + 2*a2*a3*m3*cos(2*theta2 + theta3)) 0 0;
    0 0 0 (theta1_dot*(2*a3*m3*sin(theta3) - a3*m3*sin(2*theta2 + theta3) - 2*a2*m2*sin(2*theta2) - a2*m3*sin(2*theta2) + a2*m3*sin(2*theta3) + a2*m3*sin(2*theta2 + 2*theta3) + a3*m3*sin(2*theta2 + 3*theta3)))/(2*a2*(2*m2 + m3 - m3*cos(2*theta3))) (m3*sin(theta3)*(a3*theta2_dot + 2*a3*theta3_dot + a2*theta2_dot*cos(theta3)))/(a2*(- m3*cos(theta3)^2 + m2 + m3)) (m3*sin(theta3)*(a3*theta2_dot + a3*theta3_dot + a2*theta2_dot*cos(theta3)))/(a2*(- m3*cos(theta3)^2 + m2 + m3));
    0 0 0 (theta1_dot*(a3 + a2*cos(theta3))*(a3^2*m3*sin(2*theta2 + 2*theta3) + a2^2*m2*sin(2*theta2) + a2^2*m3*sin(2*theta2) + 2*a2*a3*m3*sin(2*theta2 + theta3)))/(2*a2^2*a3*(- m3*cos(theta3)^2 + m2 + m3)) - (theta1_dot*(a3*sin(2*theta2 + 2*theta3) + a2*sin(theta3) + a2*sin(2*theta2 + theta3))*(a2^2*m2 + a2^2*m3 + a3^2*m3 + 2*a2*a3*m3*cos(theta3)))/(2*a2^2*a3*(- m3*cos(theta3)^2 + m2 + m3)) (-(sin(theta3)*(a2^2*m2*theta2_dot + a2^2*m3*theta2_dot + a3^2*m3*theta2_dot + 2*a3^2*m3*theta3_dot + 2*a2*a3*m3*theta2_dot*cos(theta3) + 2*a2*a3*m3*theta3_dot*cos(theta3)))/(a2*a3*(- m3*cos(theta3)^2 + m2 + m3))) (-(sin(theta3)*(a2^2*m2*theta2_dot + a2^2*m3*theta2_dot + a3^2*m3*theta2_dot + a3^2*m3*theta3_dot + 2*a2*a3*m3*theta2_dot*cos(theta3) + a2*a3*m3*theta3_dot*cos(theta3)))/(a2*a3*(- m3*cos(theta3)^2 + m2 + m3)))]
  
B = [0 0 0;
     0 0 0;
     0 0 0;     
     2/(a2^2*m3 + a3^2*m3 + a2^2*m3*cos(2*theta2) + 2*a2^2*m2*cos(theta2)^2 + a3^2*m3*cos(2*theta2 + 2*theta3) + 2*a2*a3*m3*cos(theta3) + 2*a2*a3*m3*cos(2*theta2 + theta3)) 0 0;
     0 1/(a2^2*m2 + a2^2*m3 - a2^2*m3*cos(theta3)^2) (-(a3 + a2*cos(theta3))/(a2^2*a3*m2 + a2^2*a3*m3 - a2^2*a3*m3*cos(theta3)^2));
     0 (-(a3 + a2*cos(theta3))/(a2^2*a3*m2 + a2^2*a3*m3 - a2^2*a3*m3*cos(theta3)^2)) (a2^2*m2 + a2^2*m3 + a3^2*m3 + 2*a2*a3*m3*cos(theta3))/(- a2^2*a3^2*m3^2*cos(theta3)^2 + a2^2*a3^2*m3^2 + m2*a2^2*a3^2*m3)]


q= [500 500 500 .01 .01 .01]
Q=diag(q)
r=[100 100 100];
R=diag(r)
N=0;
[K,S,e] = lqr(A,B,Q,R,N) 
%% Inverse Kinematics
function [theta1, theta2, theta3] = invk(x,y,z)

global a1 a2 a3

r1 = sqrt(x^2 + y^2);
r2 = a1 - z;
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
    theta1 = 0;
else
    theta1 = atan(y/x);
end

theta2 = phi2 - phi1;
theta3 = - pi + phi3;
end

