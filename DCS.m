%Dynamics of 2 DoF Robotic Manipulator
function [T] = DCS(Th_1_i,Th_1_f,Th_2_i,Th_2_f,m1,m2,l_1,l_2,g,dt_) %DCS(1,1.1,1.2,1.3,.5,.5,.15,.15,9.81,0.2);

%Calculating Angular postn velocity and accn of link
Th_1 = Th_1_f;Th_2 = Th_2_f;
Th_1_dot = (Th_1_f - Th_1_i)/dt_;
Th_1_dot2 = (Th_1_f - Th_1_i)^(2)/(dt_^(2));
Th_2_dot = (Th_2_f - Th_2_i)/dt_;
Th_2_dot2 = (Th_2_f - Th_2_i)^(2)/(dt_^(2));

%Defining Dynamic Matrices for Torque Calculation
syms T_1  T_2;
T = [T_1 ; T_2];
A = [((1/3)*m1*l_1.^(2) + m2*l_1.^(2) + (1/3)*m2*l_2.^(2) + m2*l_1*l_2*cos(Th_2)) ((1/3)*m2*l_2.^(2) + (1/2)*m2*l_1*l_2*cos(Th_2));((1/3)*m2*l_2.^(2)+(1/2)*m2*l_1*l_2*cos(Th_2)) (1/3)*m2*l_2.^(2);];
B = [0 (-(1/2)*m2*l_1*l_2*sin(Th_2)); ((1/2)*m1*l_1*l_2*sin(Th_2)) 0;];
C = [-(m2*l_1*l_2*sin(Th_2)) 0; 0 0;];
D = [((1/2)*m1+m2)*g*l_1*cos(Th_1)+(1/2)*m2*g*l_2*cos(Th_1+Th_2);( (1/2)*m2*g*l_2*cos(Th_1 + Th_2))];

A_ = [Th_1_dot2; Th_2_dot2];
B_ = [Th_1_dot.^(2); Th_2_dot.^(2);];
C_ = [Th_1_dot*Th_2_dot;Th_2_dot*Th_1_dot];

%Calculating Torque

T = A*A_+B*B_+C*C_+D;







