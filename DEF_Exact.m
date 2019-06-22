%Deflection Exact
function [x_] = DEF_Exact(E,I,T1,T2,m,l)
y_1 = (1/E*I)*(T1*((l^(2))/2) - T2*(l^(2)/2) -(m/2)*l^(2));
y_2 = (1/E*I)*(- T2*(l^(2)/2) -(m/4)*l^(2));
x_ = (y_1+y_2)*1000;
