%Kinematics and Dynamics Plot of Robotic Arm
function [IK] = IK_plot(x0 , y0, Th_1 ,Th_2)

%Plotting The workspace 
L (1)  = Link( [Th_1 0 150 0], 'R');
L (2)  = Link( [Th_2 0 150 0], 'R' ); 
figure(5)
view(3)
view(0,90)
grid on;
%axis([-1,1,-1,1,-1,1])
Robot = SerialLink(L);
Robot.name = '2 - DoF FK/IK Kinematics';
%Robot.plot([Th_1 Th_2]);

T = transl(x0,y0,0);
disp(T)
IK = [Th_1 Th_2];
%Inverse Kinamatics
%if x0 <= y0
   theta_2 = acos((x0^2+y0^2-2*150^2)/(2*150*150)); 
   theta_1 = atan2(y0,x0) - atan2(150*sin(theta_2),(150 +150*cos(theta_2)));
   
%else
  % theta_2 = -acos((x0^2+y0^2-2*150^2)/(2*150*150));
   %theta_1 = atan2(y0,x0) + atan2(150*sin(theta_2),(150 +150*cos(theta_2)));   
%end
Th_1 = theta_1;Th_2 = theta_2;
IK = [Th_1 Th_2];
%Robot.plot(IK)
