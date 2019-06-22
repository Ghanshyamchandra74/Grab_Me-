clear,clc
 %Assigning Position Values
 Th_2 = pi/2;
 Th_1 = pi/2;
 Th_1_i = pi/2;
 Th_2_i = pi/2;
 Th_1_i_ = pi/2;
 Th_2_i_ = pi/2;
x_ = 0;
y_ = 0;
fps = 5;
dt_ = 1/fps;
t_ = 0;
rho = 7860;l=0.15;
% compute the background image
Imzero = zeros(240,320,3);
for i = 1:5
Im{i} = double(imread(['DATA/',int2str(i),'.jpg']));
Imzero = Im{i}+Imzero;
end

Imback = Imzero/5;
[MR,MC,Dim] = size(Imback);

j = linspace(1,55,12)';k= linspace(5,60,12)';Imzero_ = zeros(240,320,3);
for i = 1:12
    for i_ = j(i):k(i)
     Im{i} = double(imread(['DATA/',int2str(i),'.jpg']));
     Imzero_ = Im{i}+Imzero_;
    end
end

% Kalman filter initialization
R=[[0.2845,0.0045]',[0.0045,0.0455]'];
H=[[1,0]',[0,1]',[0,0]',[0,0]'];
Q=0.01*eye(4);
P = 100*eye(4);
dt=1;
A=[[1,0,0,0]',[0,1,0,0]',[dt,0,1,0]',[0,dt,0,1]'];
g = 6; % pixels^2/time step
Bu = [0,0,0,g]';
kfinit=0;
x=zeros(100,4);

% loop over all images
for i = 1 : 60
  % load image
  Im = (imread(['DATA/',int2str(i), '.jpg'])); 
  figure(1)
  imshow(Im);
  imshow(Im)
  figure(5)
  imshow(Im)
  imshow(Im)
  Imwork = double(Im);

  %extract ball
  [cc(i),cr(i),radius,flag] = extractball(Imwork,Imback,i);
  % IK Plot of Centroid of Ball 
      x0 = cc(1,i);
      y0 = cr(1,i);
      
      %PID Controller
      u = (x0 - x_)/dt_; v = (y0 - y_)/dt_;
      
      Th_ = IK_plot(x0, y0, Th_1, Th_2);
      Th_1 = Th_(1,1);
      Th_2 = Th_(1,2);
      T = DCS(Th_1_i,Th_1,Th_2_i,Th_2,(pi/4)*(0.005^(2))*l*rho,(pi/4)*(0.005^(2))*l*rho,l,l,9.81,dt_);
      T_save{i} = T;
      x_ = DEF_Exact(210E+09,(pi/64)*(0.005^(4)),T(1,1),T(2,1),(pi/4)*(0.005^(2))*l*rho,l);
      Th__ = IK_plot((x0+x_), y0, Th_1, Th_2);
      Th_1_ = Th__(1,1);
      Th_2_ = Th__(1,2);
      T_ = DCS(Th_1_i_,Th_1_,Th_2_i_,Th_2_,(pi/4)*(0.005^(2))*l*rho,(pi/4)*(0.005^(2))*l*rho,l,l,9.81,dt_);
      T__save{i} = T_;
      Th_1_i = Th_(1,1);
      Th_2_i = Th_(1,2);
      Th_1_i_ = Th__(1,1);
      Th_2_i_ = Th__(1,2);
      figure(6) %Calculated Required Torque
      plot(t_,T(1,1),'g*');hold on;
      plot(t_,T(2,1),'b*');hold on;
      title('Calculated Required Torque');
      xlabel('Time');
      ylabel('Torque');
      figure(7) %Actual Torque Required
      plot(t_,T_(1,1),'g*');hold on;
      plot(t_,T_(2,1),'b*');hold on;
      title('Actual Required Torque');
      xlabel('Time');
      ylabel('Torque');
      figure(8) %Tip Deflection
      plot(t_,x_,'r*');hold on;
      title('Tip Deflection');
      xlabel('Time');
      ylabel('Tip Deflection (mm)');
      t_= t_+dt_;
  if flag==0
    continue
  end

  hold on
    for c = -1*radius: radius/20 : 1*radius
      r = sqrt(radius^2-c^2);
      figure(1)
      hold on;
      plot(cc(i)+c,cr(i)+r,'g.');
      hold on;
      plot(cc(i)+c,cr(i)-r,'g.'); 
      figure(5)
      plot(cc(i)+c,cr(i)+r,'g.')
      plot(cc(i)+c,cr(i)-r,'g.')
    end
  % Kalman update
i
  if kfinit==0
    xp = [MC/2,MR/2,0,0]'
  else
    xp=A*x(i-1,:)' + Bu
  end
  kfinit=1;
  PP = A*P*A' + Q
  K = PP*H'*inv(H*PP*H'+R)
  x(i,:) = (xp + K*([cc(i),cr(i)]' - H*xp))';
  x(i,:)
  [cc(i),cr(i)]
  P = (eye(4)-K*H)*PP

  hold on
    for c = -1*radius: radius/20 : 1*radius
      r = sqrt(radius^2-c^2);
      plot(x(i,1)+c,x(i,2)+r,'r.')
      plot(x(i,1)+c,x(i,2)-r,'r.')
    end
      pause(0.3)
end

% show positions
  figure
  plot(cc,'r*')
  hold on
  plot(cr,'g*')
%end

%estimate image noise (R) from stationary ball
  posn = [cc(55:60)',cr(55:60)'];
  mp = mean(posn);
  diffp = posn - ones(6,1)*mp;
  Rnew = (diffp'*diffp)/5;
  
  t = 0;
  for i = 1:60
      T12(i,1) = T_save{i}(1,1);
      T12(i,2) = T_save{i}(2,1);
      T22(i,1) = T__save{i}(1,1);
      T22(i,2) = T__save{i}(2,1);
      t = t+dt_;
  end
      
      
