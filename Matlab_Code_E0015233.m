%Sampling Frequency of 20 khz
clear all;
close all;
Fs1=20*10^3;
Ts1=1/Fs1;
Fs2=5*10^3;
Ts2=1/Fs2;
%overall plant transfer function 
num_g = [1E8]; 
den_g =[1 0 0]; 
G = tf(num_g,den_g);
num_g1=[0 (2*pi*5*10^3)^2];
den_g1=[1 (2*0.05)*(2*pi*5*10^(3)) (2*pi*5*10^(3))^2];
G1 = tf(num_g1,den_g1);
overall_plant_tf=G*G1;
figure(1);
bode(overall_plant_tf);
title('Plant Transfer Function')
%Plant Discretized using ZOH
overall_plant_discrete1=c2d(overall_plant_tf,Ts1,'ZOH')
overall_plant_discrete2=c2d(overall_plant_tf,Ts2,'ZOH');
%Notch Filter Design
Num_notch=[1 3140 985960000];
Den_notch=[1 62800 985960000];
G_notch=tf(Num_notch,Den_notch);
figure(2);
bode(G_notch,overall_plant_tf);
title('Notch Filter versus Plant')
grid;
%Notch Filter Discretized using ZOH
G_Notch_Discrete1=c2d(G_notch,Ts1,'tustin')
G_Notch_Discrete2=c2d(G_notch,Ts2,'ZOH');

%OVERALL OPEN LOOP TRANSFER FUNCTION OF THE SYSTEM WITH NOTCH FILTER
figure(3);
bode(G_notch*overall_plant_tf);
title('System With Notch Filter')
grid;

%Kp,Ki and Kd Values of PID Controller
 gain=2.410*2.85;
 kp= 2.85*4*8.26*10^(-4);
 ki = 2.85*54*10^(-8);
 kd =gain*4.60000242*10^(-6);
 %Overall Closed Loop transfer function of the System 
 num_d = [kd kp ki];
 den_d = [1 0];
 D = tf(num_d,den_d)
 D_discrete=c2d(D,Ts1,'tustin')
 C=G_notch*D
 %continuous domain
 overall_open_loop_tf=G*G1*G_notch*D
 overall_closed_loop_tf=feedback(overall_open_loop_tf,1)
 figure(4);
 step(overall_closed_loop_tf,'r-');
 title('Step Response of the Continuous System')
 grid;
 figure(5);
 %Calculation of GM,PM,Wgm and Wpm
 %Overall Compensated Transfer Function P(s)C(s);
 margin(overall_open_loop_tf)
 title('P(S)C(S)');
 %Discrete domain
 C_Discrete1=c2d(C,Ts1,'tustin')
 C_Discrete2=c2d(C,Ts2,'tustin')
 overall_open_loop_tf_discrete1=C_Discrete1*overall_plant_discrete1
 overall_open_loop_tf_discrete2=C_Discrete2*overall_plant_discrete2
 F_SYS_closedloop1 = feedback(overall_open_loop_tf_discrete1,1)
 F_SYS_closedloop2 = feedback(overall_open_loop_tf_discrete2,1)
 
  %Calculation of GM,PM,Wgm and Wpm
 figure(6);
 %Overall Compensated Transfer Function P(Z)C(Z);
 margin(overall_plant_discrete1*C_Discrete1);
 title('P(z)C(z)')
 %Unit Step Response Of the System
 figure(7);
 hold all;
 hold on;
 h = stepplot(F_SYS_closedloop1);
 setoptions(h,'Normalize','on');
 %step(F_SYS_closedloop1,'r-');
 title('Step Response of the Discrete System')
 grid;
%peak of sensitivity transfer function 
 sensitivity_tf=1/(1+overall_open_loop_tf)
 figure(8);
 bode(sensitivity_tf);
 title('Sensitivity transfer function')
 grid;
 
 %  Effect of sampling frequencies on System Performance at 5khz and 20khz
 figure(10);
 stepplot(F_SYS_closedloop2,F_SYS_closedloop1)
 title('Effect of sampling rates');
 legend('5khz', '20khz')
 grid;
