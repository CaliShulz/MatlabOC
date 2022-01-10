
clear all
clc
close all

% Compare direct transcription, single shooting and indirect single
% shooting

[time_vect_dt,x_opt_dt,u_opt_dt,cost_opt_dt] = run_direct_transcription();
[time_vect_ss,x_opt_ss,u_opt_ss,cost_opt_ss] = run_single_shooting();
[time_vect_is,x_opt_is,u_opt_is,cost_opt_is] = run_indirect_single_shooting();
u_opt_ss(end+1)=u_opt_ss(end);


figure
plot(time_vect_dt(1:end-1),u_opt_dt,'og')
hold on
stairs(time_vect_ss,u_opt_ss,'r')
plot(time_vect_is,u_opt_is,'b')
grid minor
title('Optimal Control','fontsize',14,'interpreter','latex')
xlabel('Time [s]','fontsize',14,'interpreter','latex')
ylabel('Control Action','fontsize',14,'interpreter','latex')
leg1 = legend('Direct Transcription','Direct Single Shooting','Indirect Single Shooting',...
       'Location','Best');
set(leg1,'Interpreter','latex');
set(leg1,'FontSize',14);


figure
plot(x_opt_dt(1,:),x_opt_dt(2,:),'og')
hold on
stairs(x_opt_ss(1,:),x_opt_ss(2,:),'r')
plot(x_opt_is(:,1),x_opt_is(:,2),'b')
grid minor
title('Optimal Trajectories in State Space','fontsize',14,'interpreter','latex')
xlabel('$x_1$','fontsize',14,'interpreter','latex')
ylabel('$x_2$','fontsize',14,'interpreter','latex')
leg1 = legend('Direct Transcription','Direct Single Shooting','Indirect Single Shooting',...
       'Location','Best');
set(leg1,'Interpreter','latex');
set(leg1,'FontSize',14);