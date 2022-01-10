
clear all
clc
close all

% Compare obstacle avoidance problem single integrator vs double integrator

% Single obstacle
% Direct transcription with Euler discretization step


[time_vect,x_opt_si,u_opt_si,cost_opt_si,param_out] = run_obs_avoidance_SI();
[time_vect,x_opt_di,u_opt_di,cost_opt_di,~] = run_obs_avoidance_DI();


% Plot trajectories SI
figure
plot(x_opt_si(1,1),x_opt_si(2,1),'o','MarkerSize',10,'Color','b','MarkerFaceColor','b');
hold on
plot(x_opt_si(1,:),x_opt_si(2,:),'--','LineWidth',2,'Color','b')

% Plot obstacle
x_obs = param_out.x_obs;
r_obs = param_out.r_obs;
x_T   = param_out.x_T;

dummy_angle = linspace(0,2*pi,100);
plot(x_obs(1) + r_obs*cos(dummy_angle),x_obs(2) + r_obs*sin(dummy_angle),'LineWidth',2,'Color','r');

% Plot target
plot(x_T(1),x_T(2),'o','MarkerSize',10,'Color','g','MarkerFaceColor','g');
axis([-0.1,1.1,-0.1,1.1])

% Plot trajectories DI
plot(x_opt_di(3,:),x_opt_di(4,:),'LineWidth',2,'Color','b')

grid minor
title('Optimal Trajectories','fontsize',14,'interpreter','latex')
xlabel('x [m]','fontsize',14,'interpreter','latex')
ylabel('y [m]','fontsize',14,'interpreter','latex')
leg1 = legend('IC','Single Integrator','Obstacle','Target','Double Integrator',...
       'Location','Best');
set(leg1,'Interpreter','latex');
set(leg1,'FontSize',14);

