
function [time_vect,x_opt,u_opt,cost_opt,param_out] = run_obs_avoidance_DI()
addpath('obs_avoidance')
% Direct transcription problem solved with fmincon
% exact gradients and jacobian
% Explicit Euler step integration

% Dynamics
f_dyn =@(x,u)  [u(1); u(2) ; x(1) ; x(2)];

% Jacobian of Dynamics
df_dx = @(x,u) [ 0  0  0  0 ; 0 0 0 0 ; 1 0 0 0 ; 0 1 0 0 ] ;
df_du = @(x,u) [ 1  0 ; 0 1 ; 0 0 ; 0 0 ];

% Weighting function
Q = 1E-3*eye(4);
R = 1E-3;
P = 10*eye(4);


% Target as soft constraint
x_T = [0 ; 0 ; 1 ; 1];

% Obstacle parameters
x_obs(1) = 0.5;
x_obs(2) = 0.5;
r_obs    = 0.2;

param_out.x_obs = x_obs;
param_out.r_obs = r_obs;
param_out.x_T   = x_T;

% Cost function
L = @(x,u) 0.5*(x-x_T)'*Q*(x-x_T) + 0.5*u'*R*u ; % Running cost
phi = @(x) 0.5*(x-x_T)'*P*(x-x_T) ; % Final cost

dL_dx   =@(x,u) Q*(x-x_T);
dL_du   =@(x,u) R*u;
dphi_dx =@(x) P*(x-x_T);



x_0 = [0 ; 0 ; 0 ; 0]; % Initial condition

h = 0.05;
n_steps = 100;
n_states = 4;
n_inputs = 2;

% 
% % z = [x0,u0,x1,u1,...,x_{N-1},u_{N-1},x_{N}]
% 
% 
% cost = cost_fun(z,h,n_states,n_inputs,L,phi);
% 
% g =  dyn_constr(z,f_dyn,x_0,h,n_states,n_inputs,n_steps);   % it is size n_states*(n_steps+1) = initial condition + dynamics
% [grad_cost] = grad_cost(z,n_states,n_inputs,dL_dx,dL_du,dphi_dx)
% 
% 
% [dg_dz] = grad_constr(z,df_dx,df_du,h,n_states,n_inputs,n_steps)

param.n_steps = n_steps;
param.n_states = n_states;
param.n_inputs = n_inputs;
param.L = L;
param.phi = phi;
param.dL_dx = dL_dx;
param.dL_du = dL_du;
param.dphi_dx = dphi_dx;
param.f_dyn = f_dyn;
param.df_dx = df_dx;
param.df_du = df_du;
param.h = h;
param.x_0 = x_0;
param.x_obs = x_obs;
param.r_obs = r_obs;
param.flag  = "DI";

% test functions
%[f,gradf] = cost_with_grad_param(z,param) ;
%[c,ceq,DC,DCeq] = constr_fun_grad(z,param);


% Solve with fmincon
% Exact gradient is supplied and checked

options = optimoptions('fmincon',...
    'SpecifyObjectiveGradient',true,'SpecifyConstraintGradient',true,'CheckGradients',true,'Display','iter');

objfun = @(x) cost_with_grad_param(x,param) ;
nonlcon = @(x) constr_fun_grad(x,param);




A = [];
b = [];
Aeq = [];
beq = [];
lb = [];
ub = [];

z0 = zeros(n_steps*(n_states+n_inputs)+n_states,1);

[z_opt,fval] = fmincon(objfun,z0,A,b,Aeq,beq,lb,ub,nonlcon,options);


% Retrieve optimal state, control and cost

time_vect = 0:h:n_steps*h;
x_opt = extract_states(z_opt,n_states,n_inputs,n_steps);
u_opt = extract_controls(z_opt,n_states,n_inputs,n_steps);
cost_opt = cost_fun(z_opt,h,n_states,n_inputs,L,phi);



end


