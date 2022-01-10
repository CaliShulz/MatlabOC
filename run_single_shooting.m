
function [time_vect,x_opt,u_opt,cost_opt] = run_single_shooting()
addpath('single_shooting')

% Single shooting methods solved with fminunc
% exact gradients and jacobian
% Explicit Euler step integration

% Dynamics
f_dyn =@(x,u)  [(1-x(2).^2).*x(1) - x(2) + u; x(1)]; % Van Der Pol Oscillator

% Jacobian of Dynamics
df_dx = @(x,u) [ 1-x(2).^2   -2*x(2)*x(1)-1   ; 1  0 ] ;
df_du = @(x,u) [ 1 ; 0 ];

% Weighting function
Q = 0.001*eye(2);
R = 0.001;
P = 10*eye(2);

% Cost function
L = @(x,u) 0.5*transpose(x)*Q*x + 0.5*R*u.^2 ; % Running cost
phi = @(x) 0.5*x'*P*x ; % Final cost

dL_dx =@(x,u) Q*x;
dL_du =@(x,u) R*u;
dphi_dx =@(x) P*x;



x_0 = [1;1]; % Initial condition

h = 0.01;
n_steps = 100;
n_states = 2;
n_inputs = 1;


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

u_vect = zeros(n_inputs,n_steps);
[x_vect] = forward_sim(x_0,u_vect,param);
%[dx_du] = get_sensitivity(x_vect,u_vect,param);

%param.dx_du = dx_du;
%[grad_cost] = grad_cost(u_vect,x_vect,param);

z0 = ones(n_steps*n_inputs,1);
%[f,gradf] = cost_with_grad_param(z0,param);


[error,grad_out] = test_gradient(z0,@cost_with_grad_param,param);

max(error.finite_difference)/min(grad_out.grad_sens)
max(error.complex_step)/min(grad_out.grad_sens)
% test functions
%[f,gradf] = cost_with_grad_param(z,param) ;
%[c,ceq,DC,DCeq] = constr_fun_grad(z,param);


% Solve with fmincon
% Exact gradient is supplied and checked

%options = optimoptions('fminunc',...
%    'SpecifyObjectiveGradient',true,'CheckGradients',true,'Display','iter');

options = optimoptions('fminunc',...
    'SpecifyObjectiveGradient',true,'Display','iter');

objfun = @(x) cost_with_grad_param(x,param) ;

% 

[z_opt,fval] = fminunc(objfun,z0,options);
% 
% 
% % Retrieve optimal state, control and cost
% 
time_vect = 0:h:n_steps*h;
u_opt = reshape(z_opt,n_inputs,n_steps);
x_opt = forward_sim(x_0,u_opt,param);
cost_opt = cost_fun(x_opt,u_opt,param);

end



