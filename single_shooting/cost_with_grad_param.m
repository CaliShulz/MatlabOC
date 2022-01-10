function [f,gradf] = cost_with_grad_param(x,param)

n_inputs = param.n_inputs;
x_0 = param.x_0;
n_steps = param.n_steps;

u_vect = reshape(x,n_inputs,n_steps);

%% Need to add forward simulation at each computation of the cost and gradient
x_vect = forward_sim(x_0,u_vect,param);

f = cost_fun(x_vect,u_vect,param);

% Gradient of the objective function:

if nargout  > 1
    
    dx_du = get_sensitivity(x_vect,u_vect,param);
    param.dx_du = dx_du;
    gradf = grad_cost(u_vect,x_vect,param);
    
end
end

