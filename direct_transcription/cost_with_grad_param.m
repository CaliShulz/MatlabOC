function [f,gradf] = cost_with_grad_param(x,param)

n_states = param.n_states; 
n_inputs = param.n_inputs;
L = param.L ;
phi = param.phi;
dL_dx = param.dL_dx ;
dL_du = param.dL_du ;
dphi_dx = param.dphi_dx ;
h = param.h;


f = cost_fun(x,h,n_states,n_inputs,L,phi);

% Gradient of the objective function:

if nargout  > 1
    gradf = grad_cost(x,h,n_states,n_inputs,dL_dx,dL_du,dphi_dx);
end
end

