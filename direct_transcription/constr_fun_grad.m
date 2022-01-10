function [c,ceq,DC,DCeq] = constr_fun_grad(x,param)

f_dyn = param.f_dyn;
x_0 = param.x_0;
h = param.h;
n_states = param.n_states;
n_inputs = param.n_inputs;
n_steps = param.n_steps;
df_dx = param.df_dx;
df_du = param.df_du;

c = [];
% No nonlinear equality constraints
ceq=dyn_constr(x,f_dyn,x_0,h,n_states,n_inputs,n_steps); 

% Gradient of the constraints:
if nargout > 2
    DC= [];
    DCeq = grad_constr(x,df_dx,df_du,h,n_states,n_inputs,n_steps)';
end

end


