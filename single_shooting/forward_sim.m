function [x_vect] = forward_sim(x_0,u_vect,param)

n_states = param.n_states;
n_steps = param.n_steps;
h = param.h;
f_dyn = param.f_dyn;

x_vect = zeros(n_states,n_steps+1);
x_vect(:,1) = x_0;

for ii=1:n_steps

    x_vect(:,ii+1) = x_vect(:,ii) + h*f_dyn(x_vect(:,ii),u_vect(:,ii));
    
end

end

