function [dx_du] = get_sensitivity(x_vect,u_vect,param)

% Return sensitivity of state with respect to control actions dx_du

n_steps = param.n_steps;
n_states = param.n_states;
n_inputs = param.n_inputs;

df_dx = param.df_dx;
df_du = param.df_du;
h = param.h;


% discretized time varying jacobian

dx_du = zeros(n_states,n_inputs,n_steps,n_steps);


for control_ii=1:n_steps

    state_ii = control_ii;
    dx_du(:,:,state_ii+1,control_ii) = h * df_du(x_vect(:,state_ii),u_vect(:,control_ii));
    
    for ii = state_ii+2:n_steps+1

        dx_du(:,:,ii,control_ii) = (eye(n_states)+h*df_dx(x_vect(:,ii-1),u_vect(:,ii-1))) * dx_du(:,:,ii-1,control_ii);

    end
 
end    


end

