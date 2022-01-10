function [dg_dz] = grad_obs_constr(z,n_states,n_inputs,n_steps,x_obs,flag)

%Jacobian of obstacle constraint

x_vect = extract_states(z,n_states,n_inputs,n_steps);
u_vect = extract_controls(z,n_states,n_inputs,n_steps);

n_var = length(z);

dg_dz = zeros(n_steps,n_var); % consider using sparse for speed


% filling Jacobian
for i=1:n_steps
   
    switch flag
    case 'SI'
        block_x_i = [-(x_vect(1,i+1)-x_obs(1))  , -(x_vect(2,i+1)-x_obs(2))];
    case 'DI'
        block_x_i = [0 , 0 , -(x_vect(3,i+1)-x_obs(1))  , -(x_vect(4,i+1)-x_obs(2))];
    end
    
    block_u_i = zeros(1,2);
    
    dg_dz(i ,  n_states + (n_states+n_inputs)*(i-1) + (1:(n_states+n_inputs)) ) = [block_u_i , block_x_i];

    
end

end