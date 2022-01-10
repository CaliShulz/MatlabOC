function [g] = obs_constr(z,n_states,n_inputs,n_steps,x_obs,r_obs,flag)
% single obstacle constraint

x_vect = extract_states(z,n_states,n_inputs,n_steps);
u_vect = extract_controls(z,n_states,n_inputs,n_steps);

% Get position states
switch flag
    case 'SI'
        x_pos = x_vect(1:2,:);             % Change for double integrator
    case 'DI'
        x_pos = x_vect(3:4,:);
end

g = [];

for i=1:n_steps
   
    g_tmp = -0.5*((x_pos(1,i+1)-x_obs(1)).^2 + (x_pos(2,i+1)-x_obs(2)).^2) + 0.5*r_obs.^2 ;
    g = [ g ; g_tmp]; % stack them
    
end

end