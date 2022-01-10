function [g] = dyn_constr(z,f,x_0,h,n_states,n_inputs,n_steps)

x_vect = extract_states(z,n_states,n_inputs,n_steps);
u_vect = extract_controls(z,n_states,n_inputs,n_steps);

g = x_0 - x_vect(:,1) ; % initial condition constraint

for i=1:n_steps
   
    g_tmp = x_vect(:,i)+h*f(x_vect(:,i),u_vect(:,i))-x_vect(:,i+1) ; % dynamic constraint at i-th step
    g = [ g ; g_tmp]; % stack them
    
end
    
end


