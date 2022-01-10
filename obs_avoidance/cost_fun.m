function [cost] = cost_fun(z,h,n_states,n_inputs,L,phi)

n_steps   = (length(z)-n_states)/(n_states+n_inputs) ;


x_vect = extract_states(z,n_states,n_inputs,n_steps);
u_vect = extract_controls(z,n_states,n_inputs,n_steps);

cost = 0;

for i=1:n_steps
   
    cost = cost + h*L(x_vect(:,i),u_vect(:,i));
    
end

    cost = cost + phi(x_vect(:,end));
    






end



