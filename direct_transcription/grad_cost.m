function [grad_cost] = grad_cost(z,h,n_states,n_inputs,dL_dx,dL_du,dphi_dx)

n_steps   = (length(z)-n_states)/(n_states+n_inputs) ;


x_vect = extract_states(z,n_states,n_inputs,n_steps);
u_vect = extract_controls(z,n_states,n_inputs,n_steps);

grad_cost = zeros(size(z));

for i=0:n_steps-1

    grad_cost( 1+i*(n_states+n_inputs)  :   i*(n_states+n_inputs) +n_states ,1  ) = h*dL_dx(x_vect(:,i+1),u_vect(:,i+1));
    grad_cost( n_states+1+i*(n_states+n_inputs)  :   i*(n_states+n_inputs) +n_inputs+n_states  ,1 ) = h*dL_du(x_vect(:,i+1),u_vect(:,i+1));
    
end

grad_cost(end-n_states+1:end , 1) = dphi_dx(x_vect(:,end));




end



