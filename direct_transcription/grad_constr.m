function [dg_dz] = grad_constr(z,df_dx,df_du,h,n_states,n_inputs,n_steps)

%Dynamic Constraint Jacobian

x_vect = extract_states(z,n_states,n_inputs,n_steps);
u_vect = extract_controls(z,n_states,n_inputs,n_steps);

n_var = length(z);

dg_dz = zeros(n_states*(n_steps+1),n_var); % consider using sparse for speed

dg_dz(1:n_states,1:n_states) = -eye(n_states);

% filling Jacobian
for i=1:n_steps
   
    block_x_i = eye(n_states)+h*df_dx(x_vect(:,i),u_vect(:,i));
    block_u_i = h*df_du(x_vect(:,i),u_vect(:,i));
    block_x_i_p_one = -eye(n_states);
    
    n_states+1+(i-1)*n_states:(i+1)*n_states ;
    
    dg_dz(n_states+1+(i-1)*n_states:(i+1)*n_states , 1+(i-1)*(n_states+n_inputs):(i-1)*(n_states+n_inputs) +n_states  ) = block_x_i ;
    dg_dz(n_states+1+(i-1)*n_states:(i+1)*n_states , 1+(i-1)*(n_states+n_inputs)+n_states:(i-1)*(n_states+n_inputs)+n_states +n_inputs  ) = block_u_i ;
    dg_dz(n_states+1+(i-1)*n_states:(i+1)*n_states , 1+i*(n_states+n_inputs):i*(n_states+n_inputs) +n_states   ) = block_x_i_p_one ;
    
    
end

end

