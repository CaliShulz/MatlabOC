function [u_vect] = extract_controls(z,n_states,n_inputs,n_steps)

    u_vect = zeros(n_inputs,n_steps);

    for i=0:n_steps-1


        u_vect(:,i+1) = z( n_states+1+i*(n_states+n_inputs)  :   i*(n_states+n_inputs) +n_inputs+n_states);

    end

end
