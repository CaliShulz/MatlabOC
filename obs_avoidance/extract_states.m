function [x_vect] = extract_states(z,n_states,n_inputs,n_steps)

    x_vect = zeros(n_states,n_steps+1);

    for i=0:n_steps

        x_vect(:,i+1) = z( 1+i*(n_states+n_inputs)  :   i*(n_states+n_inputs) +n_states);

    end
end

