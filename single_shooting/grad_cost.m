function [grad_cost] = grad_cost(u_vect,x_vect,param)

n_steps = param.n_steps;
n_inputs = param.n_inputs;
dL_dx = param.dL_dx;
dL_du = param.dL_du;
dphi_dx = param.dphi_dx;
h = param.h;
dx_du = param.dx_du;




% In this case the gradient of the cost is the same dimension as the
% control inputs

grad_cost = zeros(n_inputs,n_steps);

for ii=1:n_steps
   
    grad_cost(:,ii) = h * dL_du(x_vect(:,ii),u_vect(:,ii));
    
    for jj=ii+1:n_steps
        
        grad_cost(:,ii) = grad_cost(:,ii) + h*transpose(dL_dx(x_vect(:,jj),u_vect(:,jj)))*dx_du(:,:,jj,ii);
    
    end
    
    grad_cost(:,ii) = grad_cost(:,ii) + transpose(dphi_dx(x_vect(:,n_steps+1)))*dx_du(:,:,n_steps+1,ii);
    
end

grad_cost = grad_cost(:); % vectorize


end



