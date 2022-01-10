function [cost] = cost_fun(x_vect,u_vect,param)

h = param.h;
n_steps = param.n_steps;

L = param.L;
phi = param.phi;



cost = 0;

for i=1:n_steps
   
    cost = cost + h*L(x_vect(:,i),u_vect(:,i));
    
end

    cost = cost + phi(x_vect(:,end));
    


end



