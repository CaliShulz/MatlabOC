function [error,grad_out] = test_gradient(x_test,cost_with_grad_param,param)
% Test gradient with complex step differentiation

[dummy,grad] = cost_with_grad_param(x_test,param);

grad_complex_step = zeros(length(x_test),1);
grad_finite_difference = zeros(length(x_test),1);
delta_x = 10^-6;

for ii = 1:length(x_test)
    
    h_test = zeros(length(x_test),1);
    h_test(ii) = delta_x;
    grad_finite_difference(ii) = (cost_with_grad_param(x_test+h_test,param)-cost_with_grad_param(x_test,param))./delta_x;
    grad_complex_step(ii) = imag(cost_with_grad_param(x_test+i*h_test,param))./delta_x;
    
end

error.complex_step = grad-grad_complex_step;
error.finite_difference = grad-grad_finite_difference;

grad_out.grad_sens = grad;
grad_out.grad_fd = grad_finite_difference;
grad_out.grad_cs = grad_complex_step

end

