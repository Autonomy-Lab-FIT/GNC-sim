% Correction Step
function [x_mean, P_covar, y_meas] = CorrectionStep(W_c, y_pred, y_mean, chi_pred, x_pred, P_pred, R, y_meas)
  
    % % cross covariance
    y_pred_1 = y_pred - y_mean(:, ones(1,size(chi_pred,2)));
    chi_pred_1 = chi_pred - x_pred(:,ones(1,size(chi_pred, 2)));

    P_yy = y_pred_1 * diag(W_c) * y_pred_1' + R;
    P_xy = chi_pred_1 * diag(W_c) * y_pred_1';
   
    % state estimate and covariance
    K = real(P_xy * inv(P_yy));
    x_mean = x_pred + (K * (y_meas - y_mean)); % ukf estimated
    P_covar = P_pred - (K * P_xy');

end
