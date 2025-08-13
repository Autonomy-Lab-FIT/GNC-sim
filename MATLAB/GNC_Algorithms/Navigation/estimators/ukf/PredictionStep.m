% Prediction step
function [chi_pred, x_pred, P_pred, y_pred, y_mean] = PredictionStep(chi, n, f, W_m, W_c, Q, dt_dyn, t)

    % Propagating sigma points through the model
    chi_pred = zeros(size(chi));
    
    for i = 1:2*n+1
        chi_pred(:, i) = RK4(f, chi(:, i), dt_dyn, t); % ECI
    end
    
    % Predicted mean state
    x_pred = sum(W_m.* chi_pred, 2);
    
    % Predicted Covariance
    chi_pred_1 = chi_pred - x_pred(:,ones(1,size(chi_pred, 2)));
    P_pred = chi_pred_1 * diag(W_c) * chi_pred_1' + Q;
   
    % predicted measurements (needs to confirm this logic)
    y_mean = x_pred;
    y_pred = chi_pred;

end