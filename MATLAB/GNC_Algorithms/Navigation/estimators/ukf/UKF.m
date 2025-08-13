function [x_mean, P_covar] = UKF(f, x, P, Q, R, alpha, beta, kappa, dt_dyn, dt_gps, t, y_meas)
    
    n = length(x); % number of states
   
    % lambda, scaling parameter, create's spread based on the alpha and beta parameters, 
    % higher alpha and beta values spreads the sigma points from the mean and covariance
    lambda = alpha^2 * (n + kappa) - n; 

    c = n+lambda;                                     %scaling factor
    
    W_m = [lambda/c 0.5/c+zeros(1,2*n)];            % weights of the mean

    W_c = W_m;                                      % weights of the covariance

    W_c(1)=W_c(1)+(1-alpha^2+beta);  % weights of the covariance

    c = sqrt(c); 
                                    
    chi = SigmaPoints(c, P, x);                     % Generate Sigma Points ECI Frame
    
    % Prediction Step
    [chi_pred, x_pred, P_pred, y_pred, y_mean] = PredictionStep(chi, n, f, W_m, W_c, Q, dt_dyn, t);
    
   % correction step
    if ~isempty(y_meas)
        %disp("updating with measurements")
        [x_mean, P_covar, ~] = CorrectionStep(W_c, y_pred, y_mean, chi_pred, x_pred, P_pred, R, y_meas);
        
    else

        x_mean = x_pred;
        P_covar = P_pred;
        
    end

end