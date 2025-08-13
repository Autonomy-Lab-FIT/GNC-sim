% calculate the sigma points
function chi = SigmaPoints(c, P, x)
    A = c*chol(P)'; % sqrt((n + lambda) * P)' 6x6
    Y = x(:,ones(1,numel(x))); % expanding state vector from 6x1 to 6x6
    chi = [x Y+A Y-A]; % sigma points 
end