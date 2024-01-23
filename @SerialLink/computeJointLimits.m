function [lower, upper] = computeJointLimits(obj)
    maxAcc = 2;

    lower = nan(obj.n,1);
    upper = nan(obj.n,1);
    
    for i = 1:obj.n
        dq = obj.q(i) - obj.link(i).qlim(1);
        lower(i) = max( -obj.hertz*dq, ...
                   max( -obj.link(i).vlim, ...
                        -2*sqrt(maxAcc*dq) ));
                        
        dq = obj.link(i).qlim(2) - obj.q(i);
        
        upper(i) = min( obj.hertz*dq, ...
                min( obj.link(i).vlim, ...
                     2*sqrt(maxAcc*dq) ));               
    end
end