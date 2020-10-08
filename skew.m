%% Skew function
% Jonathan Woolfrey
% May 2018
%
% This function takes a 3-dimensional vector and returns a 3x3
% skew-symmetric matrix.

function ret = skew(x,y,z)
    n = nargin;
    
    switch n
        case 1 
            rows = size(x,1);
            columns = size(x,2);
            
            if rows == columns
                if rows == 3                                                % The input must be a 3x3 matrix
                    ret = [x(3,2);x(1,3);x(2,1)];                           % Return a 3x1 vector
                else
                    disp(['Incorrect number of inputs.']);
                    return 
                end
            else                                                            % The input must be a vector
                ret = [0      -x(3)   x(2)
                       x(3)    0     -x(1)
                      -x(2)    x(1)   0];
            end
        case 2
            disp('Incorrect number of inputs.')
            return
        case 3
            ret = [0   -z   y
                   z    0  -x
                  -y    x   0];
    end
end
