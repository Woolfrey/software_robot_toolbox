function ret = get_inverse(A)
    m = size(A,1);
    n = size(A,2);
    invA = zeros(n,m);

    [U,S,V] = svd(A);
        
    for i = 1:min(m,n)
        for j = 1:min(m,n)
            if S(j,j) > 1E-10
                invA(i,j) = invA(i,j) + V(i,j)/S(j,j);
            end
        end
    end
    invA = invA*U';
    
    ret = invA;
end