function  F = fundMatrix(p1,p2)
% Computes the fundamental matrix F with the 8 points algorithm

% Build W from the correspondences:

    n = size(p1,2);
    W = zeros(8,9);
    for i = 1:n
        W(i,:) = [ p2(1,i) * p1(1,i), p2(1,i) * p1(2,i), p2(1,i),p2(2,i) * p1(1,i), p2(2,i) * p1(2,i), p2(2,i),p1(1,i),         p1(2,i),     1];
    end
    
    % Solve the Wf = 0 system
    [U,D,V] = svd(W);
    
    f = V(:,end);
    % Compose rank 3 F
    F_rank3 = [f(1),f(2),f(3),
               f(4),f(5),f(6),
               f(7),f(8),f(9)];
    
    % Compose rank 2 F
    [U,D,V] = svd(F_rank3);
    diags = diag(D);
    F = U * diag([diags(1:end-1);0]) * V';
        
end

