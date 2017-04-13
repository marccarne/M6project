function [P,X] = factorization_method(x1,x2)

% Normalize the points, using the function given.

[x1,H1] = normalise2dpts(x1);
[x2,H2] = normalise2dpts(x2);

% Initialize variables
[F, Finliers] = ransac_fundamental_matrix(x1,x2,2.0);
lambda1 = ones(length(x1));
lambda2 = ones(length(x2));

% Determine the epipole
[~,~,V] = svd(F');
e = V(:,end);

d = 0;
d_old = 5;


while(abs(d-d_old) >= 0.1)
    lambda = [lambda1;lambda2] % 2xlength(x1) matrix
    lambda_old = lambda;
    similarity_matrix = 100;
    i = 0;
    % Alternate rescalling rows and columns of matrix A until convergence
    while(similarity_matrix > 0.1)
        lambda_old = lambda;
        if(i == 0)
            % Normalize rows
            for i=1:size(lambda,2)
            a = sum(lambda(:,i));
            lambda(:,i) = lambda(:,i) ./ a;
            end
        else
            % Normalize columns
            for i=1:size(lambda,1)
            a = sum(lambda(i,:));
            lambda(i,:) = lambda(i,:) ./ a;
            end
        end
        i = 1 - i;
        similarity_matrix = sum(sum(abs(lambda-lambda_old)));
    end
    
    lambda1 = lambda(1,:);
    lambda2 = lambda(2,:);
    
    % Building M
    
    M = [lambda1(1,:).*x1(1,:);
        lambda1(1,:).*x1(2,:);
        lambda1(1,:).*x1(3,:);
        lambda2(1,:).*x2(1,:);
        lambda2(1,:).*x2(2,:);
        lambda2(1,:).*x2(3,:)];
    
    % SVD of M
    
    [U,D,V]  = svd(M);
    
    % compute P and X
    Pproj = U*D(:,1:4); 
    P{1} = Pproj(1:3,:);
    P{2} = Pproj(4:6,:);
    V_T = V';
    X = V_T(1:4,:); 
    
    d_old = d;
    d = 0;
    
    for j=1:2
        m = euclid(P{j}*X);
        m1 = euclid(x{j});
        d = d + sum(sum(norm(m1(:,:) - m(:,:)).^2));
    end
    % Normalize 3D point
    X = X ./ repmat(X(end,:), size(X,1), 1);

    lambda1 = P{1}*X;
    lambda1 = lambda1(end,:);
    lambda2 = P{2}*X;
    lambda2 = lambda2(end,:);
end

% De-normalise

Pproj1 = P{1};
Pproj1 = inv(T1)*Pproj1;
Pproj2 = P{2};
Pproj2 = inv(T2)*Pproj2;
P = [Pproj1; Pproj2];

end