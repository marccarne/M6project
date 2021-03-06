function [P, X] = factorization_method_andrea(x1, x2)
% Normalize the points
[x1, H1] = normalise2dpts(x1);
[x2, H2] = normalise2dpts(x2);
x{1} = x1;
x{2} = x2;

% Get F and the epipole
[F, ~] = ransac_fundamental_matrix(x1, x2, 2.0); 
[~, ~, V] = svd(F');
e = V(:,end);

lambda1 = ones(1,length(x1));
lambda2 = [];

for i=1:length(x1)
    lambda2(i) = (x1(:,i)' * F' * cross(e,x2(:,i)))/ norm(cross(e,x2(:,i)))^2;
end

d_old = 0;
d = 15;
while (abs(d - d_old)/d) >= 0.1
    % 4_Alternate rescaling the rows of the depth matrix lambda(2x24)(formed by lambdas) to
    % have unit norm and the columns of lambda(2x24) to have unit norm until lambda(2x24)
    % stops changing significantly (usually two loops).
    lambda = [lambda1;lambda2]; % 2x24
    for j=1:2
        % normalize per row
        for i=1:size(lambda,2)
            a = sum(lambda(:,i));
            lambda(:,i) = lambda(:,i) ./ a;
        end
        % normalize per column
        for i=1:size(lambda,1)
            a = sum(lambda(i,:));
            lambda(i,:) = lambda(i,:) ./ a;
        end
    end
    lambda1 = lambda(1,:);
    lambda2 = lambda(2,:);

    % build matrix M and decompose
    M = [lambda1(1,:).*x1(1,:);
        lambda1(1,:).*x1(2,:);
        lambda1(1,:).*x1(3,:);
        lambda2(1,:).*x2(1,:);
        lambda2(1,:).*x2(2,:);
        lambda2(1,:).*x2(3,:)];

    [U,D,V] = svd(M);

    % P and X
    Pproj = U*D(:,1:4); % 3m x 4 = 6x4
    P{1} = Pproj(1:3,:);
    P{2} = Pproj(4:6,:);
    V_T = V';
    X = V_T(1:4,:); % 4 x n = 4x24

    % Convergence
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
P{1} = H1\P{1};
P{2} = H2\P{2};

end