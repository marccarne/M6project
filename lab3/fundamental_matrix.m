function [ F ] = fundamental_matrix(p1,p2)

%% Normalization of p1 and p2
n = size(p1,2);
p1 = p1 ./ [p1(3,:);p1(3,:);p1(3,:)];
p2 = p2 ./ [p2(3,:);p2(3,:);p2(3,:)];

% Homogeneous transformations:
H1 = normalizerHomography(p1);
H2 = normalizerHomography(p2);

% Get the new points
 
q1 = H1 * p1;
q2 = H2 * p2;

F = fundMatrix(q1,q2);
F = H2' * F * H1;

end

