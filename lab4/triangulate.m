function [ X ] = triangulate(x1, x2, P1, P2, imsize)
%This function performs triangulation of homogeoneous points between two
%sets of point matches belonging to separate views. This is performed by an
%SVD decomposition of the system generated with the appliance of
%transformation H over the points verctors from view 1 and view 2, and the
%corresponding projection matrices.
%   INPUTS:
% - x1: homogeneous coordintes of point matches in view 1
% - x2: homogeneous coordintes of point matches in view 2
% - P1: projection matrix for view 1
% - P2: projection matrix for view 2
% - imsize: scaling factor in shape of bi-dimensional vector

H = [2/imsize(1),           0, -1;
               0, 2/imsize(2), -1;
               0,           0,  1];

x1 = H * [x1;1];
x2 = H * [x2;1];
P1 = H * P1;
P2 = H * P2;

A = [x1(1) * P1(3,:) - P1(1,:);
     x1(2) * P1(3,:) - P1(2,:);
     x2(1) * P2(3,:) - P2(1,:);
     x2(2) * P2(3,:) - P2(2,:)];

[U,D,V] = svd(A);

X = V(:,end);
X = X ./X(end);
 
end

