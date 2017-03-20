function [ homogPoint] = homogeneous(point)
% For a vector point, returns the homogeneous coordinates, adding an extra 
% row of ones.

    homogPoint = point;
    homogPoint(size(point,1)+1,:) = 1;


end

