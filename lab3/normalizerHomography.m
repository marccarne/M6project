function H = normalizerHomography(x)
% Returns a Homography for the set of points x such as x'= H*x has the
% center of coordinates at the centroid of x and the mean square distance
% between x' and the center is 2. H is done as 
% in https://cs.adelaide.edu.au/~wojtek/papers/pami-nals2.pdf

n = size(x,2);

centroid = sum(x,2)/n;
scale = sqrt(sum(sum((x(1:2,:) - repmat(centroid(1:2),1,n)).^2))/(2*n));

H = [ 1/scale, 0, -centroid(1,1)/scale;
      0, 1/scale, -centroid(2,1)/scale;
      0,       0,                    1;];
end

