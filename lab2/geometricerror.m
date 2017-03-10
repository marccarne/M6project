function d=geometricerror(x1,x2,H)
    %d2 is a vector containing the geometric distance for each pair of
    %points
    s=size(x2);
    num_points=s(2);
    
    d=zeros(1,num_points);
    
    for ii=1:num_points
        d(ii)=norm(euclid(x2(:,ii))-euclid(H*x1(:,ii)))+norm(euclid(H\x2(:,ii))-euclid(x1(:,ii)));
    end
end