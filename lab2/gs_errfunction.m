function [E] = gs_errfunction( P0, Xobs )

H = reshape(P0(1:9),[3,3]);
%Hinv = inv(H);

%Convert points to homogeneous for applying H
x = Xobs(1:(length(Xobs)/2));
x = reshape(x,[2, (length(Xobs)/4)]);
x_h = [x ; ones(1,size(x,2))];

xp = Xobs(((length(Xobs)/2)+1):end);
xp = reshape(xp,[2, (length(Xobs)/4)]);
xp_h = [xp ; ones(1,size(xp,2))];

x_est = H\xp_h;
xp_est = H*x_h;

E = [x - euclid(x_est) xp - euclid(xp_est)];

end