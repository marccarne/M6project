function score = ssd(Ileft,Iright)
%Difference between frames  
dif = Ileft - Iright;
dif = dif.^2;
%Sum of Square Differences
score = sum(sum(dif));
end