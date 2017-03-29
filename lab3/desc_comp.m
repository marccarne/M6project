function [id] =desc_comp(descA,descB)
for i=1:size(descB,2)
    comp(i) = norm(descB(:,i) - descA);
end 
[~,id] = min(comp);
end