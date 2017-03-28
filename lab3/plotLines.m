function plotLines( I1,I2,p1,p2,lines1,lines2,inliers,N)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
m = colormap(jet);
M = [inliers(randi(size(inliers,2),1,N))];
figure()
imshow([I1,I2]);hold on;

for i = 1:size(M,2)
    plot(p1(1, M(i)), p1(2, M(i)),'Marker', 'x','LineWidth',3,'Color',m(ceil(i *size(m,1)/N),:));
end
for i = 1:size(M,2)
    l = lines1(:,M(i));
    if(abs(l(1)) > abs(l(2))) % line is more vertical
        ylim = get(get(gcf,'CurrentAxes'), 'Ylim');
        l1 = [0 1 ylim(1)]'; % line y = ylim(1)    
        l2 = [0 1 ylim(2)]'; % line y = ylim(2)    
    else
        xlim = get(get(gcf,'CurrentAxes'), 'Xlim')/2;
        l1 = [1 0 -xlim(1)]'; % line x = xlim(1)    
        l2 = [1 0 -xlim(2)]'; % line x = xlim(2)
    end
    P1 = cross(l, l1); % intersection point of l with l1
    P2 = cross(l, l2); % intersection point of l with l2

    P1 = P1/P1(3);
    P2 = P2/P2(3);
    
    % plot the line that joins points P1 and P2
    hl = line([P1(1); P2(1)],[P1(2); P2(2)], 'Marker', 'x','LineWidth',0.5,'Color',m(ceil(i *size(m,1)/N),:));
end


for i = 1:size(M,2)  
    plot(p2(1, M(i))+size(I1,2), p2(2, M(i)), 'Marker', 'x','LineWidth',3,'Color',m(ceil(i *size(m,1)/N),:));
end
for i = 1:size(M,2)
    l = lines2(:,M(i));
    if(abs(l(1)) > abs(l(2))) % line is more vertical
        ylim = get(get(gcf,'CurrentAxes'), 'Ylim');
        l1 = [0 1 ylim(1)]'; % line y = ylim(1)    
        l2 = [0 1 ylim(2)]'; % line y = ylim(2)    
    else
        xlim = get(get(gcf,'CurrentAxes'), 'Xlim')/2;
        l1 = [1 0 -xlim(1)]'; % line x = xlim(1)    
        l2 = [1 0 -xlim(2)]'; % line x = xlim(2)
    end
    P1 = cross(l, l1); % intersection point of l with l1
    P2 = cross(l, l2); % intersection point of l with l2

    P1 = P1/P1(3);
    P2 = P2/P2(3);
    
    % plot the line that joins points P1 and P2
    hl = line([P1(1)+size(I1,2); P2(1)+size(I1,2)],[P1(2); P2(2)], 'Marker', 'x','LineWidth',0.5,'Color',m(ceil(i *size(m,1)/N),:));
    
end
end

