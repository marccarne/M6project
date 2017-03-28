function h=plotColorMatches(I1,I2,p1,p2)
    m = colormap(jet);
    I3 = [I1;I2];
    height = size(I1,1);
    imshow(I3);hold on;
    for i = 1:size(p1,2)
        dx = p2(1,i) - p1(1,i);
        dy = p2(2,i) - p1(2,i);
        angleIdx = ceil(size(m,1) * (atan2(dy,dx)+pi)/(2*pi));
        plot([p1(1,i),p2(1,i)],[p1(2,i),p2(2,i)+height],'Color',m(angleIdx,:));
    end
end