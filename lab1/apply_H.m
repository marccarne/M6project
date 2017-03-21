function [rectifiedImage] = apply_H (I, H)


columns = size(I,2);
rows = size(I,1);

%Matlab ver.
corners = [1,   1       , columns   , columns ;
           1, rows      ,   1       , rows    ;
           1,   1       ,   1       , 1       ];
       
rectifiedCorners = H * corners;


minCol = floor(min(rectifiedCorners(1,:)));
maxCol = ceil (max(rectifiedCorners(1,:)));
minRow = floor(min(rectifiedCorners(2,:)));
maxRow = ceil (max(rectifiedCorners(2,:)));



X = minCol:1:maxCol;
Y = minRow:1:maxRow;

[XI,YI] = meshgrid(X,Y);
rectifiedCoordinates = [reshape(XI,1,numel(XI))                ;
                        reshape(YI,1,numel(YI))                ;
                        ones(1,length(reshape(XI,1,numel(XI))))];

rectifiedCoordinatesAtOrigin = H\rectifiedCoordinates; % Hinv * rectifiedCoordinates;

Xq = reshape(rectifiedCoordinatesAtOrigin(1,:),size(XI,1),size(YI,2));
Yq = reshape(rectifiedCoordinatesAtOrigin(2,:),size(XI,1),size(YI,2));

X = 1:1:size(I,2);
Y = 1:1:size(I,1);

rectifiedImage(:,:,1) = interp2(X,Y,I(:,:,1),Xq,Yq);
rectifiedImage(:,:,2) = interp2(X,Y,I(:,:,2),Xq,Yq);
rectifiedImage(:,:,3) = interp2(X,Y,I(:,:,3),Xq,Yq);
end
%endfunction
