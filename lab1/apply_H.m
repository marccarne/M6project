function [rectifiedImage] = apply_H (I, H)

%Hinv = inv(H);
%corners = [1, 1,       columns(I) , columns(I) ;
%           1, rows(I) ,1          , rows(I)    ;
%           1, 1       ,1          , 1          ]

%Matlab ver.
corners = [1,   1       , size(I,2) , size(I,2) ;
           1, size(I,2) ,   1       , size(I,1) ;
           1,   1       ,   1       , 1         ];
       
rectifiedCorners = H * corners;

% minCol = min(floor(min(rectifiedCorners(1,:))),1);
% maxCol = max(ceil (max(rectifiedCorners(1,:))),columns(I));
% minRow = min(floor(min(rectifiedCorners(2,:))),1);
% maxRow = max(ceil (max(rectifiedCorners(2,:))),rows(I));

%Matlab ver
minCol = min(floor(min(rectifiedCorners(1,:))),1);
maxCol = max(ceil (max(rectifiedCorners(1,:))),size(I,2));
minRow = min(floor(min(rectifiedCorners(2,:))),1);
maxRow = max(ceil (max(rectifiedCorners(2,:))),size(I,1));

%rectifiedWidth  = maxCol - minCol + 1;
%rectifiedHeight = maxRow - minRow + 1;

%origin = [minRow,maxRow] + [1,1]; %The [1,1] is because MATLAB has the compelling need to, for no reason other than screwing your programming routines, start indexes at 1.

X = minCol:1:maxCol;
Y = minRow:1:maxRow;

[XI,YI] = meshgrid(X,Y);
rectifiedCoordinates = [reshape(XI,1,numel(XI))                ;
                        reshape(YI,1,numel(YI))                ;
                        ones(1,length(reshape(XI,1,numel(XI))))];

rectifiedCoordinatesAtOrigin = H\rectifiedCoordinates; % Hinv * rectifiedCoordinates;

%Xq = reshape(rectifiedCoordinatesAtOrigin(1,:),rows(XI),columns(YI));
%Yq = reshape(rectifiedCoordinatesAtOrigin(2,:),rows(XI),columns(YI));

Xq = reshape(rectifiedCoordinatesAtOrigin(1,:),size(XI,1),size(YI,2));
Yq = reshape(rectifiedCoordinatesAtOrigin(2,:),size(XI,1),size(YI,2));

%X = 1:1:columns(I);
%Y = 1:1:rows(I);

X = 1:1:size(I,2);
Y = 1:1:size(I,1);

rectifiedImage(:,:,1) = interp2(X,Y,I(:,:,1),Xq,Yq);
rectifiedImage(:,:,2) = interp2(X,Y,I(:,:,2),Xq,Yq);
rectifiedImage(:,:,3) = interp2(X,Y,I(:,:,3),Xq,Yq);
end
%endfunction
