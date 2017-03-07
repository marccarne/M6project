function [rectifiedImage] = apply_H (I, H)

Hinv = inv(H);
corners = [1, rows(I) ,1          , rows(I)    ;
           1, 1,       columns(I) , columns(I) ;
           1, 1       ,1          , 1          ];

rectifiedCorners = H * corners;

minRow = floor(min(rectifiedCorners(1,:)));
maxRow = ceil (max(rectifiedCorners(1,:)));
minCol = floor(min(rectifiedCorners(2,:)));
maxCol = ceil (max(rectifiedCorners(2,:)));

rectifiedWidth  = maxCol - minCol + 1;
rectifiedHeight = maxRow - minRow + 1;

origin = [minRow,maxRow] + [1,1]; %The [1,1] is because MATLAB has the compelling need to, for no reason other than screwing your programming routines, start indexes at 1.

X = minCol:1:maxCol;
Y = minRow:1:maxRow;

[XI,YI] = meshgrid(X,Y);
rectifiedCoordinates = [reshape(XI,1,numel(XI))                ;
                        reshape(YI,1,numel(YI))                ;
                        ones(1,length(reshape(XI,1,numel(XI))))];

rectifiedCoordinatesAtOrigin = Hinv * rectifiedCoordinates;

Xq = reshape(rectifiedCoordinatesAtOrigin(1,:),rows(XI),columns(YI));
Yq = reshape(rectifiedCoordinatesAtOrigin(2,:),rows(XI),columns(YI));

X = 1:1:columns(I);
Y = 1:1:rows(I);

rectifiedImage = interp2(X,Y,I,Xq,Yq);

endfunction
