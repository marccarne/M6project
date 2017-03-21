%% Compute 2D homography
function H = homography2d(x1,x2)
    %Having the set of correspondent points we have to create the 2D
    %homography using the SVD decomposition of the matrix A created by the
    %points
    
    %To compute the homography we have 4 pairs of correspondences, so for
    %each correspondence we have to create a 2x9 matrix and the create the
    %matrix A, 8x9 matrix
    
    %x' points are the points we want to move, so from an image to the
    %reference (central image). x2 will be the central image
    
    % x2 = H * x1;
    s=size(x1);
    
    A=zeros(8,9); %'A' matrix to compute the homography
    
    Ai=zeros(2,9); %Matrix for a pair of points
    for ii=1:s(2) %Number of cols in the points matrix, for each pair of points
        %x'--> in the b reference
        x1_x=x2(1,ii);
        x1_y=x2(2,ii);
        x1_z=x2(3,ii);
        %x
        x2_x=x1(1,ii);
        x2_y=x1(2,ii);
        x2_z=x1(3,ii);
        
        %First row
        Ai(1,:)=[0 0 0 -x1_z*x2_x -x1_z*x2_y -x1_z*x2_z x1_y*x2_x x1_y*x2_y x1_y*x2_z];
        Ai(2,:)=[x1_z*x2_x x1_z*x2_y x1_z*x2_z 0 0 0 -x1_x*x2_x -x1_x*x2_y -x1_x*x2_z];
        
        A(1+2*(ii-1):1+2*(ii-1)+1,:)=Ai;
    end
    
    %Having A constructed we have to decompose it
    [U,D,V]=svd(A);
    
    %'h', the flat H (homography) is the last column of V
    
    h=V(:,end);
    
    H=vec2mat(h,3); %Create the 3x3 matrix, 'h' has 9 positions (coefficients)
end