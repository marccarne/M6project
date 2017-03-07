%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Lab 1: Image rectification


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 1. Applying image transformations

% ToDo: create the function  "apply_H" that gets as input a homography and
% an image and returns the image transformed by the homography.
% At some point you will need to interpolate the image values at some points,
% you may use the Matlab function "interp2" for that.
% function img = apply_H(I,H)
%     %Get the position of the corners to see the padding needed
%     s=size(I);
%     
%     top_left=H*[0 0 1]';
%     top_right=H*[0 s2 1];
%     bottom_left=H*[s1 0 1];
%     bottom_right=H*[s1 s2 1];
%     
%     %Create a padded image (img)
%     %Apply the transformation pixel-by-pixel
%     
%     for mm=1:s(1)
%         for nn=1:s(2)
%             new_p=H*[mm nn 1]'; %With homogeneous coordinates
%         end
%     end
    



%% 1.1. Similarities
I=imread('Data/0005_s.png'); % we have to be in the proper folder

% ToDo: generate a matrix H which produces a similarity transformation
%Similarity (isometry) consists in a rotation and traslation transformation
x_tras = 20;
y_tras = 30;
tras = [x_tras y_tras]';

alpha = 45 ; %Must be in degrees
alpha = alpha * (2*pi/360); % Conversion to radians
R=[cos(alpha) -sin(alpha); sin(alpha) cos(alpha)];

%Transformation matrix H
H=zeros(3);
H(1:2,1:2)=R;
H(1:2,3)=tras;
H(3,3)=1;

I2 = apply_H(I, H);
figure; imshow(I); figure; imshow(uint8(I2));


%% 1.2. Affinities

% ToDo: generate a matrix H which produces an affine transformation (knowing that the affine matrix transformation is form by 4 transformations)

%Traslation matrix
x_tras= x_tras;
y_tras= y_tras;
Ht=[1 0 x_tras;0 1 y_tras; 0 0 1];

%Scaling matrix
s_factor= 1;
Hs=[s_factor 0 0;0 s_factor 0;0 0 1];

%Rotation matrix --> alpha
alpha = 20 ; %Must be in degrees
alpha = alpha * (2*pi/360); % Conversion to radians;
Halpha=[cos(alpha) -sin(alpha) 0; sin(alpha) cos(alpha) 0; 0 0 1];

%Rotation matrix --> beta
beta = 25 ; %ºMust be in degrees
beta = beta * (2*pi/360); % Conversion to radians;;
Hbeta=[cos(beta) -sin(beta) 0; sin(beta) cos(beta) 0; 0 0 1];

%Matrix traslation+scaling
Hd=Ht*Hs;
H=Hbeta*(Halpha')*Hd*Halpha;

I2 = apply_H(I, H);
figure; imshow(I); figure; imshow(uint8(I2));

% ToDo: decompose the affinity in four transformations: two
% rotations, a scale, and a translation

[U,D,V]=SVD(H);
Halpha_decom=V';
Hd_decom=D;
Hbeta_decom=U*V';

H_decom=Hbeta_decom*Halpha_decom'*D*Halpha_decom;


% ToDo: verify that the product of the four previous transformations
% produces the same matrix H as above
disp('Are matrixes equal? '+int2str(isequal(H,H_decom)));

% ToDo: verify that the proper sequence of the four previous
% transformations over the image I produces the same image I2 as before

I3 = apply_H(I, H_decom);
figure; imshow(I); figure; imshow(uint8(I3));


%% 1.3 Projective transformations (homographies)
% ToDo: generate a matrix H which produces a projective transformation

% choose the image points
I=imread('Data/0000_s.png');
A = load('Data/0000_s_info_lines.txt');
%Extract points in image_1 and its equivalents in image_2
i = 424;
pin(:,1) = [A(i,1) A(i,2)]';
i = 240;
pin(:,2) = [A(i,1) A(i,2)]';
i = 712;
pin(:,3) = [A(i,1) A(i,2)]';
i = 565;
pin(:,4) = [A(i,1) A(i,2)]';

B = load('Data/0001_s_info_lines.txt');
i = 511;
pout(:,1) = [B(i,1) B(i,2)]';
i = 424;
pout(:,2) = [B(i,1) B(i,2)]';
i = 802;
pout(:,3) = [B(i,1) B(i,2)]';
i = 766;
pout(:,4) = [B(i,1) B(i,2)]';

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Implemented David Young's version, University of Sussex, February 2008
%   takes a 2xN matrix of input vectors and a 2xN matrix of output vectors,
%   and returns the homogeneous transformation matrix that maps the inputs 
%   to the outputs, to some approximation if there is noise.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if ~isequal(size(pin), size(pout))
    error('Points matrices different sizes');
end
if size(pin, 1) ~= 2
    error('Points matrices must have two rows');
end
n = size(pin, 2);
if n < 4
    error('Need at least 4 matching points');
end
% Solve equations using SVD
x = pout(1, :); y = pout(2,:); X = pin(1,:); Y = pin(2,:);
rows0 = zeros(3, n);
rowsXY = -[X; Y; ones(1,n)];
hx = [rowsXY; rows0; x.*X; x.*Y; x];
hy = [rows0; rowsXY; y.*X; y.*Y; y];
h = [hx hy];
if n == 4
    [U, ~, ~] = svd(h);
else
    [U, ~, ~] = svd(h, 'econ');
end
Hp = (reshape(U(:,9), 3, 3)).';
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
I2 = apply_H(I, Hp);
figure; imshow(I); figure; imshow(uint8(I2));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 2. Affine Rectification


% choose the image points
I=imread('Data/0000_s.png');
A = load('Data/0000_s_info_lines.txt');

% indices of lines
i = 424;
p1 = [A(i,1) A(i,2) 1]';
p2 = [A(i,3) A(i,4) 1]';
i = 240;
p3 = [A(i,1) A(i,2) 1]';
p4 = [A(i,3) A(i,4) 1]';
i = 712;
p5 = [A(i,1) A(i,2) 1]';
p6 = [A(i,3) A(i,4) 1]';
i = 565;
p7 = [A(i,1) A(i,2) 1]';
p8 = [A(i,3) A(i,4) 1]';

% ToDo: compute the lines l1, l2, l3, l4, that pass through the different pairs of points
%Lines are computed as cross product between two points
l1=cross(p1,p2);
l2=cross(p3,p4);
l3=cross(p5,p6);
l4=cross(p7,p8);

%l1 and l2 are the horizontal ones
%l3 and l4 are the vertical ones

% show the chosen lines in the image
figure;imshow(I);
hold on;
t=1:0.1:1000;
plot(t, -(l1(1)*t + l1(3)) / l1(2), 'y');
plot(t, -(l2(1)*t + l2(3)) / l2(2), 'y');
plot(t, -(l3(1)*t + l3(3)) / l3(2), 'y');
plot(t, -(l4(1)*t + l4(3)) / l4(2), 'y');

% ToDo: compute the homography that affinely rectifies the image
%To affinely rectify the image we have to do that the line at infinity that
%is not at infinity goes to infinity (was not at infinity due to the
%perspective transformation

%Find the vanishing point between lines, cross product of two parallel
%lines

%Vanishing point for horizontal lines
vp_h=cross(l1,l2);

%Vanishing point for vertical lines
vp_v=cross(l3,l4);

%Find line at infinity as we know is the cross product of vanishing points
line_infinity=cross(vp_h,vp_v); %This line should be at infinity but is not

H=zeros(3);
H(3,:)=line_infinity;

I2 = apply_H(I, H);
figure; imshow(uint8(I2));

% ToDo: compute the transformed lines lr1, lr2, lr3, lr4
%Transformed lines are applying the transformation matrix: inv(H)'*line
lr1=(inv(H)')*l1';
lr2=(inv(H)')*l2';
lr3=(inv(H)')*l3';
lr4=(inv(H)')*l4';

% show the transformed lines in the transformed image
figure;imshow(uint8(I2));
hold on;
t=1:0.1:1000;
plot(t, -(lr1(1)*t + lr1(3)) / lr1(2), 'y');
plot(t, -(lr2(1)*t + lr2(3)) / lr2(2), 'y');
plot(t, -(lr3(1)*t + lr3(3)) / lr3(2), 'y');
plot(t, -(lr4(1)*t + lr4(3)) / lr4(2), 'y');

% ToDo: to evaluate the results, compute the angle between the different pair 
% of lines before and after the image transformation
angle_h=acos(dot(lr1,lr2)/norm(lr1)/norm(lr2));
angle_v=acos(dot(lr3,lr4)/norm(lr3)/norm(lr4));

disp('Angle between horizontal lines: ' + int2str(angle_h));
disp('Angle between vertical lines: ' + int2str(angle_v));


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 3. Metric Rectification

%% 3.1 Metric rectification after the affine rectification (stratified solution)

% ToDo: Metric rectification (after the affine rectification) using two non-parallel orthogonal line pairs
%       As evaluation method you can display the images (before and after
%       the metric rectification) with the chosen lines printed on it.
%       Compute also the angles between the pair of lines before and after
%       rectification.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 4. OPTIONAL: Metric Rectification in a single step
% Use 5 pairs of orthogonal lines (pages 55-57, Hartley-Zisserman book)

%% 5. OPTIONAL: Affine Rectification of the left facade of image 0000

%% 6. OPTIONAL: Metric Rectification of the left facade of image 0000

%% 7. OPTIONAL: Affine Rectification of the left facade of image 0001

%% 8. OPTIONAL: Metric Rectification of the left facade of image 0001


