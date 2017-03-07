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

alpha = -45 ; %Must be in degrees
alpha = alpha * (2*pi/360); % Conversion to radians
R=[cos(alpha) -sin(alpha); sin(alpha) cos(alpha)];

%Transformation matrix H
H=zeros(3);
H(1:2,1:2)=R;
H(1:2,3)=tras;
H(3,3)=1;

I2 = apply_H(double(I), H);
figure(1); imshow(I); figure(2); imshow(uint8(I2)); title('Similarity transformation');


%% 1.2. Affinities

% ToDo: generate a matrix H which produces an affine transformation (knowing that the affine matrix transformation is form by 4 transformations)

%Traslation matrix
x_tras= x_tras;
y_tras= y_tras;
% Ht=[1 0 x_tras;0 1 y_tras; 0 0 1];
At = [x_tras;y_tras];

%Scaling matrix
s_factor= 0.5;
s_factor2= .75; 
%Hs=[s_factor 0 0;0 s_factor 0;0 0 1];
A_lambda = [s_factor , 0; 0 , s_factor2];

%Rotation matrix --> alpha
alpha = 15 ; %Must be in degrees
alpha = alpha * (2*pi/360); % Conversion to radians;
%Halpha=[cos(alpha) -sin(alpha) 0; sin(alpha) cos(alpha) 0; 0 0 1];
A_alpha = [cos(alpha), -sin(alpha) ; sin(alpha), cos(alpha)];

%Rotation matrix --> beta
beta = 40 ; %ºMust be in degrees
beta = beta * (2*pi/360); % Conversion to radians;;
%Hbeta=[cos(beta) -sin(beta) 0; sin(beta) cos(beta) 0; 0 0 1];
A_beta = [cos(beta), -sin(beta) ; sin(beta), cos(beta)];
A_mbeta = [cos(-beta), -sin(-beta) ; sin(-beta), cos(-beta)];

%Matrix traslation+scaling
% Hd=Ht*Hs;
% H=Hbeta*Hd*Halpha;
A = A_alpha*A_mbeta*A_lambda*A_beta;

H= [A,At];
Haff= [H; 0,0,1];

I2 = apply_H(double(I), Haff);
figure(1); imshow(I); figure(2); imshow(uint8(I2)); title('Affine transformation');

% ToDo: decompose the affinity in four transformations: two
% rotations, a scale, and a translation

[U,D,V]=svd(Haff);
Halpha_decom=V';
Hd_decom=D;
Hbeta_decom=U*V';

H_decom=Hbeta_decom*Halpha_decom'*D*Halpha_decom;


% ToDo: verify that the product of the four previous transformations
% produces the same matrix H as above
disp('Are matrixes equal? '+int2str(isequal(Haff,H_decom)))

% ToDo: verify that the proper sequence of the four previous
% transformations over the image I produces the same image I2 as before

I3 = apply_H(double(I), H_decom);
figure(3); imshow(uint8(I3)); title('Decomposed affine transformation');


%% 1.3 Projective transformations (homographies)
% ToDo: generate a matrix H which produces a projective transformation

%Traslation matrix
x_tras= x_tras;
y_tras= y_tras;
At = [x_tras;y_tras];

%Scaling matrix
s_factor= 2;
s_factor2= 1; 
A_lambda = [s_factor , 0; 0 , s_factor2];

%Rotation matrix --> alpha
alpha = 15 ; %Must be in degrees
alpha = alpha * (2*pi/360); % Conversion to radians;
A_alpha = [cos(alpha), -sin(alpha) ; sin(alpha), cos(alpha)];

%Rotation matrix --> beta
beta = 45 ; %ºMust be in degrees
beta = beta * (2*pi/360); % Conversion to radians;;
A_beta = [cos(beta), -sin(beta) ; sin(beta), cos(beta)];
A_mbeta = [cos(-beta), -sin(-beta) ; sin(-beta), cos(-beta)];

%Matrix traslation+scaling
A = A_alpha*A_mbeta*A_lambda*A_beta;

H= [A,At];
H= [H; 0.0004,0.00002,0.5];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
I2 = apply_H(double(I), H);
figure(1); imshow(I); figure(2); imshow(uint8(I2)); title('Perspective transformation');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 2. Affine Rectification


% choose the image points
I=imread('Data/0001_s.png');
A = load('Data/0001_s_info_lines.txt');

% indices of lines
i = 614;
p1 = [A(i,1) A(i,2) 1]';
p2 = [A(i,3) A(i,4) 1]';
i = 159;
p3 = [A(i,1) A(i,2) 1]';
p4 = [A(i,3) A(i,4) 1]';
i = 802;
p5 = [A(i,1) A(i,2) 1]';
p6 = [A(i,3) A(i,4) 1]';
i = 766;
p7 = [A(i,1) A(i,2) 1]';
p8 = [A(i,3) A(i,4) 1]';

% ToDo: compute the lines l1, l2, l3, l4, that pass through the different pairs of points
%Lines are computed as cross product between two points
l1=cross(p1,p2);
l2=cross(p3,p4);
l3=cross(p5,p6);
l4=cross(p7,p8);

% show the chosen lines in the image
figure(1);imshow(I);
hold on;
t=1:0.1:1000;
plot(t, -(l1(1)*t + l1(3)) / l1(2), 'y');
plot(t, -(l2(1)*t + l2(3)) / l2(2), 'y');
plot(t, -(l3(1)*t + l3(3)) / l3(2), 'r');
plot(t, -(l4(1)*t + l4(3)) / l4(2), 'r');

% ToDo: compute the homography that affinely rectifies the image
%To affinely rectify the image we have to do that the line at infinity that
%is not at infinity goes to infinity (was not at infinity due to the
%perspective transformation

%Find the vanishing point between lines, cross product of two parallel
%lines

%Vanishing point for yellow lines
vp_h=cross(l1,l2);

%Vanishing point for red lines
vp_v=cross(l3,l4);

%Find line at infinity as we know is the cross product of vanishing points
line_infinity=cross(vp_h,vp_v); %This line should be at infinity but is not

% Haff=[1     , 0     ,  0 ;
%       0     , 1     ,  0 ;
%       0     , 0     ,  1];
H=zeros(3);
H(1,1)=1;
H(2,2)=1;
H(3,:)=line_infinity/norm(line_infinity);

% H = Haff * H;

I2 = apply_H2(double(permute(I,[2 1 3])), H);
figure(2); imshow(uint8(permute(I2,[2 1 3])));

% ToDo: compute the transformed lines lr1, lr2, lr3, lr4
%Transformed lines are applying the transformation matrix: inv(H)'*line
lr1=(inv(H'))*l1;
lr2=(inv(H'))*l2;
lr3=(inv(H'))*l3;
lr4=(inv(H'))*l4;

% show the transformed lines in the transformed image
hold on;
t=1:0.1:1000;
plot(t, -(lr1(1)*t + lr1(3)) / lr1(2), 'y');
plot(t, -(lr2(1)*t + lr2(3)) / lr2(2), 'y');
plot(t, -(lr3(1)*t + lr3(3)) / lr3(2), 'r');
plot(t, -(lr4(1)*t + lr4(3)) / lr4(2), 'r');

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
indices = [424,240,48,534,119;
           712,565,493,227,224];
           
for i = 1:5

idx = indices(1,i)
p1 = [A(idx,1) A(idx,2) 1]';
p2 = [A(idx,3) A(idx,4) 1]';
idx = indices(2,i)
p3 = [A(idx,1) A(idx,2) 1]';
p4 = [A(idx,3) A(idx,4) 1]';
l=cross(p1,p2);
m=cross(p3,p4);

M(i,:) = [l(1)*m(1),
          (l(1)*m(2)+l(2)*m(1))/2,
          l(2)*m(2),
          (l(1)*m(3)+l(3)*m(1))/2,
          (l(2)*m(3)+l(3)*m(2))/2,
          l(3)*m(3)];

end

C = null(M);
a = C(1);
b = C(2);
c = C(3);
d = C(4);
e = C(5);
f = C(6);
Conic = [a,b/2,d/2;
         b/2,c,e/2;
         d/2,e/2,f];
H = chol(Conic);

Iopt = apply_H(I,H);

%% 5. OPTIONAL: Affine Rectification of the left facade of image 0000

%% 6. OPTIONAL: Metric Rectification of the left facade of image 0000

%% 7. OPTIONAL: Affine Rectification of the left facade of image 0001

%% 8. OPTIONAL: Metric Rectification of the left facade of image 0001


