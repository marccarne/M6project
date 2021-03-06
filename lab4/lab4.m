%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Lab 4: Reconstruction from two views (knowing internal camera parameters) 
% (optional: depth computation)

addpath('../lab2/sift'); % ToDo: change 'sift' to the correct path where you have the sift functions
addpath('../lab3'); 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 1. Triangulation

% ToDo: create the function triangulate.m that performs a triangulation
%       with the homogeneous algebraic method (DLT)
%
%       The entries are (x1, x2, P1, P2, imsize), where:
%           - x1, and x2 are the Euclidean coordinates of two matching 
%             points in two different images.
%           - P1 and P2 are the two camera matrices
%           - imsize is a two-dimensional vector with the image size

%% Test the triangulate function
% Use this code to validate that the function triangulate works properly

P1 = eye(3,4);
c = cosd(15); s = sind(15);
R = [c -s 0; s c 0; 0 0 1];
t = [.3 0.1 0.2]';
P2 = [R t];
n = 8;
X_test = [rand(3,n); ones(1,n)] + [zeros(2,n); 3 * ones(1,n); zeros(1,n)];
x1_test = euclid(P1 * X_test);
x2_test = euclid(P2 * X_test);

N_test = size(x1_test,2);
X_trian = zeros(4,N_test);
for i = 1:N_test
    X_trian(:,i) = triangulate(x1_test(:,i), x2_test(:,i), P1, P2, [2 2]);
end

% error
euclid(X_test) - euclid(X_trian)


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 2. Reconstruction from two views

%% Read images
Irgb{1} = imread('Data/0001_s.png');
Irgb{2} = imread('Data/0002_s.png');
I{1} = sum(double(Irgb{1}), 3) / 3 / 255;
I{2} = sum(double(Irgb{2}), 3) / 3 / 255;
[h,w] = size(I{1});


%% Compute keypoints and matches.
points = cell(2,1);
descr = cell(2,1);
for i = 1:2
    [points{i}, descr{i}] = sift(I{i}, 'Threshold', 0.01);
    points{i} = points{i}(1:2,:);
end

matches = siftmatch(descr{1}, descr{2});

% Plot matches.
figure();
plotmatches(I{1}, I{2}, points{1}, points{2}, matches, 'Stacking', 'v');

%% Fit Fundamental matrix and remove outliers.
x1 = points{1}(:, matches(1, :));
x2 = points{2}(:, matches(2, :));
[F, inliers] = ransac_fundamental_matrix(homog(x1), homog(x2), 2.0);

% Plot inliers.
inlier_matches = matches(:, inliers);
figure;
plotmatches(I{1}, I{2}, points{1}, points{2}, inlier_matches, 'Stacking', 'v');

x1 = points{1}(:, inlier_matches(1, :));
x2 = points{2}(:, inlier_matches(2, :));

% vgg_gui_F(Irgb{1}, Irgb{2}, F');

%% Compute candidate camera matrices.

% Camera calibration matrix
K = [2362.12 0 1520.69; 0 2366.12 1006.81; 0 0 1];
scale = 0.3;
H = [scale 0 0; 0 scale 0; 0 0 1];
K = H * K;

% ToDo: Compute the Essential matrix from the Fundamental matrix
E = K'*F*K;

[U,D,V] = svd(E);
W = [ 0, -1, 0;
      1,  0, 0;
      0,  0, 1];
t = U(:,end);

% ToDo: write the camera projection matrix for the first camera
P1 = K*[eye(3), zeros(3,1)];

% ToDo: write the four possible matrices for the second camera

R1 = U*W*V';
if(det(R1) < 0)
    R1 = -R1;
end
R2 = U*W'*V';
if(det(R2) < 0)
    R2 = -R2;
end

t = U(:,end);

Pc2 = {};

Pc2{1} = K*[R1 t];
Pc2{2} = K*[R1 -t];
Pc2{3} = K*[R2 t];
Pc2{4} = K*[R2 -t];

%% HINT: You may get improper rotations; in that case you need to change
%       their sign.
% Let R be a rotation matrix, you may check:
% if det(R) < 0
%     R = -R;
% end

% plot the first camera and the four possible solutions for the second
figure;
plot_camera(P1,w,h,'b',2);
plot_camera(Pc2{1},w,h,'r',2); % wrong matrix
plot_camera(Pc2{2},w,h,'g',2); % wrong matrix
plot_camera(Pc2{3},w,h,'y',2); % OK
plot_camera(Pc2{4},w,h,'c',2); % OK

%% Cheiralty check:

randIndex = randi(size(x1,2));

x1_match = homog(x1(:,randIndex));
x2_match = homog(x2(:,randIndex));
min_error = 10;
%% Method #1
% for p =1:4
%     X_trian = triangulate(x1_match(1:2,:), x2_match(1:2,:), P1, Pc2{p}, [2 2]);
%     X_ri1 = euclid(P1 * X_trian);
%     X_ri2 = euclid(Pc2{p} * X_trian);
%     
%     errors_i1 = pdist([euclid(x1_match)' ; X_ri1']);
%     errors_i2 = pdist([euclid(x2_match)' ; X_ri2']); 
%     errors = [errors_i1 errors_i2];
% 
%     if mean(abs(errors(:))) < min_error
%         min_error =  mean(abs(errors(:)));
%         P_index = p;
%     end
% end
%% Method #2
for p =1:4
    % Obtain the euclidean of the triangulated points (come out of triangulate in homogeneous coord.)
    X_trian = triangulate(x1_match(1:2,:), x2_match(1:2,:), P1, Pc2{p}, [2 2]);
    X_trian_euclid = euclid(X_trian);
    % Get the projection of point X in second image.
    ext_caract = inv(K) * Pc2{p};
    Rp = ext_caract(:,1:3);
    tp = ext_caract(:,4);
    X_prime_euclid = Rp * X_trian_euclid + tp;
    % Compare the Z coordinate to establish wether the point is in front of
    % the camera.
    if (X_trian_euclid(3) > 0 && X_prime_euclid(3) > 0 )
        P_index = p;
    end
    
end

%% Reconstruct structure
% ToDo: Choose a second camera candidate by triangulating a match.


%P2 = triangulate(x1_test(:,i), x2_test(:,i), P1, P2, [2 2]);
P2 = Pc2{P_index};

% Triangulate all matches.
N = size(x1,2);
X = zeros(4,N);
for i = 1:N
    X(:,i) = triangulate(x1(:,i), x2(:,i), P1, P2, [w h]);
    
end



%% Plot with colors
r = interp2(double(Irgb{1}(:,:,1)), x1(1,:), x1(2,:));
g = interp2(double(Irgb{1}(:,:,2)), x1(1,:), x1(2,:));
b = interp2(double(Irgb{1}(:,:,3)), x1(1,:), x1(2,:));
Xe = euclid(X);
figure(6); hold on;
plot_camera(P1,w,h,'-');
plot_camera(P2,w,h,'--');
for i = 1:length(Xe)
    scatter3(Xe(1,i), Xe(3,i), -Xe(2,i), 5^2, [r(i) g(i) b(i)]/255, 'filled');
end;
axis equal;


%% Compute reprojection error.

% ToDo: compute the reprojection errors
%       plot the histogram of reprojection errors, and
%       plot the mean reprojection error

for ii=1:N
    proj_p1(:,ii)=P1*X(:,ii);
    proj_p2(:,ii)=P2*X(:,ii);
    d(ii)=norm(euclid(proj_p1(:,ii))-x1(:,ii))+norm(euclid(proj_p2(:,ii))-x2(:,ii));
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 3. Depth map computation with local methods (SSD)

% Data images: 'scene1.row3.col3.ppm','scene1.row3.col4.ppm'
% Disparity ground truth: 'truedisp.row3.col3.pgm'

% Write a function called 'stereo_computation' that computes the disparity
% between a pair of rectified images using a local method based on a matching cost 
% between two local windows.
% 
% The input parameters are 5:
% - left image
% - right image
% - minimum disparity
% - maximum disparity
% - window size (e.g. a value of 3 indicates a 3x3 window)
% - matching cost (the user may able to choose between SSD and NCC costs)
%
% In this part we ask to implement only the SSD cost
%
% Evaluate the results changing the window size (e.g. 3x3, 9x9, 20x20,
% 30x30) and the matching cost. Comment the results.
%
% Note 1: Use grayscale images
% Note 2: Use 0 as minimum disparity and 16 as the the maximum one.
l_img = rgb2gray(imread('Data/scene1.row3.col3.ppm'));
r_img = rgb2gray(imread('Data/scene1.row3.col4.ppm'));

disp_gt = imread('Data/truedisp.row3.col3.pgm');
figure(31)
imshow(disp_gt);

disparity = stereo_computation(l_img, r_img, 0, 16, 31, 'ssd','none');

figure(32)
imshow(mat2gray(disparity));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 4. OPTIONAL: Depth map computation with local methods (NCC)

% Complete the previous function by adding the implementation of the NCC
% cost.
%
% Evaluate the results changing the window size (e.g. 3x3, 9x9, 20x20,
% 30x30) and the matching cost. Comment the results.
l_img = rgb2gray(imread('Data/scene1.row3.col3.ppm'));
r_img = rgb2gray(imread('Data/scene1.row3.col4.ppm'));

disparity = stereo_computation(l_img, r_img, 0, 16, 31, 'ncc','none');

figure(41)
imshow(mat2gray(disparity));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 5. Depth map computation with local methods

% Data images: '0001_rectified_s.png','0002_rectified_s.png'

% Test the functions implemented in the previous section with the facade
% images. Try different matching costs and window sizes and comment the
% results.
l_img = rgb2gray(imread('Data/0001_rectified_s.png'));
r_img = rgb2gray(imread('Data/0002_rectified_s.png'));

disparity = stereo_computation(l_img, r_img, 0, 16, 11, 'ssd','none');

figure(51)
imshow(mat2gray(disparity));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 6. OPTIONAL: Bilateral weights

% Modify the 'stereo_computation' so that you use bilateral weights (or
% adaptive support weights) in the matching cost of two windows.
% Reference paper: Yoon and Kweon, "Adaptive Support-Weight Approach for Correspondence Search", IEEE PAMI 2006
%
% Comment the results and compare them to the previous results (no weights).
%
% Note: Use grayscale images (the paper uses color images)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 7. OPTIONAL:  Stereo computation with Belief Propagation

% Use the UGM library used in module 2 and implement a  
% stereo computation method that minimizes a simple stereo energy with 
% belief propagation. 
% For example, use an L2 pixel-based data term and 
% the same regularization term you used in module 2. 
% Or pick a stereo paper (based on belief propagation) from the literature 
% and implement it. Pick a simple method or just simplify the method they propose.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 8. OPTIONAL:  Depth computation with Plane Sweeping

% Implement the plane sweeping method explained in class.
