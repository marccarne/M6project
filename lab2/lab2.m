%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Lab 2: Image mosaics

addpath('sift');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 1. Compute image correspondences

%% Open images

% imargb = imread('Data/llanes/llanes_a.jpg');
% imbrgb = imread('Data/llanes/llanes_b.jpg');
% imcrgb = imread('Data/llanes/llanes_c.jpg');

% imargb = imread('Data/castle_int/0016_s.png');
% imbrgb = imread('Data/castle_int/0015_s.png');
% imcrgb = imread('Data/castle_int/0014_s.png');

% imargb = imread('Data/aerial/site13/frame00000.png');
% imbrgb = imread('Data/aerial/site13/frame00002.png');
% imcrgb = imread('Data/aerial/site13/frame00003.png');
% % 
% ima = sum(double(imargb), 3) / 3 / 255;
% imb = sum(double(imbrgb), 3) / 3 / 255;
% imc = sum(double(imcrgb), 3) / 3 / 255;

imargb = double(imread('Data/aerial/site22/frame_00001.tif'));
imbrgb = double(imread('Data/aerial/site22/frame_00018.tif'));
imcrgb = double(imread('Data/aerial/site22/frame_00030.tif'));
ima = imargb;
imb = imbrgb;
imc = imcrgb;

%% Compute SIFT keypoints
[points_a, desc_a] = sift(ima, 'Threshold', 0.01);
[points_b, desc_b] = sift(imb, 'Threshold', 0.01);
[points_c, desc_c] = sift(imc, 'Threshold', 0.01);

% figure;
% imshow(imargb);%image(imargb)
% hold on;
% plot(points_a(1,:), points_a(2,:),'+y');
% figure;
% imshow(imbrgb);%image(imbrgb);
% hold on;
% plot(points_b(1,:), points_b(2,:),'+y');
% figure;
% imshow(imcrgb);%image(imcrgb);
% hold on;
% plot(points_c(1,:), points_c(2,:),'+y');

%% Match SIFT keypoints
%b is the central image
% between a and b
matches_ab = siftmatch(desc_a, desc_b);
figure;
plotmatches(ima, imb, points_a(1:2,:), points_b(1:2,:), matches_ab, 'Stacking', 'v');

% between b and c
matches_bc = siftmatch(desc_b, desc_c);
figure;
plotmatches(imb, imc, points_b(1:2,:), points_c(1:2,:), matches_bc, 'Stacking', 'v');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 2. Compute the homography (DLT algorithm) between image pairs

%% Compute homography (normalized DLT) between a and b, play with the homography
th = 3;
xab_a = [points_a(1:2, matches_ab(1,:)); ones(1, length(matches_ab))];
xab_b = [points_b(1:2, matches_ab(2,:)); ones(1, length(matches_ab))];

% % Points normalization
% [xab_a Taba] = normalizepts( xab_a );
% [xab_b Tabb] = normalizepts( xab_b );

[Hab, inliers_ab] = ransac_homography_adaptive_loop(xab_a, xab_b, th, 1000); % ToDo: complete this function

% % De normalization of points
% xab_a = inv(Taba)* xab_a;
% xab_b = inv(Tabb)* xab_b;

% figure(1);
% plotmatches(ima, imb, points_a(1:2,:), points_b(1:2,:), ...
%     matches_ab(:,inliers_ab), 'Stacking', 'v');

vgg_gui_H(imargb, imbrgb, Hab);


%% Compute homography (normalized DLT) between b and c, play with the homography
xbc_b = [points_b(1:2, matches_bc(1,:)); ones(1, length(matches_bc))];
xbc_c = [points_c(1:2, matches_bc(2,:)); ones(1, length(matches_bc))];

% % Points normalization
% [xbc_b Tbcb] = normalizepts( xbc_b );
% [xbc_c Tbcc] = normalizepts( xbc_c );

[Hbc, inliers_bc] = ransac_homography_adaptive_loop(xbc_b, xbc_c, th, 1000);  %Inverted if we take 'b' as the reference

% % De normalization of points
% xbc_b = inv(Tbcb)* xbc_b;
% xbc_c = inv(Tbcc)* xbc_c;

% figure(2);
% plotmatches(imb, imc, points_b(1:2,:), points_c(1:2,:), ...
%     matches_bc(:,inliers_bc), 'Stacking', 'v');

vgg_gui_H(imbrgb, imcrgb, Hbc);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 3. Build the mosaic

corners = [-400 1200 -100 650]; %[-500 1600 -50 1000]; %
iwb = apply_H_v2(imbrgb, eye(3) , corners);   % ToDo: complete the call to the function
iwa = apply_H_v2(imargb, Hab, corners); %eye=identity matrix   % ToDo: complete the call to the function
iwc = apply_H_v2(imcrgb, inv(Hbc), corners);    % ToDo: complete the call to the function

% figure;
% imshow(max(iwc, max(iwb, iwa)));%image(max(iwc, max(iwb, iwa)));axis off;
% title('Mosaic A-B-C');

% ToDo: compute the mosaic with castle_int images
% ToDo: compute the mosaic with aerial images set 13
% ToDo: compute the mosaic with aerial images set 22
% ToDo: comment the results in every of the four cases: say why it works or
%       does not work

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 4. Refine the homography with the Gold Standard algorithm

% Homography ab

%Take the inlier points
points_to_perform_x = xab_a(:,inliers_ab);
points_to_perform_xp = xab_b(:,inliers_ab);

%Transform to non-homogeneous points
x = euclid(points_to_perform_x);  %ToDo: set the non-homogeneous point coordinates of the 
xp = euclid(points_to_perform_xp); %      point correspondences we will refine with the geometric method

% x = points_a(1:2, matches_ab(1,:));  %ToDo: set the non-homogeneous point coordinates of the 
% xp = points_b(1:2, matches_ab(2,:)); %      point correspondences we will refine with the geometric method

Xobs = [ x(:) ; xp(:) ];     % The column vector of observed values (x and x')
P0 = [ Hab(:) ; x(:) ];      % The parameters or independent variables

Y_initial = gs_errfunction( P0, Xobs ); % ToDo: create this function that we need to pass to the lsqnonlin function
% NOTE: gs_errfunction should return E(X) and not the sum-of-squares E=sum(E(X).^2)) that we want to minimize. 
% (E(X) is summed and squared implicitly in the lsqnonlin algorithm.) 
err_initial = sum( sum( Y_initial.^2 ));

options = optimset('Algorithm', 'levenberg-marquardt');
P = lsqnonlin(@(t) gs_errfunction(t, Xobs), P0, [], [], options);

Hab_r = reshape( P(1:9), 3, 3 );
f = gs_errfunction( P, Xobs ); % lsqnonlin does not return f
err_final = sum( sum( f.^2 ));

% we show the geometric error before and after the refinement
fprintf(1, 'Gold standard reproj error initial %f, final %f\n', err_initial, err_final);


%% See differences in the keypoint locations

% ToDo: compute the points xhat and xhatp which are the correspondences
% returned by the refinement with the Gold Standard algorithm
xhat = reshape(P(10:length(P)), 2, ((length(P)-9)/2));
xhat_h = [xhat; ones(1,size(xhat,2))];
xhatp_h = Hab_r*xhat_h;
xhatp = xhatp_h(1:2,:) ./ xhatp_h(3,:);

figure;
imshow(imargb);%image(imargb);
hold on;
plot(x(1,:), x(2,:),'+y');
plot(xhat(1,:), xhat(2,:),'+c');

figure;
imshow(imbrgb);%image(imbrgb);
hold on;
plot(xp(1,:), xp(2,:),'+y');
plot(xhatp(1,:), xhatp(2,:),'+c');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  Homography bc

% ToDo: refine the homography bc with the Gold Standard algorithm

%Take the inlier points
points_to_perform_x = xbc_b(:,inliers_bc);
points_to_perform_xp = xbc_c(:,inliers_bc);

%Transform to non-homogeneous points
x = euclid(points_to_perform_x);  %ToDo: set the non-homogeneous point coordinates of the 
xp = euclid(points_to_perform_xp); %      point correspondences we will refine with the geometric method

% x = points_b(1:2, matches_bc(1,:));  %ToDo: set the non-homogeneous point coordinates of the 
% xp = points_c(1:2, matches_bc(2,:)); %      point correspondences we will refine with the geometric method

Xobs = [ x(:) ; xp(:) ];     % The column vector of observed values (x and x')
P0 = [ Hbc(:) ; x(:) ];      % The parameters or independent variables

Y_initial = gs_errfunction( P0, Xobs ); % ToDo: create this function that we need to pass to the lsqnonlin function
% NOTE: gs_errfunction should return E(X) and not the sum-of-squares E=sum(E(X).^2)) that we want to minimize. 
% (E(X) is summed and squared implicitly in the lsqnonlin algorithm.) 
err_initial = sum( sum( Y_initial.^2 ));

options = optimset('Algorithm', 'levenberg-marquardt');
P = lsqnonlin(@(t) gs_errfunction(t, Xobs), P0, [], [], options);

Hbc_r = reshape( P(1:9), 3, 3 );
f = gs_errfunction( P, Xobs ); % lsqnonlin does not return f
err_final = sum( sum( f.^2 ));

% we show the geometric error before and after the refinement
fprintf(1, 'Gold standard reproj error initial %f, final %f\n', err_initial, err_final);


%% See differences in the keypoint locations

% ToDo: compute the points xhat and xhatp which are the correspondences
% returned by the refinement with the Gold Standard algorithm
xhat = reshape(P(10:length(P)), 2, ((length(P)-9)/2));
xhat_h = [xhat; ones(1,size(xhat,2))];
xhatp_h = Hbc_r*xhat_h;
xhatp = xhatp_h(1:2,:) ./ xhatp_h(3,:);

figure;
imshow(imbrgb);%image(imbrgb);
hold on;
plot(x(1,:), x(2,:),'+y');
plot(xhat(1,:), xhat(2,:),'+c');

figure;
imshow(imcrgb);%image(imcrgb);
hold on;
plot(xp(1,:), xp(2,:),'+y');
plot(xhatp(1,:), xhatp(2,:),'+c');

%% Build mosaic
corners = [-400 1200 -100 650];
iwb = apply_H_v2(imbrgb, eye(3), corners); % ToDo: complete the call to the function
iwa = apply_H_v2(imargb, Hab_r, corners); % ToDo: complete the call to the function
iwc = apply_H_v2(imcrgb, inv(Hbc_r), corners); % ToDo: complete the call to the function

figure;
imshow(max(iwc, max(iwb, iwa)));%image(max(iwc, max(iwb, iwa)));axis off;
title('Mosaic A-B-C');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 5. OPTIONAL: Calibration with a planar pattern

clear all;

%% Read template and images.
T     = imread('Data/calib/template.jpg');
I{1}  = imread('Data/calib/graffiti1.tif');
I{2}  = imread('Data/calib/graffiti2.tif');
I{3}  = imread('Data/calib/graffiti3.tif');
I{4}  = imread('Data/calib/graffiti4.tif');
I{5}  = imread('Data/calib/graffiti5.tif');
Tg = sum(double(T), 3) / 3 / 255;
Ig{1} = sum(double(I{1}), 3) / 3 / 255;
Ig{2} = sum(double(I{2}), 3) / 3 / 255;
Ig{3} = sum(double(I{3}), 3) / 3 / 255;
Ig{4} = sum(double(I{4}), 3) / 3 / 255;
Ig{5} = sum(double(I{5}), 3) / 3 / 255;

N = length(I);

    %% Compute keypoints.
fprintf('Computing sift points in template... ');
[pointsT, descrT] = sift(Tg, 'Threshold', 0.05);
fprintf(' done\n');

points = cell(N,1);
descr = cell(N,1);
for i = 1:N
    fprintf('Computing sift points in image %d... ', i);
    [points{i}, descr{i}] = sift(Ig{i}, 'Threshold', 0.05);
    fprintf(' done\n');
end

%% Match and compute homographies.
H = cell(N,1);
for i = 1:N
    % Match against template descriptors.
    fprintf('Matching image %d... ', i);
    matches = siftmatch(descrT, descr{i});
    fprintf('done\n');

    % Fit homography and remove outliers.
    x1 = pointsT(1:2, matches(1, :));
    x2 = points{i}(1:2, matches(2, :));
    H{i} = 0;
    [H{i}, inliers] =  ransac_homography_adaptive_loop(homogeneous(x1), homogeneous(x2), 3, 1000);

    % Plot inliers.
    figure;
    plotmatches(Tg, Ig{i}, pointsT(1:2,:), points{i}(1:2,:), matches(:, inliers));

    % Play with the homography
    %vgg_gui_H(T, I{i}, H{i});
end

%% Compute the Image of the Absolute Conic

% We compute the matrix V from the homographies:

for n = 1:N
    % Get the corresponding homography
    h = H{n};
    % Store the columns here for convenience
    h1 = h(:,1);
    h2 = h(:,2);
    % First equation of the two provided by a planar pattern: V12_t*Omega = 0
    C((2*n-1),:) = [h1(1) * h2(1),
                    h1(1) * h2(2) + h1(2) * h2(1),
                    h1(1) * h2(3) + h1(3) * h2(1),
                    h1(2) * h2(2),
                    h1(2) * h2(3) + h1(3) * h2(2),
                    h1(3) * h2(3)];
    % Second equation. (V11-V22)_t * Omega = 0
    v11 = [h1(1) * h1(1),
           h1(1) * h1(2) + h1(2) * h1(1),
           h1(1) * h1(3) + h1(3) * h1(1),
           h1(2) * h1(2),
           h1(2) * h1(3) + h1(3) * h1(2),
           h1(3) * h1(3)];
       
    v22 = [h2(1) * h2(1),
           h2(1) * h2(2) + h2(2) * h2(1),
           h2(1) * h2(3) + h2(3) * h2(1),
           h2(2) * h2(2),
           h2(2) * h2(3) + h2(3) * h2(2),
           h2(3) * h2(3)];
    C(2*n,:) = v11-v22;
end

%% SVD decomposition. w is the last columnv of V transposed
[U D V] = svd(C);

W = V(:,end);

w = [W(1), W(2), W(3);
     W(2), W(4), W(5);
     W(3), W(5), W(6)];
 
%% Recover the camera calibration.
% Apply cholesky factorization of w:
% w = inv(K_t * K) --> then: inv(w) = (K_t * K)

K = chol(inv(w));
    
% ToDo: in the report make some comments related to the obtained internal
%       camera parameters and also comment their relation to the image size

%% Compute camera position and orientation.
R = cell(N,1);
t = cell(N,1);
P = cell(N,1);
figure;hold;
for i = 1:N
    % ToDo: compute r1, r2, and t{i}
    h = H{i};
    r1 = inv(K) * h(:,1);% / norm(inv(K) * h(:,1));
    r2 = inv(K) * h(:,2);%/ norm(inv(K) * h(:,2));
    t{i} = inv(K) * h(:,3);%/ norm(inv(K) * h(:,1));
    
    % Solve the scale ambiguity by forcing r1 and r2 to be unit vectors.
    s = sqrt(norm(r1) * norm(r2)) * sign(t{i}(3));
    r1 = r1 / s;
    r2 = r2 / s;
    t{i} = t{i} / s;
    R{i} = [r1, r2, cross(r1,r2)];
    
    % Ensure R is a rotation matrix
    [U S V] = svd(R{i});
    R{i} = U * eye(3) * V';
   
    P{i} = K * [R{i} t{i}];
    plot_camera(P{i}, 800, 600, 200);
end
%%
% ToDo: in the report explain how the optical center is computed in the
%       provided code

[ny,nx] = size(T);
p1 = [0 0 0]';
p2 = [nx 0 0]';
p3 = [nx ny 0]';
p4 = [0 ny 0]';
% Draw planar pattern
vgg_scatter_plot([p1 p2 p3 p4 p1], 'g');
% Paint image texture
surface('XData',[0 nx; 0 nx],'YData',[0 0; 0 0],'ZData',[0 0; -ny -ny],'CData',T,'FaceColor','texturemap');
colormap(gray);
axis equal;

%% Plot a static camera with moving calibration pattern.
figure; hold;
plot_camera(K * eye(3,4), 800, 600, 200);
% ToDo: complete the call to the following function with the proper
%       coordinates of the image corners in the new reference system
for i = 1:N
    p1 = [R{i} t{i}] * homogeneous([0 0 0]');
    p2 = [R{i} t{i}] * homogeneous([nx 0 0]');
    p3 = [R{i} t{i}] * homogeneous([nx ny 0]');
    p4 = [R{i} t{i}] * homogeneous([0 ny 0]');
    vgg_scatter_plot( [p1 p2 p3 p4 p1], 'r');
end

%% Augmented reality: Plot some 3D points on every camera.
[Th, Tw] = size(Tg);
cube = [0 0 0; 1 0 0; 1 0 0; 1 1 0; 1 1 0; 0 1 0; 0 1 0; 0 0 0; 0 0 1; 1 0 1; 1 0 1; 1 1 1; 1 1 1; 0 1 1; 0 1 1; 0 0 1; 0 0 0; 1 0 0; 1 0 0; 1 0 1; 1 0 1; 0 0 1; 0 0 1; 0 0 0; 0 1 0; 1 1 0; 1 1 0; 1 1 1; 1 1 1; 0 1 1; 0 1 1; 0 1 0; 0 0 0; 0 1 0; 0 1 0; 0 1 1; 0 1 1; 0 0 1; 0 0 1; 0 0 0; 1 0 0; 1 1 0; 1 1 0; 1 1 1; 1 1 1; 1 0 1; 1 0 1; 1 0 0 ]';
%pyramid = [0 0 0; 1 0 0; cos(60*pi/180) sin(60*pi/180) 0; 0 0 0; cos(45*pi/180)/3 sin(45*pi/180)/3 1; (1 + cos(45*pi/180))/3 sin(45*pi/180)/3 1; 1 0 0; (1 + cos(45*pi/180))/3 sin(45*pi/180)/3 1;(cos(60*pi/180) + cos(45*pi/180))/3 (sin(60*pi/180) + sin(45*pi/180))/3 1;cos(60*pi/180) sin(60*pi/180)  0; (cos(60*pi/180) + cos(45*pi/180))/3 (sin(60*pi/180) + sin(45*pi/180))/3 1;cos(45*pi/180)/3 sin(45*pi/180)/3 1]';

% Our simple object, a very original pyramid
pyramid = [0 0 0; 1 0 0; 0.5 1 0; 0 0 0; 0.5 0.5 1;1 0 0; 0.5 0.5 1; 0.5 1 0]';

X = (cube - .5) * Tw / 4 + repmat([Tw / 2; Th / 2; -Tw / 8], 1, length(cube));
Y = (pyramid - .5) * Tw / 4 + repmat([Tw / 2; Th / 2; -Tw / 8], 1, length(pyramid));
for i = 1:N
    figure; colormap(gray);
    imagesc(Ig{i});
    hold on;
    x = euclid(P{i} * homogeneous(X));
    y = euclid(P{i} * homogeneous(Y));
    vgg_scatter_plot(x, 'g');
    %vgg_scatter_plot(y, 'r');
end

% ToDo: change the virtual object, use another 3D simple geometric object like a pyramid

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 6. OPTIONAL: Add a logo to an image using the DLT algorithm

% Handcrafted points to place our logo in a single "cell" on the graffiti1
% image. One of the cells of the wall, it's hard to see. 
ImageToPaint = imread('Data/calib/graffiti1.tif');
[logo,map,alpha] = imread('logo.png');
logo = imresize(logo,0.5);
alpha = imresize(alpha,0.5);

% Defined selecting a square on the image, clockwise, starting on the top
% left corner

pointsImage = [ 358 756 1; 367 1111 1; 643 746 1; 657 1094 1]';
[rows,cols,~] = size(logo);
pointsLogo = [0 0 1;0 cols 1;rows 0 1; rows cols 1]';

H = homography2d(pointsLogo,pointsImage);

% Paint our logo:
transformedLogo = apply_H2(logo,H);
transformedAlpha = apply_H2(alpha,H);

origin = pointsImage(1:2,1);
[rows,cols,~] = size(transformedLogo);

for row = 1:rows
    for col = 1:cols
        if (transformedAlpha(row,col,1) ~= 0)
            ImageToPaint(origin(1) + row, origin(2) + col,:) = transformedLogo(row,col,:);
        end
    end
end

figure();imagesc(ImageToPaint);
hold on;
ImageSquare = [pointsImage(1:2,1:2),pointsImage(1:2,4),pointsImage(1:2,3), pointsImage(1:2,1)];
plot(ImageSquare(2,:),ImageSquare(1,:),'g');

