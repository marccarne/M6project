function disparity = stereo_computation(l_img, r_img, mindisp, maxdisp, wsize, cost, mode)
[h w]=size(l_img);

% The depth map has to start at 1 because MATLAB
depth_map = zeros(h,w,1+maxdisp);
disp  = zeros(h,w);

%kernel for convolution. It saves a lot of time and effort.
kernel = ones(wsize);                      

if strcmp(mode, 'none')
    weights = 1/(wsize^2);
elseif strcmp(mode, 'bilateral') 
    %ToDo
end    
% We compute the sumatory of the regions windowed at each pixel for later,
% in case we use ncc 
if strcmp(cost, 'ncc')
    left = conv2(double(l_img),double(kernel),'same')*weights;
    sigmaLeft = sqrt(conv2((double(l_img) - left).^2,kernel,'same'));
    right = conv2(double(r_img),double(kernel),'same')*weights;
    sigmaRight = sqrt(conv2((double(r_img) - right).^2,kernel,'same'));
end

for d=mindisp:maxdisp

  % First index we take is the first row of the d-th column. 
  idx = h*d+1:h*w; 

  if strcmp(cost, 'ssd')
    % if idx points to a pixel, idx - h*d points to the pixel d columns
    % before, which is the disparity measure.
    disp(idx) = (l_img(idx) - r_img(idx - h*d)).^2;
    depth_map(:,:,d+1) = conv2(disp,kernel,'same');
  elseif strcmp(cost, 'ncc')
    disp(idx) = ((double(l_img(idx)) - left(idx)) .* (double(r_img(idx - h*d)) - right(idx - h*d)))./(sigmaLeft(idx) .* sigmaRight(idx-h*d));
    % Correlation: high scores equal to matches. 
    depth_map(:,:,d+1) = 1 -conv2(disp,kernel,'same');
  else
    errordlg('Error, wrong cost model input');  
  end    
end


% figure();
% imagesc([depth_map(:,:,1),depth_map(:,:,2),depth_map(:,:,3),depth_map(:,:,4);
%          depth_map(:,:,5),depth_map(:,:,6),depth_map(:,:,7),depth_map(:,:,8);
%          depth_map(:,:,9),depth_map(:,:,10),depth_map(:,:,11),depth_map(:,:,12);
%          depth_map(:,:,13),depth_map(:,:,14),depth_map(:,:,15),depth_map(:,:,16);]);
% colormap(gray);
%      
% We get the disparity that got the minimal cost value
% if strcmp(cost, 'ssd')
%     [val,ind]=min(depth_map,[],3);
% elseif strcmp(cost, 'ncc')
%     [val,ind]=max(depth_map,[],3);
% end
     [val,ind]=min(depth_map,[],3);
% We fix this because MATLAB is a horrible language and it needs constant
% fixes to make sense

disparity = ind-1;
