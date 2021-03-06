function [disparity]=stereo_computation(l_img, r_img, min_disp, max_disp, w_size, match_cost,mode)
% The input parameters are 5:
% - left image
% - right image
% - minimum disparity
% - maximum disparity
% - window size (e.g. a value of 3 indicates a 3x3 window)
% - matching cost (the user may able to choose between SSD and NCC costs)

%For each window of w_size, the SSD is computed btw l_img and r_img, if the
%cost surpases max_disp value, then that window adds a 1 to the disparity
%value, otherwise is zero. The disparity is normalized before exiting.

dif = [];
disparity = zeros(size(l_img));

if(mod(w_size,2) == 0)
    w1 = w_size/2;
    w2 = w_size/2-1;
else
    w1 = floor(w_size/2);
    w2 = floor(w_size/2);
end

% Rotate through all image pixels, excluding borders up to half window size
for i = w1 + 1 : size(l_img,1)- w2
    for j = w1 + 1 : size(l_img,2)- w2
        Ileft = l_img( i - w1 : i + w2, j - w1 : j + w2);
        
        test_val_i = (size(l_img,1) -w2 -max_disp); 
        test_val_j = (size(l_img,2) -w2 -max_disp);
        
        if ((i > w1 + max_disp) && (i <= test_val_i) && (j > w1 + max_disp) && (j <= test_val_j))
            score_old = inf;
            score = 0;
            for dj = min_disp: max_disp
                Iright = r_img( i -w1  :i +w2  , j -w1 +dj :j +w2 +dj);
                if match_cost == 'ssd'    
                    score = ssd(Ileft,Iright);
                    if (score < score_old)
                        score_ssd = dj;
                    end    
                else
                    score_ssd = NaN;
                end
                score_old = score; 
            end
            disparity (i,j)= score;
        end
    end
end
end