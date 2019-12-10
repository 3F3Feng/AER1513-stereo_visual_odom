% This script is the first attempt for the whole VO pipeline
% Date created: 2019-12-04
% AER 1513 project
% Previous update on: 2019-12-04

% This code is originated at the "init.m" and
% "working_between_frames_assocaition.m" and
% "data_association_between_frame_*.m" and
% "motion_estimation_scalar_SVD.m" and
% "VO_center_SVD_scalar_wt_all_in_ver1" file

% NOTE:
% The stereo model used is the center model
% The reprojection method is the center point method without calculating
% the shortest legnth segment (i.e. A3 from AER 521 by T.D.Barfoot)
% The solver uses SVD method with scaler weight of 1.0 for initial guess
% Followed by a NLS using the roll, pitch, yaw optimization (ROB 501 A5)

% The stage 1. Data association is the same as before
% The stage 2. State estimation includes the non-linear optimization using
% the closed form result from the SVD method

close all;
clear;
clc;
% Add data folder to path in MATLAB
% Enter the image folder directory (manully)
if isunix
    FOLDERDIR_left =  '2011_09_26/2011_09_26_drive_0048_sync/image_00/data';
    FOLDERDIR_right = '2011_09_26/2011_09_26_drive_0048_sync/image_01/data';
else
    FOLDERDIR_left =  '2011_09_26\2011_09_26_drive_0048_sync\image_00\data';
    FOLDERDIR_right = '2011_09_26\2011_09_26_drive_0048_sync\image_01\data';
end

% Count the number of images within the directories
if isunix
    num_imgfile_left = size(dir([FOLDERDIR_left '/*.png']) ,1);
    num_imgfile_right= size(dir([FOLDERDIR_right '/*.png']),1);
else
    num_imgfile_left = size(dir([FOLDERDIR_left '\*.png']) ,1);
    num_imgfile_right= size(dir([FOLDERDIR_right '\*.png']),1);
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%% start the VO process %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Create some empty cell arrays to store features and their corresponding
% indices
    %Features within current frame that are matched and ordered accordingly
    % The values will be in struct type in these two variable 
    within_match_order_fea_left  = cell(1,num_imgfile_left);
    within_match_order_fea_right = cell(1,num_imgfile_right);
    
    % The ordered indices that record the BETWEEN frame feature matches for
    % the features stored in the above two variables
    % At each cell: columns are as follows:
    %    1         2           3          4       
    % cur_left  cur_right  next_left  next_right   
    between_order_indices  = cell(1,num_imgfile_left-1); % one cell less than total # of timesteps
  
    % This cell array stores the matched feature image plane coordinates
    % At each cell: column are as follows:
    %    1        2        3        4        5         6         7        8    
    % x_l_cur  y_l_cur  x_r_cur  y_r_cur  x_l_next  y_l_next  x_r_next  y_r_next
    between_order_match_coor = cell(1,num_imgfile_left-1); % one cell less than total # of timesteps

    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Stage 1 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%% Data Association %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Loop through all the timesteps
for input_index = 1:num_imgfile_left-1 % file name starts from 0

    input_index

% Step 1: Load the two stereo images (png images)
    if isunix
        left_dir = [FOLDERDIR_left '/' num2str(input_index-1,'%010i') '.png'];
        right_dir = [FOLDERDIR_right '/' num2str(input_index-1,'%010i') '.png'];
        left_dir_2 = [FOLDERDIR_left '/' num2str(input_index,'%010i') '.png'];
        right_dir_2 = [FOLDERDIR_right '/' num2str(input_index,'%010i') '.png'];
    else
        left_dir = [FOLDERDIR_left '\' num2str(input_index-1,'%010i') '.png'];
        right_dir = [FOLDERDIR_right '\' num2str(input_index-1,'%010i') '.png'];
        left_dir_2 = [FOLDERDIR_left '\' num2str(input_index,'%010i') '.png'];
        right_dir_2 = [FOLDERDIR_right '\' num2str(input_index,'%010i') '.png'];
    end
    % Img_left and Img_right refer to the left and right images
    Img_left = imread(left_dir);
    Img_right = imread(right_dir);
    Img_left_2 = imread(left_dir_2);
    Img_right_2 = imread(right_dir_2);

% Step 2: Convert the two images to single band grayscale images using the
% RGB bands if it is not grayscale
    if class(Img_left) ~= "uint8" 
        Img_left = rgb2gray(Img_left);
    end
    if class(Img_right) ~= "uint8" 
        Img_right = rgb2gray(Img_right);
    end
    if class(Img_left_2) ~= "uint8" 
        Img_left_2 = rgb2gray(Img_left_2);
    end
    if class(Img_right_2) ~= "uint8" 
        Img_right_2 = rgb2gray(Img_right_2);
    end

%% The following section of code uses the OpenSurf feature detector to find
% out matches within stereo images from the same frame (i.e. same timestep)

% Step 3: Call OpenSurf function to detect SURF feature descriptors
% Define the Option parameters of the OpenSurf function
    %Options.upright = false; %%%% check if needed to be true
    Options.upright = true; %%%% check if needed to be false
    Options.tresh = 0.0001;  %%%% test with other values (defalut: 0.0002)
    % Call the OpenSurf function to detect features
    features_1 = [];    features_2 = [];
    features_3 = [];    features_4 = [];
    features_1 = OpenSurf(Img_left, Options);
    features_2 = OpenSurf(Img_right, Options);
    features_3 = OpenSurf(Img_left_2, Options);
    features_4 = OpenSurf(Img_right_2, Options);

    % To visualize the extracted features on the original image
    %{
    close all;
    PaintSURF(Img_left,features_1);
    PaintSURF(Img_right,features_2);
    PaintSURF(Img_left_2,features_3);
    PaintSURF(Img_right_2,features_4);
    %}

% Step 4: Call the matching function to match the SURF features detected
% from the two images
% Always use the set with less detected features as the reference set
    matchedPts1 = [];   ord_fea_1 = [];
    matchedPts2 = [];   ord_fea_2 = [];
    matchedPts3 = [];   ord_fea_3 = [];
    matchedPts4 = [];   ord_fea_4 = [];   

    thsErr = 0.05; % user defined hyperparameter (e.g. 0.05 or 1, 1 means accepting all features found)
    if length(features_2) > length(features_1)
        [matchedPts1, matchedPts2, ord_fea_1, ord_fea_2, ~] = matchfeatures_SURF(features_1,features_2,thsErr);
    else
        [matchedPts2, matchedPts1, ord_fea_2, ord_fea_1, ~] = matchfeatures_SURF(features_2,features_1,thsErr);
    end
    if length(features_4) > length(features_3)
        [matchedPts3, matchedPts4, ord_fea_3, ord_fea_4,~] = matchfeatures_SURF(features_3,features_4,thsErr);
    else
        [matchedPts4, matchedPts3, ord_fea_4, ord_fea_3,~] = matchfeatures_SURF(features_4,features_3,thsErr);
    end

    % To visualize the extracted features on the original image
    %{
    close;
    % Show both images (horizontally)
    I = zeros([size(Img_left,1) size(Img_left,2)*2 size(Img_left,3)]);
    I(:,1:size(Img_left,2),:)=Img_left; I(:,size(Img_left,2)+1:size(Img_left,2)+size(Img_right,2),:)=Img_right;
    figure, imshow(I/255); hold on;
    % Show the best matches (horizontally)
    if size(features_1,2) > size(features_2,2)
        num_matches = size(features_2,2);
    else
        num_matches = size(features_2,1);
    end
    for i= 1 : 50
        c=rand(1,3);
        plot([matchedPts1(i,1) matchedPts2(i,1)+size(Img_left,2)],[matchedPts1(i,2) matchedPts2(i,2)],'-','Color',c)
        plot([matchedPts1(i,1) matchedPts2(i,1)+size(Img_left,2)],[matchedPts1(i,2) matchedPts2(i,2)],'o','Color',c)
    end
    close;

    % Show both images (vertically)
    I = zeros([size(Img_left,1)*2 size(Img_left,2) size(Img_left,3)]);
    I(1:size(Img_left,1),:,:)=Img_left; I(size(Img_left,1)+1:size(Img_left,1)+size(Img_right,1),:,:)=Img_right;
    figure, imshow(I/255); hold on;
    % Show the best matches (vertically)
    if size(features_1,2) > size(features_2,2)
        num_matches = size(features_2,2);
    else
        num_matches = size(features_2,1);
    end
    for i=500:593
        c=rand(1,3);
        plot([matchedPts1(i,1) matchedPts2(i,1)],[matchedPts1(i,2) matchedPts2(i,2)+size(Img_left,1)],'-','Color',c)
        plot([matchedPts1(i,1) matchedPts2(i,1)],[matchedPts1(i,2) matchedPts2(i,2)+size(Img_left,1)],'o','Color',c)
    end
    close;
    %}

% Step 5: applying criteria to filter out all the matches    
    % Calculate the disparity of the matched points
    % Perform within the current and next time step independently
    for i = 1 : size(matchedPts1,1)
        matchedPts1(i,3) = matchedPts1(i,1) - matchedPts2(i,1);
        %matchedPts2(i,3) = matchedPts1(i,1) - matchedPts2(i,1);
    end
    for i = 1 : size(matchedPts3,1)
        matchedPts3(i,3) = matchedPts3(i,1) - matchedPts4(i,1);
        %matchedPts2(i,3) = matchedPts1(i,1) - matchedPts2(i,1);
    end

    % Criterion 1:
    % remove the pairs of points that have a disparity value smaller than 1 pix
    pos_disparity_cur = matchedPts1(:,3) < 1;
    pos_disparity_next = matchedPts3(:,3) < 1;
    % Criterion 2:
    % remove the pairs of points that have a difference of y coordiantes
    % greater than ver_diff_threst pixels
    ver_diff_threst = 4;
    ver_diff_cur = abs(matchedPts1(:,2) - matchedPts2(:,2)) > ver_diff_threst;
    ver_diff_next = abs(matchedPts3(:,2) - matchedPts4(:,2)) > ver_diff_threst;
    % Criterion 3:
    % remove the pairs of points that are too 'far' away from the camera
    disparity_threst_far = 7; % 7 pixels is approximately 55 m away from the camera, 5 pix -> 78 m, 6 pix -> 65 m 
    % Criterion 4:
    % remove the pairs of points that are too 'close' to the camera
    disparity_threst_close = 65; % 77 pixels is approximately 5 m away from the camera, 6m->65pix, 10m->39 

    disparity_bool_cur = (matchedPts1(:,3) < disparity_threst_far) | (matchedPts1(:,3) > disparity_threst_close);
    disparity_bool_next = (matchedPts3(:,3) < disparity_threst_far) | (matchedPts3(:,3) > disparity_threst_close);

    Cri_remove_cur = pos_disparity_cur | ver_diff_cur | disparity_bool_cur;
    Cri_remove_next = pos_disparity_next | ver_diff_next | disparity_bool_next;

    matchedPts1(Cri_remove_cur,:) = [];
    matchedPts2(Cri_remove_cur,:) = [];
    matchedPts3(Cri_remove_next,:) = [];
    matchedPts4(Cri_remove_next,:) = [];

    ord_fea_1(:,Cri_remove_cur) = [];
    ord_fea_2(:,Cri_remove_cur) = [];
    ord_fea_3(:,Cri_remove_next) = [];
    ord_fea_4(:,Cri_remove_next) = [];

    
    % Stores the matched and ordered features to the storage variables
    within_match_order_fea_left{input_index}  = []; % empty the storage
    within_match_order_fea_right{input_index} = []; % from previous stage
    within_match_order_fea_left{input_index}  = ord_fea_1;
    within_match_order_fea_right{input_index} = ord_fea_2;
    within_match_order_fea_left{input_index+1}  = ord_fea_3;
    within_match_order_fea_right{input_index+1} = ord_fea_4;
    
    % Visualize results
    %{
    % Show both images (vertically)
    I = zeros([size(Img_left,1)*2 size(Img_left,2) size(Img_left,3)]);
    I(1:size(Img_left,1),:,:)=Img_left; I(size(Img_left,1)+1:size(Img_left,1)+size(Img_right,1),:,:)=Img_right;
    figure, imshow(I/255); hold on;
    % Show the best matches (vertically)
    if size(features_1,2) > size(features_2,2)
        num_matches = size(features_2,2);
    else
        num_matches = size(features_2,1);
    end
    for i=1:100
        c=rand(1,3);
        plot([matchedPts1(i,1) matchedPts2(i,1)],[matchedPts1(i,2) matchedPts2(i,2)+size(Img_left,1)],'-','Color',c)
        plot([matchedPts1(i,1) matchedPts2(i,1)],[matchedPts1(i,2) matchedPts2(i,2)+size(Img_left,1)],'o','Color',c)
    end
    close all;
    %}

%% The following section of code looks for matches between two different frames (i.e. current vs. next)
% In this part, the logic is:
% 1. For the left camera image, look for feature matches between the
% current frame and the next frame
% 2. For the right camera image, look for feature matches between the
% current frame and the next frame
% (i.e. the left_t vs left_t+1; right_t vs right_t+1)
% /*In theory, The matched features should have the same ordering for both 
% left and right image pairs*/

% Note: 1, 2 for current left and right, respectively
% 3, 4 for next frame left and right camera, respectively

    % Initialize some useful variables
    bet_Pts1 = []; ind_bet_1 = [];
    bet_Pts2 = []; ind_bet_2 = [];
    bet_Pts3 = []; ind_bet_3 = [];
    bet_Pts4 = []; ind_bet_4 = [];

    thsErr_bet = 0.05;

% To perform match between frame, the idea is as follows:
% 1. We find the feature matches between the left image pair using current
% frame vs next frame. We record the indices of the correspondences wrt the
% input feature structure sequence. The 'matchfeatures_SURF_index' function
% returns the indices of correspondences of the input ordered feature which
% obtained from the stereo (left and right) matching process
% 2. After matching, the sequence record in the ind_bet_1 and ind_bet_3
% variables are the same order for the right image, i.e. ind_bet_2 and
% ind_bet_4, respectively. That is: ind_bet_2 = ind_bet_1, same for 3 and 4
% 3. We loop through every feature in the ind_bet_4 vector array to see if
% the correspondence in the ord_fea_2 (i.e. current right-cam features) has
% the correct index as recorded in ind_bet_2 vector array. If yes, keep;
% not, drop it.
% Note: for step 3, we loop thorugh the one with less unique value because
% more unique value means there must be some redudency in its counter part,
% for example, if ind_bet_4 has more unique values than ind_bet_2, but they
% have the same length, this will mean that there exists redudent entries
% in ind_bet_2 vector, so loop through the shorter one to remove redudency

    % Step 1.
    if length(ord_fea_1) < length(ord_fea_3)
        [bet_Pts1, bet_Pts3, ind_bet_1, ind_bet_3, ~] = matchfeatures_SURF_index(ord_fea_1,ord_fea_3,thsErr_bet);
    else
        [bet_Pts3, bet_Pts1, ind_bet_3, ind_bet_1, ~] = matchfeatures_SURF_index(ord_fea_3,ord_fea_1,thsErr_bet);
    end
    % Step 2.
    % 1 for current left-cam; 2 for current right; 3 next left, 4 next right
    ind_bet_2 = ind_bet_1; % ind_det_ stands for index of correspondence between frames
    ind_bet_4 = ind_bet_3; % ind_det_ stands for index of correspondence between frames

    for i = 1 : length(ind_bet_2) % assign correspondences to variables
        bet_Pts2(i,1) = ord_fea_2(ind_bet_2(i)).x;
        bet_Pts2(i,2) = ord_fea_2(ind_bet_2(i)).y;
        bet_Pts4(i,1) = ord_fea_4(ind_bet_4(i)).x;
        bet_Pts4(i,2) = ord_fea_4(ind_bet_4(i)).y;
    end


    % Step 3. loop through every feature in the array with less unique val.
    if length(unique(ind_bet_4)) < length(unique(ind_bet_2))
        for i = 1: length(ind_bet_4)
            check_2 = [];
            [~,~,~,check_2,~] = matchfeatures_SURF_index(ord_fea_4(ind_bet_4(i)),ord_fea_2,1); % ruturns a scaler value
            match_bet_bool_2(i,1) = ( check_2(1) == ind_bet_2(i) );
        end
        clear check_2;
        match_bet_bool = match_bet_bool_2;
    else
        for i = 1: length(ind_bet_2)
            check_4 = [];
            [~,~,~,check_4,~] = matchfeatures_SURF_index(ord_fea_2(ind_bet_2(i)),ord_fea_4,1); % returns a scalar value
            match_bet_bool_4(i,1) = ( check_4(1) == ind_bet_4(i) );
        end
        clear check_4;
        match_bet_bool = match_bet_bool_4;
    end

    % Now, remove the mis-matched feature corrspondences (type: double)
    ind_bet_1(~match_bet_bool,:) = [];
    ind_bet_2(~match_bet_bool,:) = [];
    ind_bet_3(~match_bet_bool,:) = [];
    ind_bet_4(~match_bet_bool,:) = [];

    % THE FOLLOWING ARE THE BETWEEN FRAME CORRESPONDENCES (type: double)
    bet_Pts1(~match_bet_bool,:) = []; % current left x, y
    bet_Pts2(~match_bet_bool,:) = []; % current right x, y
    bet_Pts3(~match_bet_bool,:) = []; % next left x, y
    bet_Pts4(~match_bet_bool,:) = []; % next right x, y
    
    clear match_bet_bool;
    clear match_bet_bool_4;
    clear match_bet_bool_2;
  
    % store the between frame matched features indices into the storage var
    % 1 for cur_left; 2 for cur_right; 3 for next_left; 4 for next_right
    between_order_indices{input_index}  = [ind_bet_1, ind_bet_2, ...
                                           ind_bet_3, ind_bet_4];
                                       
    % store the between frame matched features coordinates into the storage
    % variable:
    between_order_match_coor{input_index} = [ bet_Pts1, bet_Pts2 ...
                                 bet_Pts3, bet_Pts4  ];


    % Plotting the results between frames
    %{
    close;
    % Show both images (vertically)
    I = zeros([size(Img_left,1)*2 size(Img_left,2) size(Img_left,3)]);
    I(1:size(Img_left,1),:,:)=Img_left; I(size(Img_left,1)+1:size(Img_left,1)+size(Img_left_2,1),:,:)=Img_left_2;
    figure, imshow(I/255); hold on;
    % Show the best matches (vertically)
    for i=1:150
        c=rand(1,3);
        plot([bet_Pts1(i,1) bet_Pts3(i,1)],[bet_Pts1(i,2) bet_Pts3(i,2)+size(Img_left,1)],'-','Color',c)
        plot([bet_Pts1(i,1) bet_Pts3(i,1)],[bet_Pts1(i,2) bet_Pts3(i,2)+size(Img_left,1)],'o','Color',c)
    end
    I = zeros([size(Img_left,1)*2 size(Img_left,2) size(Img_left,3)]);
    I(1:size(Img_left,1),:,:)=Img_right; I(size(Img_left,1)+1:size(Img_left,1)+size(Img_left_2,1),:,:)=Img_right_2;
    figure, imshow(I/255); hold on;
    % Show the best matches (vertically)
    for i=1:150
        c=rand(1,3);
        plot([bet_Pts2(i,1) bet_Pts4(i,1)],[bet_Pts2(i,2) bet_Pts4(i,2)+size(Img_left,1)],'-','Color',c)
        plot([bet_Pts2(i,1) bet_Pts4(i,1)],[bet_Pts2(i,2) bet_Pts4(i,2)+size(Img_left,1)],'o','Color',c)
    end
    close;
    
    %}

    
end % End the for loop for data association    

%%%%%%%%%%%%%%%%%%%%%  End of Data Association %%%%%%%%%%%%%%%%%%%%%%%%%%%%    
%%%%%%%%%%%%%%%%%%%%%%%%%% End of Stage 1 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Stage 2 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%% State Estimation %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Pose estimation

% Note: we are using the center camera model here
% All weights are assumed to be 1 for all features
close all;

% Input camera parameters (from calibration file)
baseline = 0.537;    % baseline [m]
focal = 721.5377;  % focal length [pixel] note: fu = fv
cu = 609.5593; % horiz image centre [pixel]
cv = 172.8540; % vert image centre [pixel]

% Create a cell array to restore all the transformation matrices, Tfi
% stands for Transform matrix from initial to final
% from current to next time step
Tfi = cell(1,num_imgfile_left-1); %one transform less than # timestamp

% Set the world frame (i.e. the inertia frame) to be the first frame
T0 = eye(4); % T0 as the initial condition (identity matrix)
T = T0; % Setting initial condition to be identity

% distance travelled (starts at zero)
d0 = sqrt(T(1:3,4)'*T(1:3,4));  % Setting initial condition to be zero
% distance travelled from current to next time step
d = zeros(1,num_imgfile_left-1); % one travelling dis less than # timestamp

% Loop throguh all the time steps to get the poses
for input_index = 1:num_imgfile_left-1 % file name starts from 0

    input_index
    
    % Read the between frame correspondences coordinates
    bet_Pts1 = between_order_match_coor{input_index}(:,1:2);
    bet_Pts2 = between_order_match_coor{input_index}(:,3:4);
    bet_Pts3 = between_order_match_coor{input_index}(:,5:6);
    bet_Pts4 = between_order_match_coor{input_index}(:,7:8);
    
    % Estimation of motion transformation
    % Step 1: compute the point clouds based on the point correspondences
    num_points = size(bet_Pts1,1);

    % points for current frame (u for horizontal axis, v for vertical axis)
    ul_cur = bet_Pts1(:,1);% u left coordinate for features in current frame
    ur_cur = bet_Pts2(:,1);% u right coordinate for features in current frame
    vl_cur = bet_Pts1(:,2);% v left coordinate for features in current frame
    vr_cur = bet_Pts2(:,2);% v right coordinate for features in current frame

    % points for next frame (u for horizontal axis, v for vertical axis)
    ul_next = bet_Pts3(:,1);% u left coordinate for features in next frame
    ur_next = bet_Pts4(:,1);% u right coordinate for features in next frame
    vl_next = bet_Pts3(:,2);% v left coordinate for features in next frame
    vr_next = bet_Pts4(:,2);% v right coordinate for features in next frame

    % Compute the point clouds for current and next frame (inverse CENTER stereo model)
    Pt_cloud_cur = [( baseline./(ul_cur-ur_cur) ) .* ( ((ul_cur + ur_cur) ./2) - cu ) ...
                    ( baseline./(ul_cur-ur_cur) ) .* ( ((vl_cur + vr_cur) ./2) - cv ) ...
                    baseline .* focal ./ (ul_cur - ur_cur) ]'; % the cloud for current frame
            
    Pt_cloud_next = [( baseline./(ul_next-ur_next) ) .* ( ((ul_next + ur_next) ./2) - cu ) ...
                    ( baseline./(ul_next-ur_next) ) .* ( ((vl_next + vr_next) ./2) - cv ) ...
                    baseline .* focal ./ (ul_next - ur_next) ]'; % the cloud for next frame

% Pose estimation using SVD Point Cloud Alignment (i.e. Scalar-weighted)
                
    % Step 2: Using RANSAC to calculate the motion % {reference: T.Barfoot AER521 A3}
    maxinliers = 0;
    bestinliers = [];
    p1inliers = [];
    p2inliers = [];
    itermax = 100000;
    iter = 0;
    image_sapce_distance = ((ul_cur - ul_next).^2 + (vl_cur - vl_next).^2).^(1/2);
    
    image_dis_20 = image_sapce_distance' > prctile(image_sapce_distance,20);
    image_dis_90 = image_sapce_distance' < prctile(image_sapce_distance,90);
    %while iter < itermax && maxinliers < (1/3) * num_points
    while iter < itermax && maxinliers < 0.5 * num_points

        iter = iter + 1;
            
        % shuffle the points into a random order
        pointorder = randperm(num_points);

        % use the first 3 points to propose a motion for the camera
        [C,r] = compute_motion_SVD( Pt_cloud_cur(:,pointorder(1:3)), Pt_cloud_next(:,pointorder(1:3)) ); 
    
        % compute the Euclidean error on all points and threshold to
        % count inliers
        e = Pt_cloud_next - C*(Pt_cloud_cur - r*ones(1,num_points));            
        reproj = sum(e.*e,1); % here, we ignore the 0.5 before the e^2
        
        image_sapce_distance = ((ul_cur - ul_next).^2 + (vl_cur - vl_next).^2).^(1/2);
        inliers = find(reproj < 0.04 & ...
        image_dis_20 &...
        image_dis_90);
        ninliers = size(inliers,2);
        if ninliers > maxinliers
            maxinliers = ninliers;
            bestinliers = inliers; %pt index of the inliers are stored here
            p1inliers = Pt_cloud_cur(:,inliers);
            p2inliers = Pt_cloud_next(:,inliers);
        end
    end % End of while loop of the RANSAC process  
    
    % RANSAC improvement
    [C,r] = compute_motion_SVD(p1inliers,p2inliers);
    e = Pt_cloud_next - C*(Pt_cloud_cur - r*ones(1,num_points));            
    reproj = sum(e.*e,1); % here, we ignore the 0.5 before the e^2
    inliers = find(reproj < 0.02 & ...
        image_dis_20 &...
        image_dis_90);
    ninliers = size(inliers,2);
    maxinliers = size(inliers,2);
    bestinliers = inliers; %pt index of the inliers are stored here
    p1inliers = Pt_cloud_cur(:,inliers);
    p2inliers = Pt_cloud_next(:,inliers);
    
    
    ransacrcd(:,input_index) = [maxinliers,num_points,iter];
    [maxinliers,num_points,iter] 
    [prctile(reproj,20),median(reproj),prctile(reproj,80)]% print the 80%tile error
    
    % recompute the incremental motion using all the inliers from the
    % best motion hypothesis
    [C,r] = compute_motion_SVD(p1inliers,p2inliers);

    % update global transform
    %T = [ C -C*r; 0 0 0 1]*T; % T is the transform matrix from current to next

% Pose estimation using nonlinear optimization (AER 1513) ...
% This method is an iterative method using Lie group and Lie algebra    
    
    % Define measurement uncertainty (i.e. image plane uncertainty):
    % Assuming: both left and right cameras have the same uncertainty level
    % The image plane uncertainty is one pixel, and are uncorrelated to
    % each other in u and v directions
    Sigma_left =  eye(2);
    Sigma_right = eye(2);
    
    nl_iter = 1000; % stands for number of non-linear iterations (user def.)
    for nl_index = 1 : nl_iter
        % Step 1: to extarct the rotation, translate from the previous best
        % pose estimation
        
        %C = T(1:3,1:3);
        %r = - (C') *T(1:3,4);
        
        % Define the optimization matrices
        nl_A = zeros(6, 6); % variable to store left hand side A matrix
        nl_B = zeros(6, 1); % variable to store RHS b vector
        
        % Step 2: build the cost function terms using the previously best T matrix
        for pt_index = 1 : length(bestinliers)
            % 2.1  Build the covariance matrix for current and next frame pt
            % 2.1.1 Build the Jacobian matrix of the inverse camera model for
            % this point
            % Current frame
            ul_1 = ul_cur(bestinliers(pt_index),1);
            vl_1 = vl_cur(bestinliers(pt_index),1);
            ur_1 = ur_cur(bestinliers(pt_index),1);
            vr_1 = vr_cur(bestinliers(pt_index),1);
       
            % next frame
            ul_2 = ul_next(bestinliers(pt_index),1);
            vl_2 = vl_next(bestinliers(pt_index),1);
            ur_2 = ur_next(bestinliers(pt_index),1);
            vr_2 = vr_next(bestinliers(pt_index),1);
        
            % note: fu = fv, only  one variable stored: focal
            % deri. of inv. stereo cam. model using this pt as operation pt
            G_cur = (baseline / (ul_1 - ur_1)^2) .* ...
                [-ur_1 + cu,        0,      ul_1-cu,       0; ...
                 -((vl_1+vr_1)/2 - cv), (1/2)*(ul_1 - ur_1), (vl_1+vr_1)/2 - cv, (1/2)*(ul_1-ur_1);...
                 -focal,            0,      focal,         0];
            % deri. of inv. stereo cam. model of the next frame correspondence
            G_next = (baseline / (ul_2 - ur_2)^2) .* ...
                [-ur_2 + cu,        0,      ul_2-cu,       0; ...
                 -((vl_2+vr_2)/2 - cv), (1/2)*(ul_2 - ur_2), (vl_2+vr_2)/2 - cv, (1/2)*(ul_2-ur_2);...
                 -focal,            0,      focal,         0];
        
            % 2.1.2 Define the image plane covariance matrix for the operating point
            Sigma_cur = [Sigma_left, zeros(size(Sigma_right)); ...
                         zeros(size(Sigma_left)), Sigma_right];
                 
            Sigma_next = [Sigma_left, zeros(size(Sigma_right)); ...
                         zeros(size(Sigma_left)), Sigma_right];         
        
            % 2.2 compute the combined covariance matrix
            Sigma_combined = (G_next * Sigma_next * G_next' + C * G_cur * Sigma_cur * G_cur' * C')^(-1);
            
            % 2.3 Build the E matrix
            err_cur = C * ( p1inliers(:, pt_index) - r ); % store the transformed pt from current to next frame
            E_mat = [C, -[0, -err_cur(3), err_cur(2); err_cur(3), 0, -err_cur(1); -err_cur(2), err_cur(1), 0] ];
            
            % 2.4 Calculate the error term
            % This is the difference between the coordinates after
            % transforming the cuurent frame pt to next frame
            err_term = p2inliers(:, pt_index) - err_cur; 
            
            % 2.5 Calculate the A and B matrix
            nl_A = nl_A + E_mat' * Sigma_combined * E_mat;
            nl_B = nl_B - E_mat' * Sigma_combined * err_term;
        end % end of for loop to build the A and B matrix
            
        perturbation = nl_A \ nl_B; 
        
        r = r + perturbation(1:3);
        C = (eye(3) - [0, -perturbation(6), perturbation(5); perturbation(6), 0, -perturbation(4); -perturbation(5), perturbation(4), 0]) * C;
              
    end % end of non-linear updating process
    

    % update global transform 
    T = [ C -C*r; 0 0 0 1] * T; % T is the transform matrix from current to next

    
% Recore results and generating plots

    % Store the transformation matrix to the storage variable
    Tfi{input_index} = T;
        
    % update distance travelled
    d(input_index) = sqrt(T(1:3,4)'*T(1:3,4));
    %sqrt(sum(err_term.^2))

    % Plot the output point clouds (transformed and projected)
    %{
    ptcloud_tf= C*(Pt_cloud_cur - r*ones(1,num_points));
    plot3(ptcloud_tf(1,:),ptcloud_tf(2,:),ptcloud_tf(3,:),'bo');
    hold on
    plot3(Pt_cloud_next(1,:),Pt_cloud_next(2,:),Pt_cloud_next(3,:),'ro');
    xlabel('x');
    %}
    
    
    
    % this figure shows the feature tracks that were identified as
    % inliers (green) and outliers (red)
    figure(1)
    clf;
    if isunix
        IL1 = imread([FOLDERDIR_left '/' num2str(input_index-1,'%010i') '.png']);
    else
        IL1 = imread([FOLDERDIR_left '\' num2str(input_index-1,'%010i') '.png']);
    end
    imshow(IL1);
    hold on;
    for k=1:num_points
        set(plot( [ul_cur(k) ul_next(k)], [vl_cur(k) vl_next(k)], 'r-' ), 'LineWidth', 2);
        set(plot( ul_cur(k), vl_cur(k), 'ro' ), 'LineWidth', 1);
        text(ul_next(k), vl_next(k),num2str(reproj(k)));
        text(ul_next(k), vl_next(k),num2str(image_sapce_distance(k)));
    end
    for k=1:maxinliers
        set(plot( [ul_cur(bestinliers(k)) ul_next(bestinliers(k))], [vl_cur(bestinliers(k)) vl_next(bestinliers(k))], 'g-' ), 'LineWidth', 2);
        set(plot( ul_cur(bestinliers(k)), vl_cur(bestinliers(k)), 'go' ), 'LineWidth', 1);
    end
    % draw a 1/3 section lines in the image
        set(plot( [round(1/3*size(IL1,2)) round(1/3*size(IL1,2))], [0 size(IL1,1)], 'b-' ), 'LineWidth', 0.8);
        set(plot( [round(2/3*size(IL1,2)) round(2/3*size(IL1,2))], [0 size(IL1,1)], 'b-' ), 'LineWidth', 0.8);  
    
    % this figure plots the camera reference frame as it moves through
    % the world - try rotating in 3D to see the full motion
    figure(2)
    hold on;
    startaxis = [0.1 0 0 0; 0 0.1 0 0; 0 0 0.1 0; 1 1 1 1];
    curraxis = T\startaxis;
    %rotate from vehicle pov to map view
    ry = [0, 0, 1;
          0, 1, 0;
          -1, 0, 0];%rotate -90 deg around y
    rz = [0, 1, 0;
          -1, 0, 0;
          0, 0, 1];%rotate -90 deg around z
    curraxis = [ry*rz,[0;0;0];0, 0, 0, 1] * curraxis;
    plot3( [curraxis(1,1) curraxis(1,4)], [curraxis(2,1) curraxis(2,4)], [curraxis(3,1) curraxis(3,4)], 'r-' );
    plot3( [curraxis(1,2) curraxis(1,4)], [curraxis(2,2) curraxis(2,4)], [curraxis(3,2) curraxis(3,4)], 'g-' );
    plot3( [curraxis(1,3) curraxis(1,4)], [curraxis(2,3) curraxis(2,4)], [curraxis(3,3) curraxis(3,4)], 'b-' );
    axis equal;
    
end % end of pose estimation loop

    % finish off this figure
    figure(2);
    xlabel('x'); ylabel('y'); zlabel('z');
    title('motion of camera frame');
    if isunix
        FOLDERDIR =  '2011_09_26/2011_09_26_drive_0048_sync/oxts/data';
    else
        FOLDERDIR =  '2011_09_26\2011_09_26_drive_0048_sync\oxts\data';
    end

    if isunix
        num_oxts = size(dir([FOLDERDIR '/*.txt']) ,1);
    else
        num_oxts = size(dir([FOLDERDIR '\*.txt']) ,1);
    end

    oxts = cell(1,num_oxts);

    for index = 1:num_oxts
        if isunix
            oxts{index} = load([FOLDERDIR '/' num2str(index-1,'%010i') '.txt']);
        else
            oxts{index} = load([FOLDERDIR '\' num2str(index-1,'%010i') '.txt']);
        end
    end

    transformation = convertOxtsToPose(oxts);
    startaxis = [0.1 0 0 0; 0 0.1 0 0; 0 0 0.1 0; 1 1 1 1];
    for index = 1:length(transformation)
        curraxis = transformation{index}*startaxis;
        plot3( [curraxis(1,1) curraxis(1,4)], [curraxis(2,1) curraxis(2,4)], [curraxis(3,1) curraxis(3,4)], 'r-' );
        plot3( [curraxis(1,2) curraxis(1,4)], [curraxis(2,2) curraxis(2,4)], [curraxis(3,2) curraxis(3,4)], 'g-' );
        plot3( [curraxis(1,3) curraxis(1,4)], [curraxis(2,3) curraxis(2,4)], [curraxis(3,3) curraxis(3,4)], 'b-' );
    end
    axis equal;

    %print -dpng ass3_motion.png
    
    figure(3);
    plot(ransacrcd(1,:)./ransacrcd(2,:));
    
    E = sqrt(sum((transformation{index}*[0;0;0;1] - [ry*rz,[0;0;0];0, 0, 0, 1]*(Tfi{index-1}\[0;0;0;1])).^2))/d(index-1)
    % To save necessary variables
    % save('0005_workspace.mat', 'between_order_match_coor', ...
    %    'between_order_indices', 'within_match_order_fea_left', ...
    %        'within_match_order_fea_right');
