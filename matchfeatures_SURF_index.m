function [matchedPts1, matchedPts2, ord_ind_1, ord_ind_2,err] = matchfeatures_SURF_index(features_1,features_2,thsErr)
% matchfeatures_SURF returns matched features between two images using the
% sum-of-squared-difference (SSD) metric
% Reference: matchSURFfeaturesTB
%
%   mathcfeatures_SURF(features1,features2) computes the SSD between
%   OpenSurf features and then uses a threshold error to create two
%   matrices containing the coordinates of corresponding features
%
%   Inputs:
%   -------
%   features1   - OpenSurf features for image 1 (class: 'struct') reference
%   features2   - OpenSurf features for image 2 (class: 'struct')
%
%   Outputs:
%   -------
%   matchedPts1    - x,y coordinates of matched point in image 1
%                       corresponding to points in image 2
%   matchedPts2    - x,y coordinates of matched points in image 2
%                       corresponding to points in image 1
%
%
%
% Author: Miller Tang
% Date Created: 2019-11-17


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Step 1: Extract the feature descriptors from the input data structure and
% put the descriptors into two matrices
    Feat_des1 = reshape([features_1.descriptor], 64, []);
    Feat_des2 = reshape([features_2.descriptor], 64, []);


% Step 2: Find best matches using sum-of-squared-difference (SSD)
% Define the maximum accceptable error to return points as a match
  %thsErr = 0.05; %%%% threshold of error to accept as a match [change]

% Find the best matches using the sum-of-squared-difference metric
  err=zeros(1,length(features_1));
  cor1=1:length(features_1); 
  cor2=zeros(1,length(features_1));
  % Here is the key step to find matches:
  % For each feature (with 64 directions) detected in image 1, we define an
  % array that repeat that specific feature many times to the same size as
  % the number of features detected in image 2. Every feature in image 2
  % also has 64 directions, and thus we can compute the SSD between the one
  % feature found in image 1 to every feature descriptor in image 2 to
  % obtain an array of error (i.e. SSD). we pick the pair with the lowest
  % SSD and restore the index of feature in image 2 to cor2(i), and also
  % the error term into err(i). We loop through all the feature descriptors
  % found in image 1 to complete the matching process.
  for i=1:length(features_1)
      distance=sum((Feat_des2-repmat(Feat_des1(:,i),[1 length(features_2)])).^2,1);  
      [err(i),cor2(i)]=min(distance); % the 'distance' variable is a row vector of sum of each column (i.e. sum of the squared difference)
  end
  
% Step 3: Reorder the error terms in ascending order
  [err, ind]=sort(err); 
  cor1=cor1(ind); 
  cor2=cor2(ind);
  
% Step 4: Using the threshold defined to determine how many matches are
% below the threshold
  check_thresh = err < thsErr; % return 0 or 1 as boolean
  num_matches = sum(check_thresh); % return a "double" type integer showing how many matches below threshold
 
% Step 5: Restore the matches that are below threshold into new variables
  matchedPts1 = zeros(num_matches,2);
  matchedPts2 = zeros(size(matchedPts1));
  ord_ind_1 = zeros(num_matches,1);
  ord_ind_2 = zeros(num_matches,1);
  
% Step 6: Return the top matches following the SSD order
  for i=1:num_matches
      %c=rand(1,3);
      %plot([features_1(cor1(i)).x features_2(cor2(i)).x+size(I1,2)],[features_1(cor1(i)).y features_2(cor2(i)).y],'-','Color',c)
      %plot([features_1(cor1(i)).x features_2(cor2(i)).x+size(I1,2)],[features_1(cor1(i)).y features_2(cor2(i)).y],'o','Color',c)
      matchedPts1(i,1) = features_1(cor1(i)).x; % First column is x coor.
      matchedPts1(i,2) = features_1(cor1(i)).y; % Second column is y coor.
      ord_ind_1(i,1) = cor1(i); 
      
      matchedPts2(i,1) = features_2(cor2(i)).x; % First column is x coor.
      matchedPts2(i,2) = features_2(cor2(i)).y; % Second column is y coor.
      ord_ind_2(i,1) = cor2(i);
  end

end % end of function