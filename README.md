# AER1513-stereo_visual_odom

* Use OpenSURF for feature extraction
* The stereo model used is the center model
* The reprojection method is the center point method without calculating the shortest legnth segment (A3 from AER 521 by T.D.Barfoot)
* The solver uses SVD method with scaler weight of 1.0 for initial guess
* Followed by a NLS using the Lie group for roll, pitch, yaw optimization (AER 1513)
