# AER1513-stereo_visual_odom

*The stereo model used is the center model
*The reprojection method is the center point method without calculating
*the shortest legnth segment (i.e. A3 from AER 521 by T.D.Barfoot)
*The solver uses SVD method with scaler weight of 1.0 for initial guess
*Followed by a NLS using the roll, pitch, yaw optimization (ROB 501 A5)


*The stage 1. Data association is the same as before
*The stage 2. State estimation includes the non-linear optimization using
*The closed form result from the SVD method
