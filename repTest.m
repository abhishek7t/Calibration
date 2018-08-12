params = stereo('c1c2');
% sanityCheck(250+13,params.CameraParameters2.ReprojectedPoints(:,:,1),data);
R = params.RotationOfCamera2;
T = params.TranslationOfCamera2;
undistortedPoints1 = undistortPoints(data(13).corners,...
                    params.CameraParameters1);
undistortedPoints2 = undistortPoints(data(13+250).corners,...
                    params.CameraParameters2);

worldPoints = triangulate(undistortedPoints1,undistortedPoints2,params);
% next statement  is correct
% imagePoints = worldToImage(params.CameraParameters1, eye(3), [0 0 0] , worldPoints,'ApplyDistortion', true);
% next statement is correct. #verified
imagePoints = worldToImage(params.CameraParameters2, R, T , worldPoints,'ApplyDistortion', true);
sanityCheck(13 + 250, imagePoints,data);