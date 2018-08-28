classdef Calibration
    
    
    
    methods (Access = public, Static)
       
       %%
        function [worldOrientation,worldLocation,inlierIdx] = estimateCameraPose(imgPoints,points3D,match,...
        cameraParameters)

            % worldOrientation is rotatation matrix for camera to world transformation
            %Orientation of camera in world coordinates, returned as a 3-by-3 matrix
            % worldOrientation = R'
            % worldLocation = -R*t'Location of camera, returned as a 1-by-3 unit vector.

            imagePoints  = [];
            worldPoints = [];
            for i = 1:length(match)
            %     undistortedPoints = inlierPoints1(:,:,i,1);
                undistortedPoints = undistortPoints(imgPoints(:,:,i),...
                    cameraParameters);
                imagePoints = [imagePoints;undistortedPoints];
                worldPoints =[(worldPoints);points3D(match(i)).points];
            end

            [worldOrientation,worldLocation,inlierIdx] = estimateWorldCameraPose(imagePoints,worldPoints,...
            cameraParameters, 'Confidence',99.5,'MaxReprojectionError',.2);
        end
        %%
        function inliers = findInliers(imagePoints,params)
            [~,~,n,~] = size(imagePoints);
            inliers = zeros(n,1);
            for img = 1:n
                undistortedPoints1 = undistortPoints(imagePoints(:,:,img,1),...
                    params.CameraParameters1);
                undistortedPoints2 = undistortPoints(imagePoints(:,:,img,2),...
                    params.CameraParameters2);
                points =triangulate(undistortedPoints1,undistortedPoints2,params);
                
                %find reprojection error
                R = params.RotationOfCamera2;
                T = params.TranslationOfCamera2;
                reproj = worldToImage(params.CameraParameters2, R, T , points,'ApplyDistortion', true);
                reprojError = imagePoints(:,:,img,2) - reproj;
                meanReprojectionError = norm(mean(reprojError));

                
                norms = sum((diff(points) .^ 2),2) .^ .5;
                avg = mean(norms);
                nbr55 = (norms < avg);
                norms = norms(nbr55);
                avg = mean(norms);
                if abs(avg-55)<0.7 && meanReprojectionError < 5
                    inliers(img) = 1;
                end
            end
            inliers = logical(inliers);
        end
        %%
        function loc = findLoc(points3D,H)
            np = struct;
            for j = 1 : length(points3D)
                points = points3D(j).points;

                if isempty(points)
                    np(j).points = [];
                else
%                     np(j).points = [points ones(length(points), 1)] * [H(1:3,1:3)';H(1:3,4)'] ;
                    np(j).points = [points ones(length(points), 1)] * H' ;
                    np(j).points = np(j).points(:,1:3);
                end
            end
            loc = np;
        end
        %%
        % Takes as input the two ranges of images to compare and the corner points extracted.
        % Returns a logical array of matching pairs

        function matches = findMatchingCorners(set1, set2, data)

            numImages = min(set1(2)-set1(1)+1,set2(2)-set2(1)+1);
            matches = zeros(numImages,2);
            
            for i = 0:numImages-1
                idx1 = set1(1)+i;
                idx2 = set2(1)+i;
                [size1,~] = size(data(idx1).corners);
                [size2,~] = size(data(idx2).corners);
                if(size1 + size2 == 198)
                   matches(i+1,:) = [idx1 idx2];
                end
            end
            
            logicalMatches = logical(matches);
            pairs = [matches(logicalMatches(:,1),1) matches(logicalMatches(:,2),2)];
            matches = pairs;
        end
        
       %%  returns the range of images in data given the camera number
        function range = findRange(i)
             iCor = [2*(i-7) 2*(i-6)];

            if i<7
                iCor = [0 0];
            end

            range = [250*(i-1)+1+iCor(1) 250*(i)+iCor(2)];
        end
        %% returns distorted image points (unchanged image points)
        function [inlierPoints,match] = getInliersRansac(cams,data)
            boardSize = [10 12];
            squareSize = 55;
            imageSize = [1080,1920];
            worldPoints = generateCheckerboardPoints(boardSize,squareSize);
        

            match = Calibration.findMatchingCorners(Calibration.findRange(cams(1)), Calibration.findRange(cams(2)), data);
            n = length(match);
            imagePoints = zeros(99,2,n,2);
            for i = 1:n
                imagePoints(:,:,i,1) = data(match(i,1)).corners;
                imagePoints(:,:,i,2) = data(match(i,2)).corners;
            end
            
            inlierLogical = [];
            for i = 1:170
                sample = randperm(n,4);
                samplePoints = imagePoints(:,:,sample',:);

                [params, ~, ~] = estimateCameraParameters(samplePoints,worldPoints, ...
                                              'ImageSize',imageSize,'WorldUnits','mm',...
                                                  'NumRadialDistortionCoefficients',3,...
                                                  'EstimateTangentialDistortion',false);
                logical = Calibration.findInliers(imagePoints,params);
                if sum(logical)>sum(inlierLogical)
%                     disp(i)
                    inlierLogical = logical;
                    if sum(logical)>.8*length(match)
                        break
                     end
                end

            end
            inlierPoints = imagePoints(:,:,inlierLogical ,:);
            match = match(inlierLogical,:);
        end
        %%
        function points3D = myTriang(imagePoints,match, params,camNum,numFrames)
            %returns point position wrt the first camera
            points3D = struct;
            [~,~,m,~ ] = size(imagePoints);
            for i = 1:m
                j = match(i,1)-numFrames*(camNum-1);
                if camNum > 6
                    j = match(i,1) - 250*6 -numFrames*(camNum-7);
                end
         
                undistortedPoints1 = undistortPoints(imagePoints(:,:,i,1),...
                    params.CameraParameters1);
                undistortedPoints2 = undistortPoints(imagePoints(:,:,i,2),...
                    params.CameraParameters2);

                points3D(j).points = triangulate(undistortedPoints1,undistortedPoints2,params);
            end

        end
        %%
        function [] = PlotCamerasAndPoints( Cset, Rset, X, scale )
            % scale = 25
            % Visualizes the camera 
            % Inputs: 
            %   Cset{i}: C for ith camera
            %   Rset{i}:  R for ith camera
            %   X: point cloud.
            %   scale: scaling factor to approximate camera distances (e.g. 25)

            % Colors that each camera will be rendered as. For more than 6 cameras,
            % modify this.
            Colors = ['k', 'b', 'g' , 'm', 'c', 'r', 'b', 'g' , 'm', 'c', 'r'];

            figure;
            hold on; grid on

            for i = 1 : size(Cset,2)
                % Show the ith camera
                plotCamera('Location',Cset{i}*scale,'Orientation',Rset{i},'Opacity',0, 'Color', Colors(i));
            end

            % Show the correspondence points.
            scatter3(X(:,1)*scale, X(:,2)*scale, X(:,3)*scale, [Colors(2), '*']);  

            % Set the limits so that your pointcloud can be seen.
            % xlim([-15,15*scale]);
            % ylim([-15,15*scale]);
            % zlim([-15,15*scale]);

            view(360,0);
            axis square;
            axis equal;
            hold off
        end
        %%
        function err = reprojError(X,x,R,T,K)
            n = length(X);
            M = K*[R T];
            reproj = M * [X ones(n,1)]';
            reproj = reproj ./ [reproj(3,:);reproj(3,:);reproj(3,:)];
            error = [x ones(n,1)] - reproj';
            meanReprojectionError = mean(error,1);
            err = norm(meanReprojectionError);
        end
        
        %% @ input R and T are as output by matlab stereo estimateCameraParameters function
        % M = K * [R'  T] camera matrix
        % returns undistorted reprojected image points
        function reproj = findReproj(X,R,T,K)
            [n,~] = size(X);
            if length(T(1,:)) == 3
                T = T';
            end
            if n == 0
                ME = MException('VerifyOutput:OutOfBounds', ...
             'empty X matrix');
                throw(ME);
            end
            M = K*[R' T];
            reproj = M * [X ones(n,1)]';
            reproj = reproj ./ [reproj(3,:);reproj(3,:);reproj(3,:)];
            reproj = reproj(1:2,:)';
        end
        %%
        function [params,estimationErrors] = stereoCalibrate(inlierPoints,K1, K2, rad_dist1,rad_dist2)
            boardSize = [10 12];
            squareSize = 55;
            worldPoints = generateCheckerboardPoints(boardSize,squareSize);
            imageSize = [1080,1920];

            [params,~,estimationErrors] = estimateCameraParam(inlierPoints,worldPoints, ...
                                              'ImageSize',imageSize,'WorldUnits','mm',...
                                              'NumRadialDistortionCoefficients',3,...
                                              'EstimateTangentialDistortion',false...
                                              ,'InitialIntrinsicMatrix1',K1,'InitialRadialDistortion1',rad_dist1...
                                              ,'InitialIntrinsicMatrix2',K2,'InitialRadialDistortion2',rad_dist2);
            
%             [params,~,estimationErrors] = estimateCameraParameters(inlierPoints,worldPoints, ...
%                                               'ImageSize',imageSize,'WorldUnits','mm',...
%                                               'NumRadialDistortionCoefficients',3,...
%                                               'EstimateTangentialDistortion',false...
%                                             );
        end
        
        %% find frame number from point index
        function frame = findFrame(index)
           frame = floor((index - 1)/99) + 1; 
        end
        
    end
end