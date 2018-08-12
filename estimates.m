function [H3, calibParams, p2, p3] = estimates(data)
    pairs = [3 2; 2 1; 3 4; 4 5; 5 6];
    H3 = cell(1,6);
    H3{1,3} = eye(4);
    p2 = zeros(6,250);
    p3 = cell(1,250);
    calibParams = cell(1,6);
    
    for i = 1 : length(pairs)
        pair = pairs(i,:);
        [inlierPoints,match] = Calibration.getInliersRansac(pair,data);
        [params,~] = Calibration.stereoCalibrate(inlierPoints);
        points3D = Calibration.myTriang(inlierPoints,match, params,pair(1),250);
        
        calibParams{pair(1)} = params.CameraParameters1;
        calibParams{pair(2)} = params.CameraParameters2;
        
        r = params.RotationOfCamera2;
        t = params.TranslationOfCamera2;
        relH = [r' -r'*t';0 0 0 1];
        H3{pair(2)} = H3{pair(1)}*relH;
        loc = Calibration.findLoc(points3D,H3{pair(1)});
        
        for f = 1 : length(match)
            frame = match(f,1) - 250 * (pair(1) - 1);
            p2(pair(1),frame) = 1;
            p2(pair(2),frame) = 1;
            if isempty(p3{frame})
               p3{frame} = loc(frame).points; 
%             else
%                 disp('hi');
            end
        end
    end
end