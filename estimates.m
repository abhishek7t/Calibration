function [H3, calibParams, p2, p3, matchSet, points3DSet, HSet] = estimates(data,calib)
    pairs = [3 2; 2 1; 3 4; 4 5; 5 6];
    H3 = cell(1,6);
    H3{1,3} = eye(4);
    p2 = zeros(6,250);
    p3 = cell(1,250);
    calibParams = cell(1,6);
    matchSet = cell(1,5);
    points3DSet = cell(1,5);
    HSet = cell(1,5);
    
    for i = 1 : length(pairs)
        pair = pairs(i,:);
        [inlierPoints,match] = Calibration.getInliersRansac(pair,data);
        [params,~] = Calibration.stereoCalibrate(inlierPoints,calib(pair(1)).params.IntrinsicMatrix,calib(pair(2)).params.IntrinsicMatrix...
            ,calib(pair(1)).params.RadialDistortion,calib(pair(2)).params.RadialDistortion);
        points3D = Calibration.myTriang(inlierPoints,match, params,pair(1),250);
        matchSet{i} = match;
        points3DSet{i} = points3D;
        
        calibParams{pair(1)} = params.CameraParameters1; 
        calibParams{pair(2)} = params.CameraParameters2;

       
        r = params.RotationOfCamera2;
        t = params.TranslationOfCamera2;
%         relH = [r' -r'*t';0 0 0 1];
        relH = [r' t';0 0 0 1];
        HSet{i} = relH;
        if isempty(H3{pair(2)})
            H3{pair(2)} = relH * H3{pair(1)};
        end    
        loc = Calibration.findLoc(points3D,inv(H3{pair(1)}));
       
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