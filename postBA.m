%% Compute reprojection error
import gtsam.*;
import Calibration.*;

%load result from  res.mat file
% result = res.result;
%%
X = zeros(24750,3);
val = zeros(24750,1);
H = cell(1,6);

for i=1:size(baInit.cameras,2)
    h = result.at(symbol('x',i));
    H{i} = h.matrix;
end
for j=1:length(baInit.points)
    if isempty(baInit.points{j})
            continue;
    end
    x = result.at(symbol('p',j));
    X(j,:) = x.vector()';
    val(j) = 1;
end
%%
reprojectionError = [0 0];
count = 0;
reprojection = cell(6,250);
for cam = 1 : 6
    Hs = inv(H{cam});
    for frame = 1 : 250
        if estimate.p2(cam, frame) == 1
            P = X((frame - 1) * 99 + 1 : frame * 99 , :);
            reprojection{cam, frame} = worldToImage(estimate.calibParams{1, cam}, Hs(1:3, 1:3)', (Hs(1:3,4))', P , 'ApplyDistortion', true);
            reprojectionError = reprojectionError + sum(undistortedData(frame + (cam-1)*250).corners - reprojection{cam, frame});
            count = count + 99;
        end
    end
end