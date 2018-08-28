import gtsam.*;
% H is camera to world transformation matrix
CSet = cell(1,10);
RSet = cell(1,10);
for i=1:10
    h = result.at(symbol('x',i));
%     h = h.pose;
    H = h.matrix;
    CSet{i} = H(1:3,4);
    RSet{i} = H(1:3,1:3)';
end
plotC(CSet, RSet, 1);