import gtsam.*;
[KSet, HSet, worldPoints, imagePoints] = prepForBA(estimate,undistortedData);
[baData, baInit] = dataForBA(KSet, HSet, worldPoints, imagePoints );
baTrial1;
postBA;