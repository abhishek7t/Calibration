import gtsam.*
% load('baData.mat')
% load('baInit.mat')
%% 
% Data Options
options.triangle = false;
options.nrCameras = 6;
options.showImages = false;
%% 

measurementNoiseSigma = 1.0;
pointNoiseSigma = 0.1;
poseNoiseSigmas = [0.001 0.001 0.001 0.1 0.1 0.1]';
%% Create the graph (defined in visualSLAM.h, derived from NonlinearFactorGraph)
graph = NonlinearFactorGraph;
%% Add factors for all measurements
measurementNoise = noiseModel.Isotropic.Sigma(2,measurementNoiseSigma);
for i=1:length(baData.Z)
    for k=1:length(baData.Z{i})
%         j = data.J{i}{k};
        if isempty(baData.Z{i}{k})
            continue;
        end
        graph.add(GenericProjectionFactorCal3_S2(baData.Z{i}{k}, measurementNoise, symbol('x',i), symbol('p',k), baData.K{i}));
    end
end

%% Add Gaussian priors for a pose and a landmark to constrain the system
posePriorNoise  = noiseModel.Diagonal.Sigmas(poseNoiseSigmas);
graph.add(PriorFactorPose3(symbol('x',3), baInit.cameras{3}.pose, posePriorNoise));
pointPriorNoise  = noiseModel.Isotropic.Sigma(3,pointNoiseSigma);
graph.add(PriorFactorPoint3(symbol('p',17524), baInit.points{17524}, pointPriorNoise));
%% Print the graph
% graph.print(sprintf('\nFactor graph:\n'));

%% Initialize cameras and points 
initialEstimate = Values;
for i=1:size(baInit.cameras,2)
    pose_i = baInit.cameras{i}.pose;
    initialEstimate.insert(symbol('x',i), pose_i);
end
for j=1:length(baInit.points)
    if isempty(baInit.points{j})
            continue;
    end
    point_j = baInit.points{j};
    initialEstimate.insert(symbol('p',j), point_j);
end
% initialEstimate.print(sprintf('\nInitial estimate:\n  '));

%% Fine grain optimization, allowing user to iterate step by step
parameters = LevenbergMarquardtParams;
parameters.setlambdaInitial(1.0);
parameters.setVerbosityLM('trylambda');

optimizer = LevenbergMarquardtOptimizer(graph, initialEstimate, parameters);

for i=1:20
    optimizer.iterate();
end
result = optimizer.values();
% result.print(sprintf('\nFinal result:\n  '));
%% Plot results with covariance ellipses
% marginals = Marginals(graph, result);
% cla
% hold on;
% 
% plot3DPoints(result, [], marginals);
% plot3DTrajectory(result, '*', 1, 8, marginals);
% 
% axis([-40 40 -40 40 -10 20]);axis equal
% view(3)
% colormap('hot')
 




