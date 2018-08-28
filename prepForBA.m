function [KSet, HSet, worldPoints, imagePoints] = prepForBA(estimate,undistortedData)
    %% prepare data for bundle adjustment
    calibParams = estimate.calibParams;
    p2 = estimate.p2;
    p3 = estimate.p3;
    H3 = estimate.H3;
   % HSet{i} is camera i to world, transformation matrix
    HSet = cell(1,6);
    KSet = cell(1,6);
    for i = 1:6
       KSet{i} = calibParams{1, i}.IntrinsicMatrix;
       H = inv(H3{i});
       C = H(1:3,4);
       R = H(1:3,1:3);
       HSet{i} = [R C; 0 0 0 1];
    end

    worldPoints = cell(1, 250 * 99);
    
    %%
    j = 1;
    for i = 1 : length(p3)
       if ~isempty(p3{i})
           p = p3{i};
           for k = 0:98
               worldPoints{j+k} = p(k+1,:);
           end
       end
       j = j + 99;
    end

    %%
    imagePoints = cell(1,6);
    for cam = 1 : 6
        j = 1;
         camPoints = cell(1, 250 * 99);
        for i = 1 : length(p3)
           if p2(cam, i) == 1
               p = undistortedData(i + (cam-1)*250).corners;
               for k = 0:98
                   camPoints{j+k} = p(k+1,:);
               end
           end
           j = j + 99;
        end
        imagePoints{cam} = camPoints;
    end
end