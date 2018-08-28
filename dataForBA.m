function [baData, baInit] = dataForBA(KSet, H3, pointsInit, Z )
    import gtsam.*;
    cams = cell(1,6);
    Ks = cell(1,6);
    for i = 1:6
        pose = Pose3(H3{i});
    %     pose = Pose3(gtsam.Rot3(RSet3{i}), Point3(CSet3{i}(1),CSet3{i}(2),CSet3{i}(3)));
        K = Cal3_S2(KSet{i}(1,1),KSet{i}(2,2),0,KSet{i}(3,1),KSet{i}(3,2));
        Ks{i} = K;
        cams{i} = SimpleCamera(pose,  K);
    end

    ZPoint2 = cell(1,6);
    for i = 1:6
        cp = Z{i};
        cz = cell(1,length(cp));
        for j = 1:length(cp)
    %         disp(cp{i})
            if isempty(cp{j})
               continue ;
            end
            cz{j} = Point2(cp{j}(1),cp{j}(2));
        end
        ZPoint2{i} = cz;
    end

    points = cell(1,24750);
    for i = 1:length(pointsInit)
        if isempty(pointsInit{i})
               continue ;
        end
        points{i} = Point3(pointsInit{i}(1),pointsInit{i}(2),pointsInit{i}(3));
    end
    baData = struct;
    baData.Z = ZPoint2;
    baData.K = Ks;
    baInit = struct;
    baInit.K = Ks;
    baInit.points = points;
    baInit.cameras = cams;
end