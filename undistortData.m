function undistortedData = undistortData(data, calibParams)
undistortedData = data;
for cam = 1:6
    st = (cam - 1)*250 + 1;
    for i = st : st + 249
        if ~isempty(data(i).corners)
             undistortedPoints = undistortPoints(data(i).corners,calibParams{1,cam});
             undistortedData(i).corners = undistortedPoints;
        end
    end  
end