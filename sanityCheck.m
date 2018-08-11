function [ ] = sanityCheck(frame,reproj,data)
%visualize the reprojection of the frame

    close all
   
    img1 = imread(data(frame).name);
    figure(1)
    imshow(insertMarker(img1,data(frame).corners(:,:))); 
    figure(2)
    imshow(insertMarker(img1,reproj)); 
end

