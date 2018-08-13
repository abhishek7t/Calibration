cam = 3;
[H3, calibParams, p2, p3] = estimates(data);
vis = p2(3,:) > 0;
ind = 1:250;
vis= ind(logical(vis));
frame = vis(10);

X = p3{frame}; 
H = H3{3};
R = H(1:3,1:3);
T = H(1:3,4);
K = calibParams{1, 3}.IntrinsicMatrix ;
% x = Calibration.findReproj(X,R,T,K');
x = worldToImage(calibParams{1, 3}, R, T , X, 'ApplyDistortion', true);
sanityCheck(250*(cam-1)+frame,x,data);