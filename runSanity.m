frame = 50;
X = p3{frame}; 
H = H3{1};
R = H(1:3,1:3);
T = H(1:3,4);
K = calibParams{1, 1}.IntrinsicMatrix ;
x = Calibration.findReproj(X,R,T,K');
sanityCheck(frame,x,data);