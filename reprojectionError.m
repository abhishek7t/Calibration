n = length(X);
X1 = zeros(n,3);
x1 = zeros(n,2);
val = zeros(n,1);
cam = 1;
K = KSet{cam};
p = Z{cam};
R = RSet{cam};
C = CSet{cam};
for i = 1:n
   if length(p)>=i && ~isempty(p{i})
       
      X1(i,:) = X(i,:);
      x1(i,:) = p{i};
      val(i) = 1;
   end
end
val = logical(val);
X1 = X1(val,:);
x1 = x1(val,:);
Calibration.reprojError(X1,x1,R,-R*C,K')