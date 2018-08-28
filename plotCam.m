CSet = cell(1,6);
RSet = cell(1,6);
for i = 1 : 6
   H = inv(H3{i});
%    HSet from ba.m
%    H = HSet{i};
   CSet{i} = H(1:3,4);
   RSet{i} = H(1:3,1:3)';
end
plotC(CSet, RSet, .01);