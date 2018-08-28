CSet = cell(1,6);
RSet = cell(1,6);
for i = 1 : 6
   H = inv(H3{i});
<<<<<<< HEAD
%    HSet from ba.m
%    H = HSet{i};
=======
>>>>>>> 5644f242fed247ca9baa4de52028c0895f9572bc
   CSet{i} = H(1:3,4);
   RSet{i} = H(1:3,1:3)';
end
plotC(CSet, RSet, .01);