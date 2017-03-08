function [lr1, lr2, lr3, lr4] = rectifyLinesMetric(H, p1, p2, p3, p4, p5, p6, p7, p8)

p1 = H*p1;
p1 = p1/p1(3);

p2 = H*p2;
p2 = p2/p2(3);


p3 = H*p3;
p3 = p3/p3(3);

p4 = H*p4;
p4 = p4/p4(3);


p5 = H*p5;
p5 = p5/p5(3);

p6 = H*p6;
p6 = p6/p6(3);

p7 = H*p7;
p7 = p7/p7(3);


p8 = H*p8;
p8 = p8/p8(3);

lr1 = cross(p1,p2);
lr2 = cross(p3,p4);
lr3 = cross(p5,p6);
lr4 = cross(p7,p8);

end