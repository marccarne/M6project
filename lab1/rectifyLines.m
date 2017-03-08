function [lr1, lr2, lr3, lr4, pp1, pp2, pp3, pp4, pp5, pp6, pp7, pp8] = rectifyLines(A, H, i1, i2, i3, i4)

i = i1;
p1 = [A(i,1) A(i,2) 1]';
p1 = H*p1;
pp1 = p1;
p1 = p1/p1(3);

p2 = [A(i,3) A(i,4) 1]';
p2 = H*p2;
pp2 = p2;
p2 = p2/p2(3);

i = i2;
p3 = [A(i,1) A(i,2) 1]';
p3 = H*p3;
pp3 = p3;
p3 = p3/p3(3);

p4 = [A(i,3) A(i,4) 1]';
p4 = H*p4;
pp4 = p4;
p4 = p4/p4(3);

i = i3;
p5 = [A(i,1) A(i,2) 1]';
p5 = H*p5;
pp5 = p5;
p5 = p5/p5(3);

p6 = [A(i,3) A(i,4) 1]';
p6 = H*p6;
pp6 = p6;
p6 = p6/p6(3);

i = i4;
p7 = [A(i,1) A(i,2) 1]';
p7 = H*p7;
pp7 = p7;
p7 = p7/p7(3);

p8 = [A(i,3) A(i,4) 1]';
p8 = H*p8;
pp8 = p8;
p8 = p8/p8(3);

lr1 = cross(p1,p2);
lr2 = cross(p3,p4);
lr3 = cross(p5,p6);
lr4 = cross(p7,p8);

end