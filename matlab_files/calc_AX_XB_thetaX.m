function thetaX=calc_AX_XB_thetaX(A,B)

M=[[ A(1,1) - B(1,1),        A(1,2),        A(1,3),       -B(2,1),           0,           0,       -B(3,1),           0,           0];...
[        A(2,1), A(2,2) - B(1,1),        A(2,3),           0,       -B(2,1),           0,           0,       -B(3,1),           0];...
[        A(3,1),        A(3,2), A(3,3) - B(1,1),           0,           0,       -B(2,1),           0,           0,       -B(3,1)];...
[       -B(1,2),           0,           0, A(1,1) - B(2,2),        A(1,2),        A(1,3),       -B(3,2),           0,           0];...
[           0,       -B(1,2),           0,        A(2,1), A(2,2) - B(2,2),        A(2,3),           0,       -B(3,2),           0];...
[           0,           0,       -B(1,2),        A(3,1),        A(3,2), A(3,3) - B(2,2),           0,           0,       -B(3,2)];...
[       -B(1,3),           0,           0,       -B(2,3),           0,           0, A(1,1) - B(3,3),        A(1,2),        A(1,3)];...
[           0,       -B(1,3),           0,           0,       -B(2,3),           0,        A(2,1), A(2,2) - B(3,3),        A(2,3)];...
[           0,           0,       -B(1,3),           0,           0,       -B(2,3),        A(3,1),        A(3,2), A(3,3) - B(3,3)]]
[~,~,U] = svd(M);
x = U(:, end)./U(end,end);
thetaX = [x(1), x(4), x(7);...
    x(2), x(5), x(8);...
    x(3), x(6), x(9)];

end