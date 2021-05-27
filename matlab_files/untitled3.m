A = sym('A', [3 3])
B = sym('B', [3 3])

X = sym('X', [3 3])
assume(A,'real');
assume(B,'real');
assume(X,'real');
I = eye(3)
I = I(:)
AX=kron(X',A)*I
XB=kron(B',X)*I

M = AX-XB

vars =X(:);
eqns = []
for i = 1:1:length(M)
    equns_ = M(i)==0
    eqns=[eqns,equns_]
end
[A,b] = equationsToMatrix(eqns,vars)