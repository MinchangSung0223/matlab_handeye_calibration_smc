close all;
clear
format shortG
load data.mat
load_robot

s = size(image)
imagePoints_list = {}
worldPoints_list = {}
squareSize = 15%mm
board_size =[8,11]
imagePoints_=zeros(board_size(1)*board_size(2),2,s(4));
worldPoints_=zeros(board_size(1)*board_size(2),2,s(4));

for i =1:1:s(4)
    I = image(:,:,1:3,i);
    [imagePoints,boardSize] = detectCheckerboardPoints(I);
    boardSize
    imagePoints_list{end+1} = imagePoints;
    worldPoints = generateCheckerboardPoints(boardSize, squareSize);
    worldPoints_list{end+1} = worldPoints-repmat([75,105/2],88,1);
    imageSize = [size(I, 1),size(I, 2)];
    
    imagePoints_(:,:,i)=imagePoints;
    worldPoints_(:,:,i)=worldPoints;
    
%      figure(i)
%      imwrite(I,"./Images/"+string(i)+".png")
%      imshow(I)
%      hold on;
%      plot(imagePoints(:,1),imagePoints(:,2),'ro');
%      pause(0.5);
end

cameraParams = estimateCameraParameters(imagePoints_,worldPoints);


%add square size to chessboard
worldPoints = [worldPoints, zeros(size(worldPoints,1),1), ones(size(worldPoints,1),1)]';

%get camera matrix
K = cameraParams.IntrinsicMatrix';

%get distortion parameters
p = cameraParams.TangentialDistortion;
k = cameraParams.RadialDistortion;
baseEst = eye(4);
baseT = eye(4);
gripPose = [0.06  -0.13-0.0375 0 1.57079632679 0 3.141592];
uv_list = []
uv_list_tt= []
Tbc_answer = pose_to_transformation([0.7, 0.5, 0.5, 0,-pi+pi/4,-pi/4])
Tbc_list = zeros(4,4,s(4));
err= 0;
M_list = []
A = zeros(4,4,s(4)-1);
B = zeros(4,4,s(4)-1);
rob_pose_list = zeros(4,4,s(4));
obj_pose_list = zeros(4,4,s(4));

for i=1:1:s(4)-2
    TbE1 = Tbe(:,:,i);
    TbE2 = Tbe(:,:,i+1);
    TH1C = pose_R_to_transformation(cameraParams.TranslationVectors(i,:)'/1000  ,cameraParams.RotationMatrices(:,:,i)');
    TH2C = pose_R_to_transformation(cameraParams.TranslationVectors(i+1,:)'/1000  ,cameraParams.RotationMatrices(:,:,i+1)');
    A(:,:,i) = TransInv(TbE2)*(TbE1);
    B(:,:,i) = TransInv(TH2C)*(TH1C);
end
X=calibration_AX_XB(A,B)

cam_pose_list = zeros(4,4,s(4))
rob_pose_list = zeros(4,4,s(4));
obj_pose_list = zeros(4,4,s(4));
for i =1:1:s(4)
TbE1 = Tbe(:,:,i);
TH1C = pose_R_to_transformation(-cameraParams.TranslationVectors(i,:)'/1000  ,cameraParams.RotationMatrices(:,:,i)');

cam_pose = TbE1*X*(TH1C);
cam_pose_list(:,:,i) = cam_pose
rob_pose_list(:,:,i) = TbE1;
obj_pose_list(:,:,i) = TH1C;

end


clc;
baseToCamTransformation = mean(cam_pose_list,3)
EndEffectorToBoardTransformation =X

function invT = TransInv(T)
[R, p] = TransToRp(T);
invT = [transpose(R), -transpose(R) * p; 0, 0, 0, 1];
end
function [R, p] = TransToRp(T)
R = T(1: 3, 1: 3);
p = T(1: 3, 4);
end

function T = pose_to_transformation(pose)
T = eye(4);
x=pose(1);
y=pose(2);
z=pose(3);
roll = pose(4);
pitch = pose(5);
yaw = pose(6);
rotationMatrix = rotationVectorToMatrix([roll,pitch,yaw])
tform = trvec2tform([x,y,z]);
tform(1:3,1:3) =  rotationMatrix;
T = tform;
end
function T = pose_R_to_transformation(t,R)
T = eye(4);
T(1:3,1:3) = R;
T(1:3,4) =t;
end

function T = calibration_AX_XB(A,B)
    T = eye(4);
    N = size(A,3);
    M =zeros(3,3);
    for i = 1:1:N
        Ra = A(1:3,1:3,i);
        Rb = B(1:3,1:3,i);
        M = M+(so3ToVec(MatrixLog3(Rb))*so3ToVec(MatrixLog3(Ra))')
    end
    Rx = sqrtm(inv(M'*M))*M';
    C = zeros(3*N,3);
    d = zeros(3*N,1);
    for i = 1:1:N
        Ra = A(1:3,1:3,i);
        ta = A(1:3,4,i);
        Rb = B(1:3,1:3,i);
        tb = B(1:3,4,i);
        C(3*i:3*i+2,:) = eye(3)-Ra;
        d(3*i:3*i+2,1) = ta-Rx*tb;
    end
    tx = (pinv(C)*d);
    T(1:3,1:3) = Rx;
    T(1:3,4) = tx;
end
function ret =invsqrt(mat)
[U,S,V]=svd(mat);
V = V';
s = diag(S);
ret = U*diag(1./sqrt(s))*V;
end
