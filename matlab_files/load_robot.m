clear

load data.mat
robot = importrobot("../Panda/panda_.urdf")
homeconfig = randomConfiguration(robot);
Tbe=zeros(4,4,size(image,4));
for i = 1:1:size(image,4)
config = homeconfig;
for j =1 :1:7
    config(j).JointPosition = jointPoses(j,i)
end
T = getTransform(robot,config,"panda_link0","panda_link8")
Tbe(:,:,i) =T;
show(robot,config)
drawnow;
hold on;
end
save data.mat
