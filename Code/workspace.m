%usage:         calculate the workspace of the HUBO biped robot
%auther:        Tze-Yuan Cheng
clear all;
close all;

%maximum joint angle
JointM=[-77;-90;-90;0;-90;-40];
%minimum joint angle
JointP=[60;90;38;150;90;20];
%increment 20 degree for each joint
incAngle=20;
for i= JointM(1,:):incAngle:JointP(1,:)
  for j= JointM(2,:):incAngle:JointP(2,:)
    for k= JointM(3,:):incAngle:JointP(3,:)
      for l= JointM(4,:):incAngle:JointP(4,:)
        for m= JointM(5,:):incAngle:JointP(5,:)
          for n= JointM(6,:):incAngle:JointP(6,:)
    iV_J=[i*pi/180;j*pi/180;k*pi/180;l*pi/180;m*pi/180;n*pi/180];
    PM= FWDE(iV_J);
          end
        end
      end
    end
  end
end


