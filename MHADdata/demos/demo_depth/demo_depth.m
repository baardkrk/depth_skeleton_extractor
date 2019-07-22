clear;
close all;

%%%% REQUIREMENTS
% Before running this script, you need to do the following:
% 1) download and extract the following packages into the working directory:
%    - MOCAP (0.136) and NDLUTIL (0.161) at http://staffwww.dcs.shef.ac.uk/people/N.Lawrence/mocap/
% 2) run the 'BerkeleyMHAD_download.bat' script to download the test data into the working directory
% 3) now you can just run this script
%
%%%% NOTES
% 1) Make sure to include 'bvh2xyz.m' that comes with the download in your working directory so that it will override calls to the original from the MOCAP package.
% 2) Make sure to include 'rodrigues.m' in your working directory. This script is taken from the Camera Calibration Toolbox of Dr. Jean-Yves Bouguet from Caltech, which is available online at http://www.vision.caltech.edu/bouguetj/calib_doc/download/index.html
%
% Updated: 2017/01/11

addpath 'MOCAP0p136';
addpath 'NDLUTIL0p161';

% set this flag to 1 if you want to save the demo as a video
v = 0;

% selected subject - action - recording
s = 1;
a = 1;
r = 1;

drc = 'BerkeleyMHAD';

% load image-mocap correspondences, i.e., image frame corresponds to mocap
% frame correspondences
im_mc = load(sprintf('%s/Kinect/Correspondences/corr_moc_kin01_s%02d_a%02d_r%02d.txt',drc,s,a,r));

% load mocap data
moc_data = load(sprintf('%s/Mocap/OpticalData/moc_s%02d_a%02d_r%02d.txt',drc,s,a,r));

% load skeleton data
[skel,channel,framerate] = bvhReadFile(sprintf('%s/Mocap/SkeletalData/skl_s%02d_a%02d_r%02d.bvh',drc,s,a,r));
skel_jnt = 10*chan2xyz(skel,channel);

% load camera calibration parameters
calib_params;

% virtual digital camera in camera coordinate frame
CC=[-50 -50 50 1;
    50 -50 50 1;
    50 50 50 1;
    -50 50 50 1;
    -50 -50 50 1;
    0 0 0 1;
    -50 50 50 1;
    0 0 0 1;
    50 50 50 1;
    0 0 0 1;
    50 -50 50 1]';

% virtual kinect camera in kinect coordinate frame
CK=[-150 -50 50 1;
    150 -50 50 1;
    150 50 50 1;
    -150 50 50 1;
    -150 -50 50 1;
    0 0 0 1;
    -150 50 50 1;
    0 0 0 1;
    150 50 50 1;
    0 0 0 1;
    150 -50 50 1]';

% get focal length and image center from the intrinsic matrix (K)
fx1 = K(1).K(1,1);
fy1 = K(1).K(2,2);
cx1 = K(1).K(1,3);
cy1 = K(1).K(2,3);

fx2 = K(2).K(1,1);
fy2 = K(2).K(2,2);
cx2 = K(2).K(1,3);
cy2 = K(2).K(2,3);

% homogeneous transformation from Kinect coordinate frame to the world
% coordinate frame (R,t)
% H1 = [K(1).R' -K(1).R'*K(1).t; 0 0 0 1];
H1 = K(1).Tinv;

% H2 = [K(2).R' -K(2).R'*K(2).t; 0 0 0 1];
H2 = K(2).Tinv;

% position of the kinect in the world coordinate frame
% p_K1 = H1 * [0 0 0 1]'; % this is already stored in K(1).p

% p_K2 = H2 * [0 0 0 1]'; % this is already stored in K(2).p


% virtual Kinect1 in world coordinate frame
K1_CC = H1*CK;

% virtual Kinect2 in world coordinate frame
K2_CC = H2*CK;

% virtual digital cameras in world coordinate frame
L1_C1_CC = L(1).Tinv*L(1).C(1).Tinv*CC;
L1_C2_CC = L(1).Tinv*L(1).C(2).Tinv*CC;
L1_C3_CC = L(1).Tinv*L(1).C(3).Tinv*CC;
L1_C4_CC = L(1).Tinv*L(1).C(4).Tinv*CC;

L2_C1_CC = L(2).Tinv*L(2).C(1).Tinv*CC;
L2_C2_CC = L(2).Tinv*L(2).C(2).Tinv*CC;
L2_C3_CC = L(2).Tinv*L(2).C(3).Tinv*CC;
L2_C4_CC = L(2).Tinv*L(2).C(4).Tinv*CC;

L3_C1_CC = L(3).Tinv*L(3).C(1).Tinv*CC;
L3_C2_CC = L(3).Tinv*L(3).C(2).Tinv*CC;

L4_C1_CC = L(4).Tinv*L(4).C(1).Tinv*CC;
L4_C2_CC = L(4).Tinv*L(4).C(2).Tinv*CC;

%%
% create movie object
if v,
    movobj = VideoWriter('demo_depth','MPEG-4');
    movobj.FrameRate = 10;
    open(movobj);
end

aa = linspace(0,360-(360/size(im_mc,1)),size(im_mc,1));

hf = figure('Position',[100,100,640,480]);

for i=1:size(im_mc,1),
    
    % get the mocap frame
    moc = reshape(moc_data(im_mc(i,3)+1,1:end-2),3,[]);
    
    % get the skeleton frame
    jnt = reshape(skel_jnt(im_mc(i,3)+1,:),3,[]);
    
    % read in the depth data
    d1 = double(imread(sprintf('%s/Kinect/Kin01/S%02d/A%02d/R%02d/kin_k01_s%02d_a%02d_r%02d_depth_%05d.pgm',drc,s,a,r,s,a,r,i-1)));
    
    d2 = double(imread(sprintf('%s/Kinect/Kin02/S%02d/A%02d/R%02d/kin_k02_s%02d_a%02d_r%02d_depth_%05d.pgm',drc,s,a,r,s,a,r,i-1)));
    
    % number of non-zero depth values
    nd1 = nnz(d1);
    
    nd2 = nnz(d2);
    
    
    % convert the pixel location + depth value pairs into 3D points in kinect
    % coordinate frame
    D1 = ones(4,nd1);
    ind = 1;
    for y=1:size(d1,1),
        for x=1:size(d1,2),
            % consider only those points that are farther than 3.5 meters
            % for the purpose of this example
            if d1(y,x) > 3500, continue; end;
            if d1(y,x) ~= 0,
                D1(1,ind) = (x-cx1)*d1(y,x)/fx1;
                D1(2,ind) = (y-cy1)*d1(y,x)/fy1;
                D1(3,ind) = d1(y,x);
                ind = ind + 1;
            end
        end
    end
    D1 = D1(:,1:ind-1);
    
    D2 = ones(4,nd2);
    ind = 1;
    for y=1:size(d2,1),
        for x=1:size(d2,2),
            % consider only those points that are farther than 3.5 meters
            % for the purpose of this example
            if d2(y,x) > 3500, continue; end;
            if d2(y,x) ~= 0,
                D2(1,ind) = (x-cx2)*d2(y,x)/fx2;
                D2(2,ind) = (y-cy2)*d2(y,x)/fy2;
                D2(3,ind) = d2(y,x);
                ind = ind + 1;
            end
        end
    end
    D2 = D2(:,1:ind-1);
    
    % transform the Kinect point clouds into world coordinate frame
    DD1 = H1*D1;
    DD2 = H2*D2;
    
    
    % plot the kinect and the point cloud in world coordinate frame
    figure(1)
    plot3(0,0,0,'ko');
    hold on;
    
    plot3(moc(3,:), moc(1,:), moc(2,:), 'y*');
    plot3(jnt(3,:), jnt(1,:), jnt(2,:), 'k*');
    
    plot3(K(1).p(3),K(1).p(1),K(1).p(2),'r*');
    plot3(K1_CC(3,:),K1_CC(1,:),K1_CC(2,:),'r-','LineWidth',2);
    plot3(K(2).p(3),K(2).p(1),K(2).p(2),'b*');
    plot3(K2_CC(3,:),K2_CC(1,:),K2_CC(2,:),'b-','LineWidth',2);
    
    plot3(DD1(3,:),DD1(1,:),DD1(2,:),'r.');
    plot3(DD2(3,:),DD2(1,:),DD2(2,:),'b.');
    
    plot3(L(1).C(1).p(3),L(1).C(1).p(1),L(1).C(1).p(2),'m*');
    plot3(L1_C1_CC(3,:),L1_C1_CC(1,:),L1_C1_CC(2,:),'m-','LineWidth',2);
    plot3(L(1).C(2).p(3),L(1).C(2).p(1),L(1).C(2).p(2),'c*');
    plot3(L1_C2_CC(3,:),L1_C2_CC(1,:),L1_C2_CC(2,:),'c-','LineWidth',2);
    plot3(L(1).C(3).p(3),L(1).C(3).p(1),L(1).C(3).p(2),'g*');
    plot3(L1_C3_CC(3,:),L1_C3_CC(1,:),L1_C3_CC(2,:),'g-','LineWidth',2);
    plot3(L(1).C(4).p(3),L(1).C(4).p(1),L(1).C(4).p(2),'k*');
    plot3(L1_C4_CC(3,:),L1_C4_CC(1,:),L1_C4_CC(2,:),'k-','LineWidth',2);
    
    plot3(L(2).C(1).p(3),L(2).C(1).p(1),L(2).C(1).p(2),'m*');
    plot3(L2_C1_CC(3,:),L2_C1_CC(1,:),L2_C1_CC(2,:),'m-','LineWidth',2);
    plot3(L(2).C(2).p(3),L(2).C(2).p(1),L(2).C(2).p(2),'c*');
    plot3(L2_C2_CC(3,:),L2_C2_CC(1,:),L2_C2_CC(2,:),'c-','LineWidth',2);
    plot3(L(2).C(3).p(3),L(2).C(3).p(1),L(2).C(3).p(2),'g*');
    plot3(L2_C3_CC(3,:),L2_C3_CC(1,:),L2_C3_CC(2,:),'g-','LineWidth',2);
    plot3(L(2).C(4).p(3),L(2).C(4).p(1),L(2).C(4).p(2),'k*');
    plot3(L2_C4_CC(3,:),L2_C4_CC(1,:),L2_C4_CC(2,:),'k-','LineWidth',2);
    
    plot3(L(3).C(1).p(3),L(3).C(1).p(1),L(3).C(1).p(2),'m*');
    plot3(L3_C1_CC(3,:),L3_C1_CC(1,:),L3_C1_CC(2,:),'m-','LineWidth',2);
    plot3(L(3).C(2).p(3),L(3).C(2).p(1),L(3).C(2).p(2),'c*');
    plot3(L3_C2_CC(3,:),L3_C2_CC(1,:),L3_C2_CC(2,:),'c-','LineWidth',2);
    
    plot3(L(4).C(1).p(3),L(4).C(1).p(1),L(4).C(1).p(2),'m*');
    plot3(L4_C1_CC(3,:),L4_C1_CC(1,:),L4_C1_CC(2,:),'m-','LineWidth',2);
    plot3(L(4).C(2).p(3),L(4).C(2).p(1),L(4).C(2).p(2),'c*');
    plot3(L4_C2_CC(3,:),L4_C2_CC(1,:),L4_C2_CC(2,:),'c-','LineWidth',2);
    
    
    xlabel('z');
    ylabel('x');
    zlabel('y');
    axis([-2500 2500 -2500 2500 0 2500]);
    % axis equal;
    hold off;
    
    view([aa(i) 10]);
    
    pause(0.1);
    
    if v,
        gf = getframe(hf);
        writeVideo(movobj,im2frame(gf.cdata,gf.colormap));
    end
    
end

if v, close(movobj); end

close(1);