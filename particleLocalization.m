function myPose = particleLocalization(ranges, scanAngles, map, param)

%param.init_pose(3) = - param.init_pose(3);

N = size(ranges, 2);
myPose = zeros(3, N);
myResolution = param.resol;
myOrigin = param.origin;
myPose(:,1) = param.init_pose;

% Decide the number of particles, M.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
M = 100 ; % Please decide a reasonable number of M,
% based on your experiment using the practice data.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Create M number of particles
Particles_samp = repmat(myPose(:,1), [1, M]);
P_new=zeros(3,M);
dR=.2;
dTheta = .2;

for j = 2:N % You will start estimating myPose from j=2 using ranges(:,2).
correlation = zeros(1,M);
Weight_map = ones(1,M)/M;
% % 1) Propagate the particles
P_new(1,:) = Particles_samp(1,:)+ dR*cos(P_new(3,:)).*randn(1,M);
P_new(2,:) = Particles_samp(2,:) - dR*sin(P_new(3,:)).*randn(1,M);
P_new(3,:) = Particles_samp(3,:)+dTheta*randn(1,M);

% % 2) Measurement Update
for Particle_number = 1:M
% % 2-1) Find grid cells hit by the rays (in the grid map coordinate frame)
myMAP = zeros(size(map));
mapx = round(P_new(1,Particle_number)*param.resol+param.origin(1));
mapy = round(P_new(2,Particle_number)*param.resol+param.origin(2));
if map(mapy,mapx)>=.49
continue;
end
d = ranges(:,j);
lidar_global_x = round((d.*cos(P_new(3,Particle_number)+scanAngles) + P_new(1,Particle_number))*myResolution + myOrigin(1));
lidar_global_y = round((-d.*sin(P_new(3,Particle_number)+scanAngles) + P_new(2,Particle_number))*myResolution + myOrigin(2));

for k=1:size(scanAngles,1)
if (lidar_global_x(k) >1 && lidar_global_y(k)>1 && lidar_global_x(k)<=size(map,2) && lidar_global_y(k)<=size(map,1))
sub_indice = sub2ind(size(map), lidar_global_y(k), lidar_global_x(k));
myMAP(sub_indice)=1;
end
end

% % 2-2) For each particle, calculate the correlation scores of the particles
correlation(Particle_number) = sum(sum(myMAP.*map));
end
%
% % 2-3) Update the particle weights
Weight_map = Weight_map.*correlation;
Weight_map = Weight_map - min(Weight_map);
%
% % 2-4) Choose the best particle to update the pose
Weight_map = Weight_map / sum(Weight_map);
[~, ind] = sort(Weight_map,'descend');

 % % 3) Resample if the effective number of particles is smaller than a threshold
myPose(:,j) = mean(P_new(:,ind(1:3)),2);
Particles_samp = repmat(myPose(:,j), [1,M]);

if (j>100)
dTheta = .1;
end
end

end