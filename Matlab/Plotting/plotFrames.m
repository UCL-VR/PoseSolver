function [] = plotFrames(f,n)
%PLOTFRAMES Summary of this function goes here
%   Detailed explanation goes here



positions = f(:,1:3);
rotations = f(:,4:7);
rotations = quaternion(rotations(:,4),rotations(:,1),rotations(:,2),rotations(:,3));

fig = figure;
num = height(f);

patch1 = poseplot(positions(1,:), "ENU", Orientation=rotations(1,:));

ylim([-15 15])
xlim([-15 15])
zlim([-15 15])
xlabel("North-x (m)")
ylabel("East-y (m)")
zlabel("Down-z (m)");

hold on;

patch2 = poseplot(positions(2,:), "ENU", Orientation=rotations(2,:));


function update(i)
  set(patch1,Orientation=rotations(i,:),Position=positions(i,:));
  set(patch2,Orientation=rotations(i+1,:),Position=positions(i+1,:));
  drawnow
end

b = uicontrol('Parent',fig,'Style','slider','Position',[81,54,419,23], ...
    'value',0,'min',0, 'max',1);

Callback = @(es,ed) update((floor((ed.AffectedObject.Value * num) / 2) * 2) + 1);

addlistener(b, 'Value', 'PostSet', Callback);

end

