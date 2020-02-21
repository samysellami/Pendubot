% %{ generation of the linear matrices along the nominal trajectory
%run('nominal_trajectory.m');
% %{

s = x(:,1);
s_d = x(:,2);

[~,locs] = findpeaks(s);
T_x = t(locs(2)) - t(locs(1)); 
T_ind = locs(2) - locs(1) + 1;

A = zeros(3,3,T_ind);
B = zeros(3,1,T_ind);

for i = 1:T_ind
    A(:,:,i) = A_mtrx( s(locs(1) + i -1),s_d(locs(1) + i -1));
    B(:,:,i) = B_mtrx( s(locs(1) + i -1),s_d(locs(1) + i -1));
end

disp(size(A));