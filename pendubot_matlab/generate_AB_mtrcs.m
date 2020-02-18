% %{
%run('nominal_trajectory.m');
% %{
% Trying to fit eps2 with fourier series
% f = fit(eps_iterp(:,1),eps_iterp(:,2),'fourier4');

s = x_iterp(:,1);
s_d = x_iterp(:,2);

[~,locs] = findpeaks(s);
T_x = t_iterp(locs(2)) - t_iterp(locs(1)); 
T_ind = locs(2) - locs(1) + 1;

A = zeros(3,3,T_ind);
B = zeros(3,1,T_ind);

for i = 1:T_ind
    A(:,:,i) = A_mtrx( s(locs(1) + i -1),s_d(locs(1) + i -1));
    B(:,:,i) = B_mtrx( s(locs(1) + i -1),s_d(locs(1) + i -1));
end

disp(size(A));
