data = load('errorsamples10000.txt');
[m,n] = size(data);
length = m/5;

%perform innovation bound test.
% get the list of error and their correlations.
error_stack = zeros(length, 4);
cov_stack = zeros(length*4, 4);
j = 1;
k = 1;
dist = 0;
dist_m = [];
for i = 1:5:m
    error_stack(j,:) = data(i,:);
    cov_stack(k:k+3,1:4) = data(i+1:i+4,1:4);
   
    
    diff = error_stack(j,:);
    S = cov_stack(k:k+3,1:4);
    tempDist = (diff*inv(S)*diff');
    dist = dist + tempDist;
    dist_m = vertcat(dist_m, tempDist);
       j = j + 1;
    k = k + 1;
end
hist(dist_m, 100);
title('Mahalanobis Distance Sample');
%analysis
disp(dist/length);
%find bounds 0.99
low = chi2inv(0.025, 4);
high = chi2inv(0.975, 4);
disp(low);
disp(high);