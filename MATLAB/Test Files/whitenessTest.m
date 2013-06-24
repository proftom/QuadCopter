data = load('errorsamples10000.txt');
[m,n] = size(data);
length = m/5;

%perform innovation bound test.
% get the list of error and their correlations.
error_stack = zeros(length, 4);
cov_stack = zeros(length*4, 4);
j = 1;
k = 1;
for i = 1:5:m
    error_stack(j,:) = data(i,:);
    cov_stack(k:k+3,1:4) = data(i+1:i+4,1:4);
    j = j + 1;
    k = k + 1;
end

[ACF, lags, bounds] = autocorr(error_stack(:,4), length-1, 2);
bound = 2/sqrt(length);
count = 0;
[len,x] = size(ACF);
for i = 1:len
if (abs(ACF(i)) > bound)
    count = count + 1;
end
end

disp(1-count/len);