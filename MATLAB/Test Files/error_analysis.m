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

%bound counts
%find the average variances in all dimensions.
[len,x] = size(cov_stack);
avecov = zeros(4,4);
for i = 1:4:len
    avecov = avecov + cov_stack(i:i+3, 1:4);
end
avecov = avecov*4/len;

[len,x] = size(error_stack);
xbound = 2* sqrt(abs(avecov(1,1)));
ybound = 2* sqrt(abs(avecov(2,2)));
zbound = 2* sqrt(abs(avecov(3,3)));
dbound = 2* sqrt(abs(avecov(4,4)));
xcount = 0;
ycount = 0;
zcount = 0;
dcount = 0;
for i = 1:len
    if (abs(data(i, 1)) > xbound)
        xcount = xcount + 1;
    end
    if (abs(data(i, 2)) > ybound)
        ycount = ycount + 1;
    end
    if (abs(data(i, 3)) > zbound)
        zcount = zcount + 1;
    end
    if (abs(data(i, 4)) > dbound)
        dcount = dcount + 1;
    end
end
disp(1- xcount/len);
disp(1- ycount/len);
disp(1- zcount/len);
disp(1- dcount/len);

figure;
autocorr(error_stack(:,1));
title('Autocorrelation of Noise in Nx');
figure;
autocorr(error_stack(:,2));
title('Autocorrelation of Noise in Ny');
figure;
autocorr(error_stack(:,3));
title('Autocorrelation of Noise in Nz');
figure;
autocorr(error_stack(:,4));
title('Autocorrelation of Noise in d');