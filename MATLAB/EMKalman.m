
N = length(Gyrobase);

F = eye(3);
H = eye(3);
Q = diag([1e-1 1e-1 1e-1]);
R = [88.4331  -43.8488   41.1907;...
  -43.8488  169.9083   70.4014;...
   41.1907   70.4014  112.5151];

%preallocate
x = cell(2,N);
P = cell(2,N);
C = cell(1,N);

x{1,1} = [-5.5783; 3.7975; -37.5309];
P{1,1} = zeros(3);

for k = 2:N
    x{1,k} = F*x{1,k-1};
    P{1,k} = F*P{1,k-1}*F.' + Q;
    
    y = Gyrobase(k,:).' - H*x{1,k};
    S = H*P{1,k}*H.' + R;
    K = P{1,k}*H.'*inv(S);
    x{1,k} = x{1,k}+K*y;
    P{1,k} = (eye(3) - K*H)*P{1,k};
end

x{2,N} = x{1,N};
P{2,N} = P{1,N};

for k = N-1:-1:1
    Cl = P{1,k} * F.' * inv(F*P{1,k}*F.' + Q);
    C{k} = Cl;
    x{2,k} = x{1,k} + Cl*(x{2,k+1} - x{1,k+1});
    P{2,k} = P{1,k} + Cl*(P{2,k+1} - P{1,k+1})*Cl.';
end

Ro = zeros(3);
for k = 1:N
	y = Gyrobase(k,:).' - H*x{2,k};
	Ro = Ro + y*y.' + H*P{2,k}*H.';
end
Ro = Ro ./ (N+1);

Qo = zeros(3);
for k = 1:N-1
    dx = x{2,k+1} - F*x{2,k};
    Qo = Qo + dx*dx.' + F*P{2,k}*F.' + P{2,k+1} - P{2,k+1}*C{k}.'*F.' - F*C{k}*P{2,k+1};
end
Qo = Qo ./ N;