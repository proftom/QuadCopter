
Qgyro = 1.0e-03 * ...
  [ 0.1322   -0.0655    0.0616;...
   -0.0655    0.2539    0.1052;...
    0.0616    0.1052    0.1682];

Qacc = 1.0e-03 * ...
  [ 0.8222   -0.1422   -0.0377;...
   -0.1422    2.7314    0.0648;...
   -0.0377    0.0648    0.6670];

Qgyrobias = 1.0e-05 * ...
  [ 0.4734   -0.1316    0.0932;...
   -0.1316    0.5002    0.2104;...
    0.0932    0.2104    0.4087];

Qaccbias = 1.0e-05 * ...
  [ 0.0518   -0.0024   -0.0005;...
   -0.0024    0.1216    0.0002;...
   -0.0005    0.0002    0.0340];

Q = blkdiag(Qgyro, Qacc, Qgyrobias, Qaccbias);

Ts = 0.01;

X = [0 0 0, 0 0 0, 1 0 0 0, 0 0 0, 0,0,0].'; %-0.0336 0.1013 0.0674].'; %-0.1013 -0.0674 -0.0336].';

P0r = zeros(3);
P0v = zeros(3);
P0q = zeros(4);
P0wb = eye(3) * 0.001;
P0ab = eye(3) * 1;

P = blkdiag(P0r, P0v, P0q, P0wb, P0ab);

N = length(Gyro);
Xtrace = zeros(N,16);
Xdot = zeros(16,1);

for k = 1:N
    
    wm = Gyro(k,:).' ./ 818.51113590117601252569;
    am = Accel(k,:).' .* (9.816/(2^14));
    am = [am(3) -am(1) -am(2)].'; %TODO temp hack

    w = wm - X(11:13);
    a = am - X(14:16);
    q = X(7:10);

    Omega = [0    -w(1) -w(2) -w(3); ...
             w(1)  0     w(3) -w(2); ...
             w(2) -w(3)  0     w(1); ...
             w(3)  w(2) -w(1)  0];

    Xi = [-q(2) -q(3) -q(4); ...
           q(1) -q(4)  q(3); ...
           q(4)  q(1) -q(2); ...
          -q(3)  q(2)  q(1)];

    %DCM = quat2dcm(X(7:10));
    %DCMderiv = [norm(q)^2-2*q(3)*q(3)-2*q(4)*q(4) 2*q(2)*q(3)+2*q(4)*q(1) 2*q(2)*q(4)-2*q(3)*q(1);
    %       2*q(2)*q(3)-2*q(4)*q(1) norm(q)^2-2*q(2)*q(2)-2*q(4)*q(4) 2*q(3)*q(4)+2*q(2)*q(1);
    %       2*q(2)*q(4)+2*q(3)*q(1) 2*q(3)*q(4)-2*q(2)*q(1) norm(q)^2-2*q(2)*q(2)-2*q(3)*q(3)];

    %DCM = [1-2*q(3)*q(3)-2*q(4)*q(4) 2*q(2)*q(3)+2*q(4)*q(1) 2*q(2)*q(4)-2*q(3)*q(1);
    %       2*q(2)*q(3)-2*q(4)*q(1) 1-2*q(2)*q(2)-2*q(4)*q(4) 2*q(3)*q(4)+2*q(2)*q(1);
    %       2*q(2)*q(4)+2*q(3)*q(1) 2*q(3)*q(4)-2*q(2)*q(1) 1-2*q(2)*q(2)-2*q(3)*q(3)];

    DCM =  [2*(q(1)^2 + q(2)^2) - 1, 2*(q(2)*q(3) + q(1)*q(4)), 2*(q(2)*q(4) - q(1)*q(3));...
            2*(q(2)*q(3) - q(1)*q(4)), 2*(q(1)^2 + q(3)^2) - 1, 2*(q(3)*q(4) + q(1)*q(2));...
            2*(q(2)*q(4) + q(1)*q(3)), 2*(q(3)*q(4) - q(1)*q(2)), 2*(q(1)^2 + q(4)^2) - 1];


    %Transform X

    %Pos
    Xdot(1:3) = X(4:6);

    %vel
    Xdot(4:6) = DCM.' * a;
    Xdot(6) = Xdot(6) + 9.816;

    %quat
    Xdot(7:10) = 0.5 .* Xi * w;

    %biases
    Xdot(11:16) = 0;

    X = X + Xdot.*Ts;
    X(7:10) = X(7:10)/norm(X(7:10));
    Xtrace(k,:) = X;


    % Transform P

    Fvqparts = Xi * a;
    Fvq = 2 .* [Fvqparts(2) -Fvqparts(1) Fvqparts(4) -Fvqparts(3); ...
                Fvqparts(3) -Fvqparts(4) -Fvqparts(1) Fvqparts(2); ...
                Fvqparts(4) Fvqparts(3) -Fvqparts(2) -Fvqparts(1)];

    Fvba = -DCM.';
    Fqq = 0.5 .* Omega;
    Fqbw = -0.5 .* Xi;

    F = eye(16) + Ts.* [zeros(3) eye(3) zeros(3, 10); ...
                        zeros(3,6) Fvq zeros(3) Fvba; ...
                        zeros(4,6) Fqq Fqbw zeros(4,3); ...
                        zeros(6,16)];

    Gvwa = DCM.';
    Gqww = 0.5 .* Xi;

    G = Ts.* [zeros(3, 12); ...
              zeros(3) Gvwa zeros(3,6); ...
              Gqww zeros(4, 9); ...
              zeros(3,6) eye(3) zeros(3); ...
              zeros(3,9) eye(3)];

    P = F*P*F.' + G*Q*G.';
    
    
    %-------------------
    
    if (k <= 5000)
        z = [0 0 0 0 0 0 1 0 0 0].';
        H = blkdiag([eye(10) zeros(10,6)]);
        y = z - H*X;
        R = eye(10) .* 0.0001;

        S = H*P*H.' + R;
        K = P*H.'*inv(S);

        X = X + K*y;
        P = (eye(16) - K*H)*P;
    end
    
    
    
end

