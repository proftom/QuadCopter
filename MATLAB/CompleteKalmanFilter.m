function [Xtrace, iniXtrace] = CompleteKalmanFilter( Gyro, Accel, Planedata)
[X, P, T, iniXtrace] = KalmanFilterInitialiser(Gyro, Accel, Planedata);
%Pause for a bit
[Xtrace] = KalmanFilter(Gyro, Accel, Planedata, X, P, T);
end


function [Xtrace] = KalmanFilter(Gyro, Accel, Planedata, Xin, Pin, T)
%Take only robot state and P.
X = Xin(1:16, 1);
P = Pin(1:16, 1:16);
N = length(Gyro);
Xtrace = zeros(N,16);
lastplanedata = 0;
[bla1,bla2,Q] = initialiseKalman;

for k = 3001:N
    T = T + 0.01;
    [X, P] = prediction(X, P, Q, Accel, Gyro, k);
    [X, P, lastplanedata] = processObservation(X, P, Planedata, lastplanedata, T);
    Xtrace(k,:) = X(1:16);
end
end

function [ X, P, T , iniXtrace] = KalmanFilterInitialiser( Gyro, Accel, Planedata )
lastplanedata = 0;
% KalmanFilter
[X,P,Q] = initialiseKalman;
T = 0;
iniXtrace = zeros(3000,16);
for k = 1:3000
    T = T + 0.01;
    [X, P] = prediction(X, P, Q, Accel, Gyro, k);
    [X, P , lastplanedata] = update(X,P, Planedata,lastplanedata,T);
    iniXtrace(k,:) = X(1:16);    
end
end

function [X, P, lastplanedata] = processObservation(X, P, Planedata, lastplanedata, T)
 Toffset = Planedata{1,1}(1,1).T / 1000;
    thisplane = lastplanedata + 1;
    if(thisplane <= length(Planedata) && T > Planedata{1,thisplane}(1,1).T / 1000 - Toffset)
        lastplanedata = thisplane;
        XtionCovarFudge = 100;
        for plane_idx = 1:length(Planedata{1,thisplane})
            
            Pstr = Planedata{1,thisplane}(1,plane_idx);
            
            q = X(7:10);
            DCM =  [2*(q(1)^2 + q(2)^2) - 1, 2*(q(2)*q(3) + q(1)*q(4)), 2*(q(2)*q(4) - q(1)*q(3));...
                    2*(q(2)*q(3) - q(1)*q(4)), 2*(q(1)^2 + q(3)^2) - 1, 2*(q(3)*q(4) + q(1)*q(2));...
                    2*(q(2)*q(4) + q(1)*q(3)), 2*(q(3)*q(4) - q(1)*q(2)), 2*(q(1)^2 + q(4)^2) - 1];
            
            Hpermute = [0 0 1 0;...
                        1 0 0 0;...
                        0 1 0 0;...
                        0 0 0 1];
            TransPlane = [DCM      zeros(3,1);...
                          X(1:3).' 1        ];
            inP = Hpermute * Pstr.P;
            inC = Hpermute * Pstr.C * Hpermute.';
            inC = inC .* XtionCovarFudge;
            Xi = [-q(2) -q(3) -q(4); ...
                  q(1) -q(4)  q(3); ...
                  q(4)  q(1) -q(2); ...
                -q(3)  q(2)  q(1)];
              
            Xip = [-q(2) -q(3) -q(4); ...
                   -q(1) -q(4)  q(3); ...
                    q(4) -q(1) -q(2); ...
                   -q(3)  q(2) -q(1)];
       
 
            idx1 = 17;
            idx2 = 20;
            index = -1;
            [m,n] = size(P);
            num = (m-16)/4;
            for i = 1:num
                currTP = X(idx1:idx2);
                
                z = inP - TransPlane * currTP;
                
                Hqparts = Xip * currTP(1:3);
                Hq = 2 .* [-Hqparts(2) -Hqparts(1) Hqparts(4) -Hqparts(3); ...
                           -Hqparts(3) -Hqparts(4) -Hqparts(1) Hqparts(2); ...
                           -Hqparts(4) Hqparts(3) -Hqparts(2) -Hqparts(1)];

                %H = [[-D.*DCM; zeros(1,3)], zeros(4,3), [Hq; zeros(1,4)], zeros(4,3), zeros(4,3), zeros(4,4*(plane_idx-1)), [DCM -DCM*X(1:3); zeros(1,3) 1], zeros(4,4*(n-plane_idx)) ];
                H = [[zeros(3); currTP(1:3).'], zeros(4,3), [Hq; zeros(1,4)], zeros(4,3), zeros(4,3), zeros(4,4*(i-1)), [DCM zeros(3,1); X(1:3).' 1], zeros(4,4*(num-i))];

                S = H*P*H.' + inC;
                mdp = z.'*inv(S)*z;
                mdp = abs(mdp);
                if ( mdp < 1000 )
                   %Data is associated.
%                     mini = i;
                    %mdtrace(thisplane) = md;
                    index = i;
                    break;
                end
                idx1 = idx1 + 4;
                idx2 = idx2 + 4;
            end
            
            %update or register
            if ( index ~= -1) %update
                K = (P*H.') / S;
                X = X + K*z;
                X(7:10) = X(7:10)/norm(X(7:10));
                P = (eye(length(P)) - K*H)*P;
            else
                %add new plane to state and add its cov to P
                InvTransPlane = [DCM.' zeros(3,1);...
                -X(1:3).' * DCM.' 1];
                plane = InvTransPlane * inP;
                X = vertcat(X, plane);
                
                %%%%%%%%%%%%%%%
                %Build P
                GNqparts = Xi * inP(1:3); %use world or body frame here?
                GNq = 2 .* [GNqparts(2) -GNqparts(1) GNqparts(4) -GNqparts(3); ...
                            GNqparts(3) -GNqparts(4) -GNqparts(1) GNqparts(2); ...
                            GNqparts(4) GNqparts(3) -GNqparts(2) -GNqparts(1)];
                Nglob = DCM.' * plane(1:3);
                E_r = [[zeros(3); -Nglob.'] zeros(4,3) [GNq; -X(1:3).' * GNq] zeros(4,3) zeros(4,3)];
                E_y = InvTransPlane;
                Plx = E_r* P(1:16,1:16);
                Pll = E_r*P(1:16,1:16) * E_r.' +  E_y*inC*E_y.';
                P = horzcat (P, zeros(m,4));
                P = vertcat (P, zeros(4,n+4));
                P(m+1:m+4, 1:16) = Plx;
                P(1:16,n+1:n+4) = Plx.';
                P(m+1:m+4, n+1:n+4) = Pll;
            end
        end
    end
end









function [X, P, Q] = initialiseKalman
    
Qgyro = 1.0e-03 * ...
  [ 0.1679   -0.0615   -0.1051;...
   -0.0615    0.1320   -0.0654;...
   -0.1051   -0.0654    0.2536];

Qacc = 1.0e-03 * ...
  [ 0.6670    0.0377   -0.0648;...
    0.0377    0.8222   -0.1422;...
   -0.0648   -0.1422    2.7314];

%Qgyrobias = 1.0e-06 * ...
Qgyrobias = 1.0e-03 * ...
  [ 0.3552   -0.0323   -0.2509;...
   -0.0323    0.2822   -0.2381;...
   -0.2509   -0.2381    0.5686];

%Qaccbias = 1.0e-05 * ...
Qaccbias = 1.0e-02 * ...
  [ 0.3501   -0.1312    0.2149;...
   -0.1312    0.4719   -0.2273;...
    0.2149   -0.2273    1.3670];

Q = blkdiag(Qgyro, Qacc, Qgyrobias, Qaccbias);

X = [-2.2 2.2 2.2, 0 0 0, 0.853553 0.146447 0.353553 -0.353553, -0.0456    0.0069   -0.0048   -0.0331    0.1024    0.1473].'; %-0.0336 0.1013 0.0674].'; %-0.1013 -0.0674 -0.0336].';
%X = [0 0 0, 0 0 0, 1 0 0 0, -0.0456    0.0069   -0.0048   -0.0331    0.1024    0.1473].'; %-0.0336 0.1013 0.0674].'; %-0.1013 -0.0674 -0.0336].';

P0r = eye(3) * 0.3*0.3; %zeros(3);
P0v = zeros(3);
P0q = eye(4) * 0.1*0.1;
P0wb = eye(3) * 0.05*0.05;
P0ab = eye(3) * 1;

P = blkdiag(P0r, P0v, P0q, P0wb, P0ab);
n = 3;
Planes = [-1 0 0 0;...
           0 1 0 0;...
           0 0 1 0];
       
for i = 1:n
    X = [X; Planes(i,:).'];
    P = [P, zeros(length(P), 4);...
         zeros(4, length(P)), zeros(4)];
end
end

function [X, P]= prediction(X, P, Q, Accel, Gyro, k)
    Ts = 0.01;
    wm = Gyro(k,:).' ./ 818.51113590117601252569;
    wm = [wm(3) -wm(1) -wm(2)].';
    am = Accel(k,:).' .* (9.816/(2^14));
    am = [am(3) -am(1) -am(2)].';

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

    n = length(X) - 16;
    n = n/4;
    %Transform X
    Xdot = zeros(16+4*n,1);
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

    P(1:16,1:16) = F*P(1:16,1:16)*F.' + G*Q*G.';
    P(1:16,17:end) = F*P(1:16,17:end);
    P(17:end,1:16) = P(1:16,17:end).';
end

function [X, P , lastplanedata] = update(X, P, Planedata,lastplanedata, T)

    Toffset = Planedata{1,1}(1,1).T / 1000;
    thisplane = lastplanedata + 1;
    if(thisplane <= length(Planedata) && T > Planedata{1,thisplane}(1,1).T / 1000 - Toffset)
        lastplanedata = thisplane;
        XtionCovarFudge = 100;
        for plane_idx = 1:length(Planedata{1,thisplane})
            
            Pstr = Planedata{1,thisplane}(1,plane_idx);
            
            q = X(7:10);
            DCM =  [2*(q(1)^2 + q(2)^2) - 1, 2*(q(2)*q(3) + q(1)*q(4)), 2*(q(2)*q(4) - q(1)*q(3));...
                    2*(q(2)*q(3) - q(1)*q(4)), 2*(q(1)^2 + q(3)^2) - 1, 2*(q(3)*q(4) + q(1)*q(2));...
                    2*(q(2)*q(4) + q(1)*q(3)), 2*(q(3)*q(4) - q(1)*q(2)), 2*(q(1)^2 + q(4)^2) - 1];
            
            Hpermute = [0 0 1 0;...
                        1 0 0 0;...
                        0 1 0 0;...
                        0 0 0 1];
            TransPlane = [DCM      zeros(3,1);...
                          X(1:3).' 1        ];
            inP = Hpermute * Pstr.P;
            inC = Hpermute * Pstr.C * Hpermute.';
            
            %Xi = [-q(2) -q(3) -q(4); ...
            %       q(1) -q(4)  q(3); ...
            %       q(4)  q(1) -q(2); ...
            %     -q(3)  q(2)  q(1)];
              
            Xip = [-q(2) -q(3) -q(4); ...
                   -q(1) -q(4)  q(3); ...
                    q(4) -q(1) -q(2); ...
                   -q(3)  q(2) -q(1)];
       
            %find closest mahal
            idx1 = 17;
            idx2 = 20;
            firsttime = 1;
            n = 3;
            for i = 1:n
                currTP = X(idx1:idx2);
                
                z = inP - TransPlane * currTP;
                
                Hqparts = Xip * currTP(1:3);
                Hq = 2 .* [-Hqparts(2) -Hqparts(1) Hqparts(4) -Hqparts(3); ...
                           -Hqparts(3) -Hqparts(4) -Hqparts(1) Hqparts(2); ...
                           -Hqparts(4) Hqparts(3) -Hqparts(2) -Hqparts(1)];

                %H = [[-D.*DCM; zeros(1,3)], zeros(4,3), [Hq; zeros(1,4)], zeros(4,3), zeros(4,3), zeros(4,4*(plane_idx-1)), [DCM -DCM*X(1:3); zeros(1,3) 1], zeros(4,4*(n-plane_idx)) ];
                H = [[zeros(3); currTP(1:3).'], zeros(4,3), [Hq; zeros(1,4)], zeros(4,3), zeros(4,3), zeros(4,4*(i-1)), [DCM zeros(3,1); X(1:3).' 1], zeros(4,4*(n-i))];

                S = H*P*H.' + inC .* XtionCovarFudge;
                mdp = z.'*inv(S)*z;
            
                if (firsttime == 1 || mdp < md )
                    firsttime = 0;
                    md = mdp;
%                     mini = i;
                    %mdtrace(thisplane) = md;
                    minH = H;
                    minS = S;
                    minz = z;
                end
                idx1 = idx1 + 4;
                idx2 = idx2 + 4;
            end
            
            %TODO: check for md < thresh for reg instead of assoc here!
            %mdtrace(k,plane_idx) = md;
            
            K = (P*minH.') / minS;
            
            X = X + K*minz;
            X(7:10) = X(7:10)/norm(X(7:10));
            P = (eye(length(P)) - K*minH)*P;
        end
    end

end