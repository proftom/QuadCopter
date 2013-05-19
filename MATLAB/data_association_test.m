%Performs data association from data output from the planeSort function.
function associate(data)

    L = length(data);
    state  = createState(data(1));
    for i = 2:L   
            state = data_assoc(data(i), state);
    end
    disp('no of planes');
    disp(length(state.landmarks));
    disp('associations');
    num =0;
    for i = 1:length(state.landmarks)
        disp(state.landmarks(i).count);
    end
end


function new_state = data_assoc(new_data, state)
R = new_data.cov;
new_plane = new_data.plane;
isMatched = 0;
if (isempty(state.landmarks))
    len  = 0;
else
    len = length(state.landmarks);
end
    
for i = 1:len
    landmark = state.landmarks(i);
    old_plane = landmark.plane;
    P = landmark.cov;
    count = landmark.count;
    cov = P + R;
    if (det(R) < 0.000001)
        cov = cov - R;
    end
    if (det(P) < 0.000001)
        cov = cov - P;
    end
    % if cov is all zeroes
    if (det(cov) < 0.00001)
        dist = EuclideanDist(old_plane,new_plane);
         if (abs(dist) < 0.05 )
             disp('Euclidean dist used!');
            %newly observed feature is already part of map
            count = count + 1;        
            %update the number of times that feature has been seen.
            state.landmarks(i).count = count;
            isMatched = 1;
            break;
        end
    else
        dist = mahalanobis_square(old_plane,new_plane, cov);
         if (abs(dist) < 9.49 )
             disp('Maha dist used!');
            %newly observed feature is already part of map
            count = count + 1;        
            %update the number of times that feature has been seen.
            state.landmarks(i).count = count;
            isMatched = 1;
            break;
        end
    end
    
   
end
if (isMatched == 0)
    %newly observed plane maybe be in map. Put in map for now
    % create a  new landmark struct in state
    disp('New Plane Found');
    new_state = addLandmark(state, new_data);
else
    disp('Plane successfully associated!');
    new_state = state;
end
    
end

function nstate = createState(data)
landmark = struct('plane', data.plane, 'cov', data.cov,'count',  1);
landmarks(1) = landmark;
nstate.landmarks = landmarks;

end
function new_state = addLandmark(state, data)
index = length(state.landmarks) + 1;

landmark = struct('plane', data.plane, 'cov', data.cov, 'count', 1);
state.landmarks(index) = landmark;
new_state = state;
end

function dist = EuclideanDist(A, B)
dist = sqrt((A-B)'* (A-B));
end

function lambda = mahalanobis_square(A, B, S) 
%The direction has not yet been accounted for.
lambda = (A-B)'* inv(S) *  (A-B);
end