%Sorts planes into bins. Takes 150 samples to compute covariance matrix and
%average plane equation. Returns planes and the covariance matrix.

function all_planes = getcov(fileName)
buffer = load(fileName);

    [L,M] = size(buffer);
    index = 1;
    %Divide Data into four sets. The first 275 PLANES are used to compute 
    %the expected covariance. Then then
    %each
    for i = 1:5: L/2
        plane = buffer(i, 1:4)';
        mycov = buffer(i+1:i+4,1:4);
        if (i < 13) %initialisation
            switch index
                case 1
                    planes.A = plane;
                    cov_planes.A = mycov;
                    plane_stack.A = plane';
                case 2
                    planes.B = plane;
                    cov_planes.B = mycov;
                    plane_stack.B = plane';
                case 3
                    planes.C = plane;
                    cov_planes.C = mycov;
                    plane_stack.C = plane';
            end   
        
        else 
           
            dista = EuclideanDist(plane, planes.A);
            distb = EuclideanDist(plane, planes.B);
            distc = EuclideanDist(plane, planes.C);
            dist = min([dista, distb, distc]);
            if(dist == dista)
                plane_stack.A = vertcat(plane_stack.A, plane');
             
            elseif(dist == distb)
                plane_stack.B = vertcat(plane_stack.B, plane');
                
            elseif(dist == distc)
                plane_stack.C = vertcat(plane_stack.C, plane');
            end
        
        end
        index = index + 1;
    end
    
    
    
    %create the plane equations and covariances for stack A.
    
    [L,M] = size(plane_stack.A);
    for i = 1:150:L
        %account for when data set is less than 150.
        alt_size = L-i;
        if (alt_size < 150)
            [ave, data_cov] = createData(plane_stack.A(i:i+alt_size,:));
            data_cov = data_cov/alt_size;
        else
            [ave, data_cov] = createData(plane_stack.A(i:i+150,:));
            data_cov/150;
        end
        plane_set = struct('plane', ave, 'cov', data_cov);
        if (i == 1)
            data.plane_set(1)= plane_set;
        else
            j = length(data.plane_set) + 1;
            data.plane_set(j) = plane_set;
        end
    end
    
    %create the plane equations and covariances for stack B.
    
    [L,M] = size(plane_stack.B);
    for i = 1:150:L
        %account for when data set is less than 150.
        alt_size = L-i;
        if (alt_size < 150)
            [ave, data_cov] = createData(plane_stack.B(i:i+alt_size,:));
        else
            [ave, data_cov] = createData(plane_stack.B(i:i+150,:));
        end
        plane_set = struct('plane', ave, 'cov', data_cov);
        j = length(data.plane_set) + 1;
        data.plane_set(j) = plane_set;
    end
    
    disp(L);
    disp(length(data.plane_set));
     %create the plane equations and covariances for stack C.
    
    [L,M] = size(plane_stack.C);
    for i = 1:150:L
        %account for when data set is less than 150.
        alt_size = L-i;
        if (alt_size < 150)
            [ave, data_cov] = createData(plane_stack.C(i:i+alt_size,:));
        else
            [ave, data_cov] = createData(plane_stack.C(i:i+150,:));
        end
        plane_set = struct('plane', ave, 'cov', data_cov);
        j = length(data.plane_set) + 1;
        data.plane_set(j) = plane_set;
    end
    
    all_planes = data.plane_set;
    %find out how many covs are singular
end

function [ave,data_cov] = createData(m)
[L,M] = size(m);

ave = m(1,:);
for i= 2:L
    ave = ave + m(i,:);
end
ave = ave/L;
ave = ave';
data_cov = cov(m)';
end

function new_data = addData(data, plane, cov)
index = length(data.landmarks) + 1;

landmark = struct('plane', plane, 'cov', cov);
data.landmarks(index) = landmark;
new_data = data;
end
function dist = EuclideanDist(A, B)
dist = sqrt((A-B)'* (A-B));
end