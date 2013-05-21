%Sorts planes into bins. Displays a mahalanobis histogram.

function distHist(data)

    L = length(data);
    % Create bins
    planes.A = data{1,1}(1,1);
    planes.B = data{1,1}(1,2);
    planes.C = data{1,1}(1,3);
    
    %Compute Lengths first
    indexA = 1;
    indexB = 1;
    indexC = 1;
    
    for i = 2:L
        L2 = length(data{1,i});
        if (L2 < 4)
            for j = 1:L2
                plane = data{1,i}(1,j).plane;
    %             mycov = data{1,i}(1,j).cov;
                dista = EuclideanDist(plane, planes.A.plane);
                distb = EuclideanDist(plane, planes.B.plane);
                distc = EuclideanDist(plane, planes.C.plane);
                dist = min([dista, distb, distc]);
                if(dist == dista)
                    % a stack is a struct containing plane, cov, timestamp, etc
                    plane_stack.A(indexA) = data{1,i}(1,j);
                    indexA = indexA + 1;

                elseif(dist == distb)
                    plane_stack.B(indexB) = data{1,i}(1,j);
                    indexB = indexB + 1;

                elseif(dist == distc)
                    plane_stack.C(indexC) = data{1,i}(1,j);
                    indexC = indexC + 1;
                end        
            end
        end
    end
    
%     
%     %Get sizes of stacks
%     sizeA = indexA - 1;
%     sizeB = indexB - 1;
%     sizeC = indexC - 1;
%     
%     
%     plane_stack.A = cell([1, sizeA]);
%     plane_stack.B = cell([1, sizeB]);
%     plane_stack.C = cell([1, sizeC]);
%     
%     % Sort Planes into Bins
%     indexA = 2;
%     indexB = 2;
%     indexC = 2;
%     
%     for i = 2:L
%         L2 = length(data{1,i});
%         if (L2 < 4)
%             for j = 1:L2
%                 plane = data{1,i}(1,j).plane;
%     %             mycov = data{1,i}(1,j).cov;
%                 dista = EuclideanDist(plane, planes.A.plane);
%                 distb = EuclideanDist(plane, planes.B.plane);
%                 distc = EuclideanDist(plane, planes.C.plane);
%                 dist = min([dista, distb, distc]);
%                 if(dist == dista)
%                     % a stack is a struct containing plane, cov, timestamp, etc
%                     plane_stack.A{1,indexA} = data{1,i}(1:L2);
%                     indexA = indexA + 1;
% 
%                 elseif(dist == distb)
%                     plane_stack.B{1, indexB} = data{1,i}(1:L2);
%                     indexB = indexB + 1;
% 
%                 elseif(dist == distc)
%                     plane_stack.C{1,indexC} = data{1,i}(1:L2);
%                     indexC = indexC + 1;
%                 end        
%             end
%         end
%     end
    
    
    %create mahalanobis distance array
    
    %For stack A
    
    mahaA = [];
    for i = 1:indexA-1
        C = planes.A.cov + plane_stack.A(i).cov;
        mahaA = horzcat(mahaA, mahalanobis_square(plane_stack.A(i).plane, planes.A.plane, C));
    end
    
    %For stack B
    
    mahaB = [];
    for i = 1:indexB-1
        C = planes.B.cov + plane_stack.B(i).cov;
        mahaB = horzcat(mahaB, mahalanobis_square(plane_stack.B(i).plane, planes.B.plane, C));
    end
    
    %For stack C
    
    mahaC = [];
    for i = 1:indexC-1
        C = planes.C.cov + plane_stack.C(i).cov;
        mahaC = horzcat(mahaC, mahalanobis_square(plane_stack.C(i).plane, planes.C.plane, C));
    end
    
  
    
    %Compute and display histograms.
    figure;
    hist(mahaA, 50);
    title('Mahalanobis Distances for Plane A Samples');
    
    figure;
    hist(mahaB, 50);
    title('Mahalanobis Distances for Plane B Samples');
    
    figure;
    hist(mahaC, 50);
    title('Mahalanobis Distances for Plane C Samples');
end



function lambda = mahalanobis_square(A, B, S) 
%The direction has not yet been accounted for.
lambda = (A-B)'* inv(S) *  (A-B);
end

function dist = EuclideanDist(A, B)
dist = sqrt((A-B)'* (A-B));
end