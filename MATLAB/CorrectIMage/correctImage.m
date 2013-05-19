for imcount = 1:samples
    for j = 1:120
        for i = 1:160

               prev = Image{imcount}(j,i);


               for distcount = 1:56
                  diffTemp(distcount) = abs(prev - reverseDist(distcount)); 
               end
               [min1,index1]= min(diffTemp);
               diffTemp(index1) = NaN;
               [min2,index2]= min(diffTemp);
               indexX0 = min(index1,index2);

               newVal = (((stackReverse{j}{i}(indexX0 +1)-stackReverse{j}{i}(indexX0))/(reverseDist(indexX0 +1)-reverseDist(indexX0)))*(prev - reverseDist(indexX0)))+ stackReverse{j}{i}(indexX0);

               Image2{imcount}(j,i) = newVal;
              
        end
    end
    imcount
end