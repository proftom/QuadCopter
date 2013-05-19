gbxNaI(1) = gbxNa(1);
for i = 2:length(Gyrobasex)
    gbxNaI(i) = gbxNaI(i-1) + gbxNa(i);
end