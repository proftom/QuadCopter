fid = fopen('notmoving3corners2.2.txt');

nument = 0;
tline = fgetl(fid);
while ischar(tline)
   matches = strfind(tline, '<<<');
   num = length(matches);
   if num > 0
      nument = nument + num;
      %fprintf(1,'%d:%s\n',num,tline);
   end
   tline = fgetl(fid);
end

frewind(fid);

Planedata = cell([1,nument]);

for t = 1:nument
    N = fscanf(fid, '%d\n', 1);
    
    P = cell([1,N]);
    C = cell([1,N]);
    pts = cell([1,N]);
    for p = 1:N
        P{p} = fscanf(fid, '%f', 4);
        C{p} = fscanf(fid, '%f', [4,4]);
        pts{p} = fscanf(fid, '%d\n', 1);
    end

    T = fscanf(fid, '%d\n', 1);
    delim = fscanf(fid, '%s\n', 1);
    
    Planedata{t} = struct('T', T, 'P', P, 'C', C, 'pts', pts);
end

fclose(fid);