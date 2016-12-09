
function [rangescan]=LidarScan2test(lidar)
proceed=0;
licznik=0;
% while (proceed==0)
    fprintf(lidar,'GD0044072500');
    pause(0.1);
    fscanf(lidar);
    s=fscanf(lidar);
    s=strcat(s,{' '});
    tag=0;
    tag=fscanf(lidar);
    s=strcat(s,tag);
    
    fscanf(lidar);
    s=strcat(s,{' '});
    s=strcat(s,'6Ge4F');
    q=0;
%     if numel(s)==13
%           proceed=1;
%     end
   %% dla terminatora LF - do tego powyzszszy proceed dla numel(s)=13 
   
           for q=1:32
               s=strcat(s,{' '});
               s=strcat(s,fscanf(lidar));
           end
           s=strcat(s,{' '});
           s=strcat(s,{' '});

   
   
   if numel(s)==2134
        proceed=1;
   end
    w=0;
% end
data=char(s);
i=find(data==data(13));
rangedata=data(i(3)+1:end-1);
for j=0:31
    onlyrangedata((64*j)+1:(64*j)+64)=rangedata(1+(66*j):64+(66*j));
end
    j=0;
    for i=1:floor(numel(onlyrangedata)/3)
    encodeddist(i,:)=[onlyrangedata((3*j)+1) onlyrangedata((3*j)+2) onlyrangedata((3*j)+3)];
    j=j+1;
    end
    for k=1:size(encodeddist,1)
        rangescan(k)=decodeSCIP(encodeddist(k,:));
end





end