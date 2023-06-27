% compare2signal
function Compare2posesInfo2D(poses,Final)

n=length(poses);

index=1:n;
Initial_rotation=zeros(1,n);
Final_rotation=zeros(1,n);

for i=1:n
    Initial_rotation(i)=wrapToPi(poses(i).R);
    Final_rotation(i)=wrapToPi(Final(i).R);
end 

plot(index,Initial_rotation,index,Final_rotation);


end