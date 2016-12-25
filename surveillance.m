%% %% variables %% %%
sensor_count=6;
uav_count=3;
v_scaling=uav_count^.5;
grid_size=uav_count*10;
velocity=7+5*rand(uav_count,1);
Wo=1./velocity;
uav_v_diff=1+rand(uav_count,1);
distanceof=zeros(uav_count,sensor_count); % distance of ith uav from jth sensor = distanceof(i,j)
parameter=zeros(uav_count,sensor_count);  % to store all the parameters for the sensors for corresponding uavs
I=zeros(uav_count);     % to keep track of the max parameter indices

sensor_pos=((grid_size)*rand(sensor_count,2));                                 

base_station=zeros(uav_count,2);
for n=1:uav_count
    base_station(n,1)=(n-1)*10;
end

uav_pos=base_station;
sensor_age=zeros(sensor_count,1);
residue=zeros(sensor_count,1);
sensor_age(:,1)=randperm(sensor_count);
AGE0=[];
AGE0(:,1)=randperm(sensor_count);
sensor_age(:,1)=AGE0(:,1);
plotter=[];   % For plotting age variations
max_plotter=[];
plot_limit=2500;
%% %% plotting the sensor locations and uavs %% %%

MyPlot=plot(NaN,NaN,'r*');
T=title('SURVEILLANCE');
hold off;
hold on;
for n=1:1:sensor_count
    str=strcat('^.',num2str(n));
    text(sensor_pos(n,1),sensor_pos(n,2),str);
end
% for n=1:1:uav_count
%     str=strcat('.Base',num2str(n));
%     text(base_station(n,1),base_station(n,2),str);
% end
hold on;
plot(sensor_pos(:,1),sensor_pos(:,2),'k.'); % plot sensor locations
plot(base_station(:,1),base_station(:,2),'bO'); % plot Base Station locations

axis('equal');
xlim([-0.1*grid_size 1.1*grid_size]);
ylim([-0.1*grid_size 1.1*grid_size]);

%% %% Surveillance Begins %% %%
g=0;
tic;
while(1)
    g=g+1
    %% %% plot and update the UAV positions %% %%
    set(MyPlot,'Xdata',uav_pos(:,1),'YData',uav_pos(:,2));
    drawnow;

    %% %% revise sensor Ages %% %% 
    
    v=zeros(uav_count,2);
    mod_v=zeros(uav_count,1);
    time=toc;
    
    plotter=[plotter sensor_age]; 
    for i=1:sensor_count
        sensor_age(i,1)=time-residue(i,1)%+AGE0(i,1);
    end
    
    %% %% distance computations %% %%
    for i=1:uav_count          
        for j=1:sensor_count
            dist=sensor_pos(j,:)-uav_pos(i,:);
            distanceof(i,j)= ((dist(1,1))^2+(dist(1,2))^2)^0.5;
        end
    end
              
    
    %% %% velocity calculations %% %%
    
    for i=1:uav_count      
        
       [M,I(i)] = max(parameter(i,:));
       v(i,:)=sensor_pos(I(i),:)-uav_pos(i,:);
       mod_v(i)=(v(i,1)*v(i,1)+v(i,2)*v(i,2))^(0.5);      
    end
    
    v_diff=[uav_v_diff uav_v_diff];
    magnitude_v=[mod_v,mod_v];
    uav_v=v./magnitude_v;                       %% velocity unit vectors
    uav_pos=uav_pos+(uav_v.*[velocity velocity])./(160-10*uav_count);    %% position update for uavs
    time=toc;
    
%% %% Uncomment for path tracing %% %%
%     
%     for i=1:uav_count
%         plot(uav_pos(i,1),uav_pos(i,2),'k.');
%     end
%     
    %% %% parameter computations for sensors %% %%
    
    for i=1:uav_count         
        for j=1:sensor_count
            if uav_count==1
                parameter(i,j)=max([(sensor_age(j)-Wo*distanceof(i,j)) 0]);
            else
                dist2=distanceof(:,j);
                dist2(i)=[];
                W1=Wo;
                W1(i)=[];
                parameter(i,j)=max([(sensor_age(j)-Wo(i)*distanceof(i,j)+min(W1(:).*dist2(:))) 0]);
            end
        end
    end   
    
    %% %% updating sensor Age %% %%
    
    for x=1:uav_count
        if (distanceof(x,I(x))<.25)
            residue(I(x),1)=time%+AGE0(I(x),1);
        end
    end
    
    if g==plot_limit
        %plot_now(g,plotter,sensor_count);
    end
    
end

        