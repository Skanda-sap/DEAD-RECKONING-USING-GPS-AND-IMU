%%
clc
clear all
close all
format long;

bagselect= rosbag("fusion.bag");
rosbag info fusion.bag;
bSel = select(bagselect,'Topic','imu_message');
bSel1 = select(bagselect,'Topic','mag_message');
bSel2 = select(bagselect,'Topic','imu_raw');
bSel3 = select(bagselect, 'Topic','gps_message');

msgStructs = readMessages(bSel,'DataFormat','struct');
msgStructs{1};
msgStructs1 = readMessages(bSel1,'DataFormat','struct');
msgStructs1{1};
msgStructs2 = readMessages(bSel2,'DataFormat','struct');
msgStructs2{1};
msgStructs3 = readMessages(bSel3,'DataFormat','struct');
msgStructs3{1};

mag_x=cellfun(@(m) double(m.MagneticField_.X), msgStructs1);
mag_y=cellfun(@(m) double(m.MagneticField_.Y), msgStructs1);
mag_z=cellfun(@(m) double(m.MagneticField_.Z), msgStructs1);

timepoints_index_mag =cellfun(@(m) int64(m.Header.Seq),msgStructs1);
timepoints_index_mag = timepoints_index_mag - min(timepoints_index_mag);

Angular_velx=cellfun(@(m) double(m.AngularVelocity.X), msgStructs);
Angular_vely=cellfun(@(m) double(m.AngularVelocity.Y), msgStructs);
Angular_velz=cellfun(@(m) double(m.AngularVelocity.Z), msgStructs);

linearaccel_x=cellfun(@(m) double(m.LinearAcceleration.X), msgStructs);
linearaccel_y=cellfun(@(m) double(m.LinearAcceleration.Y), msgStructs);
linearaccel_z=cellfun(@(m) double(m.LinearAcceleration.Z), msgStructs);

yaw=cellfun(@(m) double(m.Yaw), msgStructs2);
pitch=cellfun(@(m) double(m.Pitch), msgStructs2);
roll=cellfun(@(m) double(m.Roll), msgStructs2);
magx=cellfun(@(m) double(m.Magx), msgStructs2);
magy=cellfun(@(m) double(m.Magy), msgStructs2);
magz=cellfun(@(m) double(m.Magz), msgStructs2);
accx=cellfun(@(m) double(m.Accx), msgStructs2);
accy=cellfun(@(m) double(m.Accy), msgStructs2);
accz=cellfun(@(m) double(m.Accz), msgStructs2);
gyrox=cellfun(@(m) double(m.Gyrox), msgStructs2);
gyroy=cellfun(@(m) double(m.Gyroy), msgStructs2);
gyroz=cellfun(@(m) double(m.Gyroz), msgStructs2);

timepoints_index_imu = 1:51247;

lat=cellfun(@(m) double(m.Latitude), msgStructs3);
long=cellfun(@(m) double(m.Longitude), msgStructs3);
alt=cellfun(@(m) double(m.Altitude), msgStructs3);
utm_east=cellfun(@(m) double(m.UtmEasting), msgStructs3);
utm_north=cellfun(@(m) double(m.UtmNorthing), msgStructs3);
zone=cellfun(@(m) double(m.Zone), msgStructs3);
letter=cellfun(@(m) double(m.Letter), msgStructs3);

timepoints_index_gps = 1:1282;


figure
plot(utm_east,utm_north)
title("UTM EASTING VS UTM NORTHING")
xlabel("UTM EASTING in Meters")
ylabel("UTM NORTHING in Meters")
grid on
figure
scatter(magx(8000:10636),magy(8000:10636))
title("Magnetometer data before correction")
xlabel("Magx in Guass")
ylabel("Magy in Guass")
grid on 

axis equal
%%
% Hard and Soft Iron Correction Section
% Hard Iron Correction - starting from 8000 to 10636
for i=1:length(mag_x)
    magxyz_tilt_cor= [mag_x(i);mag_y(i);mag_z(i)];
    magx_tilt_cor(i,1)=magxyz_tilt_cor(1);
    magy_tilt_cor(i,1)=magxyz_tilt_cor(2);
    magz_tilt_cor(i,1)=magxyz_tilt_cor(3);
end

alpha=max(magx(8000:10636)+min(magx(8000:10636)))/2
beta=max(magy(8000:10636)+min(magy(8000:10636)))/2
gama=max(magz(8000:10636)+min(magz(8000:10636)))/2
magx_hard_corrected=magx_tilt_cor -alpha
magy_hard_corrected=magy_tilt_cor -beta
magz_hard_corrected=magz_tilt_cor -gama

figure
scatter(magx_hard_corrected(8000:10636), magy_hard_corrected(8000:10636))
title("Hard Iron Correction of Magx and MagY")
xlabel("Magx in Guass")
ylabel("Magy in Guass")



% Soft Iron Correction
r=sqrt((magx_hard_corrected(8000:10636)).^2 + (magy_hard_corrected(8000:10636)).^2)
[M,I]=max(r)
thetax=asin(magy_hard_corrected(8005)/magx_hard_corrected(8005))
soft=[cos(thetax) sin(thetax); -sin(thetax) cos(thetax)];

for i=1:length(magx_hard_corrected)
    magxy_soft_correction= soft*[magx_hard_corrected(i); magy_hard_corrected(i)];
    magx_soft_correction(i,1)=magxy_soft_correction(1);
    magy_soft_correction(i,1)=magxy_soft_correction(2);
end
figure
plot(magx_soft_correction(8000:10636), magy_soft_correction(8000:10636))
grid on
axis equal
xlabel("Magx in Guass")
ylabel("Magy in Guass")
hold on
plot(magx(8000:10636),magy(8000:10636))
grid on 

% Shrinking to form circle
q=0.087;
r=0.12;
sigma=q/r;
magx_soft_correction=magx_soft_correction*sigma;
softs=[cos(-thetax) sin(-thetax); -sin(-thetax) cos(-thetax)];
for i=1:length(magx_soft_correction)
    magxy_soft_correction= softs*[magx_soft_correction(i); magy_soft_correction(i)];
    magx_soft_correction(i,1)=magxy_soft_correction(1);
    magy_soft_correction(i,1)=magxy_soft_correction(2);
end

hold on
plot(magx_soft_correction(8000:10636), magy_soft_correction(8000:10636))
title("Magnetometer data")
grid on
axis equal 
plot(magx(8000:10636),magy(8000:10636))
legend("Originial Data","Hard iron correction ","Soft iron correction")
xlabel("Magx in Guass")
ylabel("Magy in Guass")
grid on


%%
% Yaw from magnetometer and gyroscope section

yaw1=deg2rad(yaw);
yaw1=unwrap(yaw1);

yaw_mag=atan2(-magy_soft_correction,magx_soft_correction);
yaw_mag=unwrap(yaw_mag);
yaw_mag_scaled=yaw_mag;
yaw_mag_scaled=yaw_mag_scaled-3.56;
figure
plot(timepoints_index_imu,yaw_mag_scaled, "r")
hold on

Angular_velz1=Angular_velz;
yaw_gyro=cumtrapz(double(timepoints_index_imu), Angular_velz1)/40;
yaw_gyros=unwrap(yaw_gyro);
hold on
plot(timepoints_index_imu,yaw_gyros,"b")
title("Magnetometer vs Yaw Integrated from Gyro")
xlabel("Time in Seconds")
ylabel("Yaw in Radians")
legend("Yaw from Magnetometer scaled after correction","Yaw from Gyro scaled")
grid on
hold off

%%
%complimentary filter section
tau = 0.025;
alpha = tau/(tau+0.025);

yaw_from_mag_filtered(1,1) = yaw_mag_scaled(1,1);

for n = 2: length(yaw_mag_scaled)
    yaw_from_mag_filtered(n,1)=(1-alpha)*yaw_mag_scaled(n,1) + alpha*yaw_from_mag_filtered(n-1,1);    
end
tau = 0.00000001;
alpha = tau/(tau+0.025);

yaw_from_gyro_filtered(1,1) = yaw_gyros(1,1);

for n = 2:length(yaw_gyros)
    yaw_from_gyro_filtered(n,1) = (1-alpha)*yaw_from_gyro_filtered(n-1,1) + (1-alpha)*(yaw_gyros(n,1) - yaw_gyros(n-1,1));
end
alpha = 0.1;

final_yaw = alpha*yaw_from_mag_filtered + (1-alpha)*yaw_from_gyro_filtered;

figure
hold on
plot(timepoints_index_imu,yaw_from_mag_filtered,'DisplayName','Magnetometer Heading Filtered','Color','red')
hold on
plot(timepoints_index_imu,yaw_from_gyro_filtered,'DisplayName','Gyro Heading Filtered (integrated)','Color','blue')
hold on
plot(timepoints_index_imu,final_yaw,'DisplayName','Final Headings Estimate','Color','black')
hold on
plot(timepoints_index_imu, yaw1,'DisplayName','Yaw-IMU','Color','green')
legend
xlabel("Time in Seconds")
ylabel("Yaw in Radians")
grid on
%%
%Forward velocity section

new_linx=linearaccel_x-1.5*linearaccel_x
imu_velocity(13000,1)=0;
for i = 13000:length(timepoints_index_imu)
    imu_velocity(i,1) = imu_velocity(i-1,1) + new_linx(i,1)/400;

end

dt = 1;

for i = 2:length(utm_east)
    delta_easting = (utm_east(i) - utm_east(i-1));
    delta_northing = (utm_north(i)- utm_north(i-1));
    delta(i) = sqrt(delta_easting^2 + delta_northing^2);
end

for i = 1:length(utm_north)
    for j = 1:40
        velocity_gps(40*(i-1) + j) = delta(i);
    end
end

figure
plot(timepoints_index_imu, imu_velocity, 'DisplayName','Estimate of Imu velocity', "color","Red")
hold on

plot(timepoints_index_imu, velocity_gps(1:51247),'DisplayName','Estimate of GPS velocity', "color", "blue")
legend
xlabel("Time in seconds")
ylabel("Velocity in m/s")
hold on

LinearAcceleration_x_adjusted = new_linx;
LinearAcceleration_x_adjusted = LinearAcceleration_x_adjusted - 0.291;

time_window = 5;
time_window = time_window * 40;
LinearAcceleration_x_filtered = LinearAcceleration_x_adjusted;

for i = 0:length(LinearAcceleration_x_filtered)-time_window
    set_acc_zero = 1;
    for j = 1:time_window
        if abs(LinearAcceleration_x_filtered(i+j,1)) <= 0.5
            set_acc_zero = set_acc_zero*1; 
        else
            set_acc_zero = set_acc_zero*0; 
        end
    end
    if set_acc_zero == 1
        for j = 1:time_window
            LinearAcceleration_x_filtered(i+j,1) = 0;
        end
    end
end

%1300 is a approximate point where the circle has ended 
velocity_imu_new(13000,1) = 0; 
for i = 13000+1:length(timepoints_index_imu)
    velocity_imu_new(i,1) = velocity_imu_new(i-1,1) - LinearAcceleration_x_filtered(i,1)/40;
end


figure

plot(timepoints_index_imu(13000:length(magx)),velocity_imu_new(13000:length(magx)),'DisplayName','IMU Velocity new','Color','red')
title('Velocity Estimate after acceleration adjustment')
xlabel('Time in Seconds')
ylabel('Velocity (m/s)')

hold on
plot(timepoints_index_imu,velocity_gps(1:51247),'DisplayName','GPS Velocity estimate','Color','blue')
legend

hold on
grid on
omega_x_dot = Angular_velz.*velocity_imu_new;
figure;
plot(timepoints_index_imu,omega_x_dot,'.','DisplayName','Omega x dot','Color','green')
title('Omega x dot and Y double dot vs time','Interpreter','latex')
xlabel('Time in Seconds')
ylabel('Acceleration (m/s^2)')
grid on
legend

hold on
plot(timepoints_index_imu,linearaccel_y,'.','DisplayName','Y double dot','Color','red')
%%
% Dead Reckoning section

utm_east=utm_east-utm_east(1);
utm_north=utm_north-utm_north(1);
figure
plot(utm_east,utm_north,'DisplayName','GPS Plot','Color','red')
title('UTM Easting vs UTM Northing')
xlabel('UTM Easting (m)')
ylabel('UTM Northing (m)')
grid on
legend

distance_from_imu = cumtrapz(timepoints_index_imu,velocity_imu_new);

Angular_velx(1,1) = 0;
Angular_vely(1,1) = 0;
correction_angle = -250*pi/180;
clear heading_mags;
heading_mags = yaw_mag_scaled;
heading_mags(24180:length(heading_mags),1) = heading_mags(24180:length(heading_mags),1)-40*pi/180;

for i = 24180:length(yaw_from_mag_filtered) 
      Angular_velx(i,1) = Angular_velx(i-1,1) + (distance_from_imu(i) - distance_from_imu(i-1))*cos(unwrap(heading_mags(i))+ correction_angle);
      Angular_vely(i,1) = Angular_vely(i-1,1) + (distance_from_imu(i) - distance_from_imu(i-1))*-sin(unwrap(heading_mags(i))+ correction_angle); 
end

heading_mags(36000:length(heading_mags),1) = heading_mags(36000:length(heading_mags),1)+60*pi/180;

for i = 36000:length(yaw_from_mag_filtered) 
      Angular_velx(i,1) = Angular_velx(i-1,1) + (distance_from_imu(i) - distance_from_imu(i-1))*cos(unwrap(heading_mags(i))+ correction_angle);
      Angular_vely(i,1) = Angular_vely(i-1,1) + (distance_from_imu(i) - distance_from_imu(i-1))*-sin(unwrap(heading_mags(i))+ correction_angle); 
end 
heading_mags(45000:length(heading_mags),1) = heading_mags(45000:length(heading_mags),1)-45*pi/180;

for i = 45000:length(yaw_from_mag_filtered) 
      Angular_velx(i,1) = Angular_velx(i-1,1) + (distance_from_imu(i) - distance_from_imu(i-1))*cos(unwrap(heading_mags(i))+ correction_angle);
      Angular_vely(i,1) = Angular_vely(i-1,1) + (distance_from_imu(i) - distance_from_imu(i-1))*-sin(unwrap(heading_mags(i))+ correction_angle); 
end 
% 
Angular_velx = Angular_velx - Angular_velx(1);
Angular_vely = Angular_vely - Angular_vely(1);

imu__x_scaled = Angular_velx*(0.03)
imu_y_scaled = Angular_vely*(0.03)


hold on
plot(imu__x_scaled*0.9,imu_y_scaled*0.9,'DisplayName','Dead Reckoning','Color','blue')
%%
%calculating xc section
omega_dot(1,1) = 0;
for i = 2:length(Angular_velz)
    omega_dot(i,1) = (Angular_velz(i) - Angular_velz(i-1))*40;
end


xc = (linearaccel_y - omega_x_dot)./omega_dot;
mean(xc(400:1000))
