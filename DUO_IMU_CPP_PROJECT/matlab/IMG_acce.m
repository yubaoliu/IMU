
%The IMU data obtained before filter
fp=fopen('accelerator.txt');
%[Gyo,count]=fscanf(fd,'Gyro: %f , %f , %f')
acce=fscanf(fp,'%f,%f,%f',[3 Inf]);
acce=acce';
fclose(fp);

plot(acce);
