%##This file is used to compare the IMU Gyo

%The IMU data obtained before filter
originalFp=fopen('originalIMU.txt');
%[Gyo,count]=fscanf(fd,'Gyro: %f , %f , %f')
Gyo=fscanf(originalFp,'%f,%f,%f',[3 Inf]);
Gyo=Gyo';
fclose(originalFp);

%calculate the IMU data after filter, here use Kalman filter

filteredFp=fopen('kalmanfilterIMU.txt');
state=fscanf(filteredFp,'%f,%f,%f',[3,Inf]);
state=state';
fclose(filteredFp);

%final IMU data
finalFp=fopen('finalIMU.txt');
finalData=fscanf(finalFp,'%f,%f,%f',[3,Inf]);
finalData=finalData';
fclose(finalFp);

%plot
subplot(2,2,1);plot(Gyo)
%xlable('No. of samples'),ylable('Output')
title('Original Gyo data')
%axis([0,3000,-10,10]);

subplot(2,2,2); plot(state)
%xlable('No. of samples');ylable('Outputs');
title('Kalman filter')
%axis([0,5000,-180,180]);

subplot(2,2,3);plot(finalData)
title('Final data')
%axis([0,5000,-180,180]);