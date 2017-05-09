%The IMU data obtained before filter
originalFp=fopen('./originalIMU.txt');
%[Gyo,count]=fscanf(fd,'Gyro: %f , %f , %f')
Gyo=fscanf(originalFp,'%f,%f,%f',[3 Inf]);
Gyo=Gyo';
fclose(originalFp);


%plot
plot(Gyo)
%xlable('No. of samples'),ylable('Output')
title('Original Gyo data')
axis([0,3000,-10,10]);

%Calculate the bias error
bias=mean(Gyo)
