%final IMU data
finalFp=fopen('currentOrientation.txt');
unity=fscanf(finalFp,'%f,%f,%f',[3,Inf]);
unity=unity';
fclose(finalFp);


%plot
subplot(2,2,1);plot(unity)
%xlable('No. of samples'),ylable('Output')
title('Gyo data from unity')
axis([0,5000,-10,10]);
