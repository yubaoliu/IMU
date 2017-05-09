%final IMU data
fp_gyo=fopen('originalIMU.txt');
Gyo=fscanf(fp_gyo,'%f,%f,%f',[3,Inf]);
Gyo=Gyo';
fclose(fp_gyo);


%plot
plot(Gyo)
%xlable('No. of samples'),ylable('Output')
title('Gyo data from unity')
axis([0,5000,-10,10]);
mean(Gyo)%calculate the bias value
