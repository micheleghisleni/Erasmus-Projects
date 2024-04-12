[yawpks,yawind] = findpeaks(yawRate);
[SApks,SAind] = findpeaks(SteerAng);

freqind = zeros(1,length(SAind));
response = zeros(1,length(SAind));

freq = .63333333*Time.^2;

for i=1:length(SAind)
    freqind(i) = freq(SAind(i))*2*pi;
    response(i) = mag2db(16*yawpks(i)/SApks(i));
end

figure
subplot(311)
plot(Time,yawRate,Time,SteerAng)
subplot(312)
semilogx(freqind,response)
%xlim([1 20])
