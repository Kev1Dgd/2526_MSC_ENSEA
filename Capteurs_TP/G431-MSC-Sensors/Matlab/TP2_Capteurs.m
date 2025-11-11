BW = [3600, 250, 100, 20, 5]; % bande-passante
gx_rms = [55.963, 5.452, 0.704, 0.822, 0.828];
gy_rms = [63.229, 8.271, 0.701, 0.680, 0.670];
gz_rms = [64.064, 8.643, 0.221, 0.338, 0.338];

figure;
plot(BW, gx_rms, '-o', 'DisplayName','gx');
hold on;
plot(BW, gy_rms, '-s', 'DisplayName','gy');
plot(BW, gz_rms, '-^', 'DisplayName','gz');
set(gca, 'XScale','log'); % optionnel : échelle log pour mieux visualiser
xlabel('Bande-passante Gyro (Hz)');
ylabel('RMS Gyro (°/s)');
title('Valeur efficace du bruit du gyroscope en fonction de la bande-passante');
legend show;
grid on;


