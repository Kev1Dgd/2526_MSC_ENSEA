close all;
clear all;

% Chemin vers le fichier CSV
filename = '../data/MeasureBCalib.csv';

% Lire le fichier CSV sans prendre en compte la première ligne
data = csvread(filename, 1, 0);

% Séparer les colonnes
Bx = data(:,1);
By = data(:,2);
Bz = data(:,3);


% === Graphe 3D des points ===

figure;
plot3(Bx, By, Bz, 'o', 'MarkerSize', 6, 'MarkerFaceColor', 'b');
grid on;
xlabel('Bx'); ylabel('By'); zlabel('Bz');
title('Visualisation 3D des mesures B');


% === Courbes 2D des composantes ===

figure;
plot(Bx, 'r', 'LineWidth', 1.5); hold on;
plot(By, 'g', 'LineWidth', 1.5);
plot(Bz, 'b', 'LineWidth', 1.5);
xlabel('Index de mesure');
ylabel('Valeur');
legend('Bx','By','Bz');
title('Évolution des composantes B');
grid on;



