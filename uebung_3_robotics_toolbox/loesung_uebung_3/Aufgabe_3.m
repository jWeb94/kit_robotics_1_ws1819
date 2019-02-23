clear
clc

% Aufgabe 3

disp('gegebene, initiale Transformation: ')
T0 = [1 0 0 5; 0 1 0 3; 0 0 1 0; 0 0 0 1]

disp('rotation der Transformation um die z-Achse: ')
T01 = trotz(90)

disp('translation in x um 4 einheiten: ')
T12 = transl(4, 0, 0)

disp('translation in (2, 3, 0) und rotation um -45 um die z-Achse: '); 
T23 = transl(2, 3, 0) * trotz(-45)

disp('kombination zu einer Transformation: '); % Wichtig ist die Reihenfolge, da Transformationen nicht kommutativ sind

T_ges = T0 * T01 * T12 * T23
