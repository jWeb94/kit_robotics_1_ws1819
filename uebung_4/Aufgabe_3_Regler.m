clear
close all
clc

% Aufgabe 3: 
%
% Baue die haendisch hergeleiteten Uebertragungsfunktionen in
% Matlab nach und simuliere die kaskadierte Regelung mit Simulink


% Parameter des elektrischen Systems: 
R_a = 0.235;    % (Ohm) Wicklungswiderstand Stator (Anker) der E-Maschine
L_a = 0.0008;   % (Henry) Wicklungsinduktivitaet

% Parameter des mechanischen Systems: 
K_v     =   0.00074;       % (N*m/rad) Reibungswiderstandskoeffizient der E-Maschine
J_arm   =   5.3933e-05; % (kg*m^2) Traegheitsmoment des Arms um das 
                        % Gelenk (mechanisch muss das Traegheitsmoment um den Schwerpunkt 
                        % des Arms mit dem Satz von Steiner zum Gelenkdehpunkt transformiert werden)
konvert_i_torque = 0.11; 

                        
                        % Uebertragungsfunktion der E-Maschine: 
tf_motor = tf([1], [L_a, R_a]);     % tf([Zaehler], [Nenner]), wobei von links nach Rechts s^(n-1) bis s = s^0 = 1 ist 

% Uebertragungsfunktion des mechanischen Systems: 

tf_arm = tf([1], [J_arm, K_v]); 

%% Bestimme Trajektorie fuer den Roboterarm: 
% Der Code ist zum groessten Teil aus Aufgabe_2_Trajektorie.m uebernommen

% Definiere Gelenke mit den DH-Parametern

linear_gelenk   =   Prismatic('theta', pi/2, 'a', 0, 'alpha', pi/2, 'm', 1, 'r', [0 0 -0.5], 'I', [0.25 0.25 0], 'B', 0, 'G', 0, 'Jm', 0, 'standard')
dreh_gelenk     =   Revolute('d', 0, 'a', 1, 'alpha', 0, 'm', 1, 'r', [-0.5 0 0], 'I', [0.25 0.25 0], 'B', 0, 'G', 0, 'Jm', 0, 'standard') 

% Erstelle den Roboter durch zusammensetzen der zuvor erstellten Gelenken
roboter = SerialLink([linear_gelenk, dreh_gelenk])      % Aufbau der kinematischen Kette durch uebergabe als linear geordnetes Array [Gelenk 0, Gelenk 1, ...]

% Setze Basiskoordinatensystem fuer den Roboter
roboter.base = SE3(0, 0, 0) * SE3.Ry(90); 

% Setze Nullwerte fuer die Robotergelenke (Stellung im Konfigurationsraum, bei der die Parameter als 0 betrachtet)
roboter.links(1).offset = 1;        % Gelenk 1 in der kinematischen Kette
roboter.links(2).offset = pi/2;     % Gelenk 2 in der kinematischen Kette

% Setze Gelenkwinkelgrenzen (Stellgroessenbeschraenkung)
roboter.links(1).qlim = [0 0.5];    % [min max]

% Setze Richtung der Gravitation fest
roboter.gravity = ([0 0 9.81]); 

workspace_roboter = [0 3 -1 1 -0.5 3]; 
plot_options = {'workspace', workspace_roboter}; 
roboter.plotopt = plot_options; 

% Definiere Startpose: 
T1 = transl(2, 0, 0); 
% Definiere Endpose: 
T2 = transl(1.5, 1, 0) * trotz(90); 

% Berechne inverse Kinematik (= Roboterkonfiguration) zu den Posen: 
q1 = roboter.ikine(T1, 'mask', [1 1 0 0 0 0]); 
q2 = roboter.ikine(T2, 'mask', [1 1 0 0 0 0]); 


% Definiere Zeitschrittvektor
T = [0 : 0.05 : 2]'; % Spaltenvektor -> ' steht fuer transponieren

% Lasse die Trajektorie bestimmen: 
[q qd qdd] = mtraj(@lspb, q1, q2, T);

% Definiere Trajektorie fuer die Regelung: 

qdv = [T, qd(:, 2)];    % Zeitschritt in erster Zeile, Geschwindigkeit in zweiter Zeile
                        % (:, 2) bedeutet alle Zeilen der zweiten Spalte
                        % aus dem Vektor qd                        

% Zeige Roboter zur Visualisierung in der Startpose an. Verwende teach um
% damit zu experimentieren
roboter.teach(q(1, :)); 


