clc; 
close all; 

% Aufgabe 2 - Selbstkonfiguration eines Roboters

% Roboter: Ein Lineargelenk und ein Rotationsgelenk
% DH-Parameter: (Herleitung im Aufschrieb)
%      Gelenk    teta    d      a   alpha
%       1         90     var    0     90
%       2         var    1      0     0

% Definiere Gelenke mit den DH-Parametern

linear_gelenk   =   Prismatic('theta', pi/2, 'a', 0, 'alpha', pi/2, 'm', 1, 'r', [0 0 -0.5], 'I', [0.25 0.25 0], 'B', 0, 'G', 0, 'Jm', 0, 'standard')
dreh_gelenk     =   Revolute('d', 0, 'a', 1, 'alpha', 0, 'm', 1, 'r', [-0.5 0 0], 'I', [0.25 0.25 0], 'B', 0, 'G', 0, 'Jm', 0, 'standard') 

% Durch Weglassen der DH-Parameter werden diese als Bewegungsachse (Lineargelenk), bzw 
% Drehachse (Rotatotisches Gelenk) gewertet
%
% Syntax: 'Parametername', Parameterwert (das ist Matlab Standart)
% 
% Aufschluesselung der Parameter: 
%
% m = Masse des Roboterelements
% r = Vektor zum Massenschwerpunkt (3x1)
% I = Tragheitsmoment
%
% Diese Parameter kommen (zB) aus dem CAD

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

% Visualisierung des Roboter
%   Dazu bedarf es der Konfiguration des Arbeitsraums, in dem der Roboter
%   steht (dessen Abmasse)
workspace_roboter = [0 3 -1 1 -0.5 3]; 
plot_options = {'workspace', workspace_roboter}; 
roboter.plotopt = plot_options; 
roboter.teach([0 0], 'view', 'top'); % [0 0] ist die Startkonfiguration


disp('forwaertskinematik fuer d1 = 0.5 und theta = 90: '); 
fk_test = roboter.fkine([0.5 pi/2])                 % Pose des Roboter TCP 
disp('Eulerwinkel zur TCP-Pose: ')
fk_test.toeul

disp('Inverse Kinematik fuer das Ergebnis der Vorwaertskinematik: ')
ik_test = roboter.ikine(fk_test, 'mask', [1 1 0 0 0 0])
    % Mask bedeutet, dass wir nur eine Loesung in [x, y, z, roll, pitch,
    % yaw] mit jeweils wahrheitswert fuer 1 = true und 0 = false
    % 
    % Das ist notwendig, da wir einen Roboter mit 2 DoF haben. Er kann
    % somit nicht den kompletten 6D Arbeitsraum abdecken. Der Solver der
    % inversen Kinematik sucht aber im kompletten Arbeitsraum und wuerde
    % dann einen Fehler feststellen, da der Roboter die gefundene Loesung
    % nicht abbilden kann!
    
roboter.plot(ik_test)   






