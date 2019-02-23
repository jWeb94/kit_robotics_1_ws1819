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

%% Bis hier hin ist alles gleich zu Aufgabe_2.m

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

% Visualisieren der Trajektorie mit dem Roboter
roboter.plot(q)
hold on; 

% Visualisierung der Trajektorie als Verlauf der Konfigurationsparameter
% ueber die Zeit: 

FK1 = roboter.fkine(q);             % Forwaertskinematikmethode kann auch mit Arrays als Uebergabe arbeiten
p = transl(FK1);                    % Mache daraus ein Translationsobjekt -> (x, y, z)
plot3(p(:, 1), p(:, 2), p(:, 3))    % Mehrfachplot
figure(2)
plot(T, q); 

% Man kann eine polynomielle Interpolation fuer den Weg erkennen -> Rampen
% in der Geschwindigkeit und Spruenge in der Beschleunigung

%% Jacobi Matrix
% 
% Bei Verwendung mit der Vorwaerskinematik lassen sich mit der Jacobi
% Matrix die Endeffektorgeschwindigkeiten im Arbeitsraum fuer eine gegebene Konfiguration
% q und dessen Geschwindigkeiten im Konfigurationsraum berechnen
%
% Analog dazu lassen sich die Kraefte in den Gelenken (tau), bzw die Kraft
% am Endeffektor F zu einer gegebenen Konfiguration berechnen. (Mittels der
% transponieren Jacobi-Matrix) (vgl Kinematik S 52)

figure(4)
konfig_jacobi = [0 pi/4]; 
J3 = roboter.jacob0(konfig_jacobi); % Jacobi-Matrix zur Koniguration d1 = 0 und theta = pi/4
vB3 = J3*[0 pi/2]'; % In gegebener Konfiguration soll mit d1_dot = 0 m/s und theta = 90 Grad/s bewegt werden -> Wie gross ist die Endeffektorgeschwindigkeit?
roboter.plot(konfig_jacobi, 'view', 'top'); 
disp('Geschwindigkeit des Endeffektors in der Konfiguration [0, pi/4]: '); 
vB3

%% Weiteres Beispiel: 

J_test = roboter.jacob0([0 0]); 
figure(3); 
roboter.plot([0 0], 'view', 'top') 
v_test = J_test*[0.5 0]' % Hier kommt logischer Weise 0..5 raus, da ich mich nur in d1 Richtung mit der entsprechenden Geschwindigkeit bewege


%% Recursive Newton Euler Verfahren zum Berechnen der Kraefte und Momente in
%% den Gelenken
roboter.gravity=([9.81 0 0]);   % Gravitation in x-Richtung
qn = [0 0];
qz = [0 0];
%% Berechnen der Joint Torque/Force fuer Position 0 0.
Q = roboter.rne(qn,qz,qz)

%Transpose Jacobi
Fend = [ 3 3 0 0 0 2];       %% Kraftvektor am Ende des Arms -> 3 Newton in x und y und 2 Nm um z-Achse
%% Kraft in den Gelenken erzeugt durch Kraefte und Momente am Ende 
%% des Arms fuer Stellung [0 0];
F = J_test'*Fend' % Kraft in den Gelenken





