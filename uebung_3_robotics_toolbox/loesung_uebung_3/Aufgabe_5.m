clear
clc

% Aufgabe 5: Quaternionen

disp('position kartesisch: ')
p = [5 1 7]
disp('position als Quaternion: ')
v = Quaternion(0, p)    % Punkte als Quaternion darstellen, in dem man den Realanteil 0 setzt und den Vektor
                        % Komponentenweise in die Imagenaeranteile schreibt
                        

disp('Transformation als Quaternion dargestellt: Verdrehung um 90 Grad um die Z-Achse'); 
phi = 90;       % Grad
a = [0 0 1];    % Drehachse -> z-Achse

q = UnitQuaternion.angvec(phi/180*pi, a)    % Einheitsquaternion 
                                            % Einheitsquaternionen stellen
                                            % Transformationen dar!
                                            % In der Form: 
                                            % q_rot = (cos(phi/2), a*sin(phi/2))
                                            % Wobei abs(a) = 1 (Drehachse)

disp('konjugiertes Quaternion, um die Rotation auf ein anderes Quaternion anwenden zu koennen: '); 
% Quaternion als Transformation anwenden: 
% v_transformiert = q*v*q_konj -> q, v, q_conj als Quaternionen 
q_conj = q.conj()

disp('transformiertes Quaternion: ')
v_transformiert = q*v*q_conj
