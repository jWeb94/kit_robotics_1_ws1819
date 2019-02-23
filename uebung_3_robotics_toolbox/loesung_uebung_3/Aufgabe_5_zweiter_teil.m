clear
clc

% Aufgabe 5 zweiter Teil

disp('definiere Rotation (180 Grad) um x-Achse als Quaternion: '); 
a_x = [1 0 0]   % x-Achse als Einheitsvektor
phi_x = 180;    % Rotation um 180 Grad um x

q_rot_x = UnitQuaternion.angvec(phi_x/180*pi, a_x)


disp('definiere Rotation (180 Grad) um y-Achse als Quaternion: ')
a_y = [0 1 0]; 

q_rot_y = UnitQuaternion.angvec(phi_x/180*pi, a_y)

disp('interpolation zwischen den Quaternionen: ');
% Interpolation mit SLERP
% Quaternionen sind gut geeignet um zu interpolieren, da sie kontunierlich
% sind!
q_interpolation = q_rot_y.interp(q_rot_x, 0.5) % 0.5 durch Parameter der SLERP-Interpolation -> war in Aufgabe vorgegeben
