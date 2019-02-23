clear
clc

%Aufgabe 1
disp('the given matrix is: '); 
R = [0.36 0.48 -0.8; -0.8 0.6 0; 0.48 0.64 0.6]

p = SO3(R); % Konvertiere Matlab Matrix in SO3 Objekt der Robotics-Toolbox
            % SO3 = Spezieller Orientierungsraum; nur Rotation darstellbar 

disp('roll pitch yaw representation: '); 
p.torpy()
disp('euler angles representation: ')
p.toeul()
disp('angel, rotation axis respresentation: '); 
p.toangvec()
disp('unit quaternion representation: ')
p.UnitQuaternion()
