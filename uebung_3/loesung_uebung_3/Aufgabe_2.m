% Aufgabe 2

disp('the given matrix is: ')
T = [0 0 1 5; 0 1 0 0; -1 0 0 0; 0 0 0 1]
p = SE3(T); % Konvertiere Matlab Matrix is SE3 Objekt -> Spezieller Euler Raum

disp('nur die durch die SE3 Objekt beschriebene Rotation: ')
p.SO3

disp('roll pitch yaw: '); 
p.torpy(); 
disp('roll pitch yaw in grad: ')
p.torpy('deg')

disp('translation: ')
p.t

% Anwenden der Transformation auf einen Vektor
disp('Vektor im kartesischen System: ')
v = [1; 2; 3; 1]    % Erweitert auf affine Transformation
                    % Das muss ein Spaltenvektor sein, da sonst die
                    % Dimensionen zur Matrix-Vektor Multiplikation nicht
                    % passen!

disp('transformation ')
p.T*v; % p.T ist die vollstaendige 4x4 Transformation

disp('inverse der Transformation: '); 
p_inv = inv(p.T)


