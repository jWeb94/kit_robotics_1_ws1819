clear
clc

% Aufgabe 4

disp('pose des TCP: '); 
T_tcp = [0 -1 0 1; 1 0 0 2; 0 0 1 3; 0 0 0 1];
p_tcp = SE3(T_tcp)

disp('ziel pose: ');
T_goal = [0 0 -1 7; 0 1 0 6; 1 0 0 5; 0 0 0 1];
p_goal = SE3(T_goal)


disp('translationsdifferenz: '); 
d_t = p_goal.t - p_tcp.t
disp('betrag des euklidischen Abstands: ');
norm(d_t)

disp('Rotationsdifferenz: '); 
R_diff = inv(p_tcp.SO3) * p_goal.SO3
disp('Rotationsdifferenz in Angle-Vektor-Darstellung (Winkel in Grad): ')
R_diff.toangvec('deg')
