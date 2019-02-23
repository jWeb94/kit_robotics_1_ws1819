clear
clf
clc

% Aufgabe 1: 
% puma560 Roboter kennen lernen und mit den wichtigsten Befehlen vertraut
% machen

mdl_puma560;    % Lade den Roboter aus der Toolbox in den Workspace
                % Das ist ein Objekt vom Typ SerialLink (Datentyp der Robotics Toolbox)

p560            % zeige die DH-Parameter des Roboters an
                % Der Roboter hat 6 DoF (6 Axis in der Beschreibung)
                % Zustaende, die dem Roboter uebergeben werden, muessen in 
                % Zustandsvektoren mit 6 Eintraegen uebergeben werden
                
disp('Das ist die Beispielkonfiguration q: ');                 
q = [0 0 0 0 0 0]

p560.plot(q);   % Zeige Roboter in Konfiguration q an
                % Jedes SerialLink Objekt hat eine Methode plot, mit der
                % man den Roboter visualisieren kann!
                
disp('Druecke enter fuer teach Funktion!');                
waitforbuttonpress;                 
                
p560.teach(q);  % Jedes SerialLink Objekt hat eine Methode teach, mit der man 
                % die Konfigurationsparameter in einer GUI veraendern kann
                % und der TCP angezeigt wird. 
                % Bewegen des Roboters mit der GUI ist quasi
                % Vorwaertskinematik auswerten!
        
test_value =  1%input('Wollen Sie in der teach-Funktion bleiben, oder fortfahren? (0 = bleiben; 1 = fortfahren)'); 
if test_value == 0
    disp('teach-Modus (kann in Zeile 33 veraendert werden)');
    return; 
elseif (test_value ~= 0) || (test_value ~= 1)
        disp('Falsche Eingabe. Das Skript wird abgebrochen!'); 
end; 

disp('trajektory-Modus (kann in Zeile 33 veraendert werden)');

disp('Vorwaertskinematik fuer Konfiguration q: ')                
p560.fkine(q)   % Man kann sehen, dass keine Rotation, aber Translation entsteht
                % Die Robotics Toolbox nimmt die Berechnung der
                % Vorwaertskinematik dem Nutzer ab. Er muss nur die
                % DH-Parameter selbst bestimmen. Der Rest wird uebernommen
                
disp('Berechne inverse Kinematik des Roboters fuer eine Pose: ');
disp('Pose: ')
T_tcp_soll = transl(0.7, 0.3, 0) * troty(90)    % Translation (x, y, z) = (0.7, 0.3, 0) 
                                                % kombiniert mit
                                                % anschliessender Rotation
                                                % um die y-Achse um 90 Grad

                                                
disp('Die inverse Kinematik (der zur Pose des Endeffektors gehoerende Konfigurationsparametersatz) ist: '); 
q_soll_pose = p560.ikine(T_tcp_soll)      % 6 Spalten Vektor fuer 6 Konfigurationsparameter

disp('zeige druecke enter, um die Konfiguration anzuzeigen!'); 
waitforbuttonpress; 
p560.plot(q_soll_pose); 

disp('Druecke enter, um die Trajektorie von q zu q_soll_pose in der Visualisierung abzufahren!'); 
waitforbuttonpress; 

% Zeitschrittweite
t = [0 : 0.05: 2]; 

% Interpoliere zwischen den Posen
disp('Das ist die Trajektorie in diskreten Werten in der Formatierung (q, qd, qdd): '); 
[q_traj, qd_traj, qdd_traj] = mtraj(@lspb, q, q_soll_pose, t)

% plot ist ueberladen, sodass wenn ich einen Vektor uebergebe, die
% Trajektorie vom Roboter im Plot abgefahren wird

p560.plot(q_traj); 





                



