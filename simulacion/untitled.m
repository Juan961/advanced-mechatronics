txt = fileread('robot.urdf');
j = regexp(txt, '<joint name="([^"]*)".*?<parent link="([^"]*)"/>\s*<child link="([^"]*)"/>', 'tokens');
j = vertcat(j{:});
T = cell2table(j, 'VariableNames', {'joint','padre','hijo'})

%%

robot = importrobot('robot.urdf');
showdetails(robot)   % deben salir ~6 cuerpos con sus joints revolute
show(robot);         % vista rápida, sin Simscape

%%
% 1. Rutas absolutas para las mallas
carpeta = strrep(pwd, '\', '/');
txt = fileread('robot.urdf');
txt = regexprep(txt, 'package://', '');
txt = regexprep(txt, 'filename="([^"]*)"', 'filename="${strrep($1,''\'',''/'')}"');
txt = regexprep(txt, 'filename="(?![A-Za-z]:|/)', ['filename="' carpeta '/']);
fid = fopen('robot_abs.urdf','w'); fwrite(fid, txt); fclose(fid);

% 2. Verificación
robot = importrobot('robot_abs.urdf');
showdetails(robot)
show(robot);
masas = cellfun(@(b) b.Mass, robot.Bodies)

% 3. Importar a Simscape
smimport('robot_abs.urdf', 'ModelName', 'brazo_planta');

%% 
% Revisa los STL de la escena: tamaño, posición y alineación con el robot.
% Poner los STL de la escena (banda, rodillos, plantilla, pieza) en la
% subcarpeta brazo/escena/ y correr este script desde la carpeta brazo.

ESCALA = 1/1000;   % 1/1000 si exportaste en mm, 1 si exportaste en metros

robot = importrobot('robot_abs.urdf');
robot.DataFormat = 'row';

archivos = dir(fullfile('escena', '*.stl'));
figure('Name','Escena + robot');
show(robot, zeros(1, numel(robot.Bodies)), 'Frames','off');
hold on

fprintf('%-28s %22s %22s %22s\n', 'Archivo', 'X [min max]', 'Y [min max]', 'Z [min max]');
for i = 1:numel(archivos)
    TR = stlread(fullfile('escena', archivos(i).name));
    P  = TR.Points;
    fprintf('%-28s [%9.1f %9.1f] [%9.1f %9.1f] [%9.1f %9.1f]\n', archivos(i).name, ...
        min(P(:,1)), max(P(:,1)), min(P(:,2)), max(P(:,2)), min(P(:,3)), max(P(:,3)));
    P = P * ESCALA;
    trisurf(TR.ConnectivityList, P(:,1), P(:,2), P(:,3), ...
        'EdgeColor','none', 'FaceAlpha',0.6, 'FaceColor', rand(1,3));
end

axis equal; grid on; camlight; view(3);
title('¿La escena queda bien ubicada respecto al robot?');
