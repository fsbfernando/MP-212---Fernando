%% Enviar uma única pose de juntas (graus) para o KUKA via ROS
close all; clc; clear;

% --- Conexão ROS ---
try, rosshutdown; end %[output:76a57d93]
rosinit;  % conecte ao seu master (ex.: CoppeliaSim rosInterface) %[output:2aef585c]

% --- Publisher: vetor de 7 juntas (Float64MultiArray) ---
pub = rospublisher('/kuka_cmd','std_msgs/Float64MultiArray');
msg = rosmessage(pub);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% === EDITAR AQUI: sua pose desejada EM GRAUS (7 valores) ===
q_deg = [-8 20 0 110 0 -0 -90];

% --- Checagem rápida ---
assert(numel(q_deg)==7, 'Forneça exatamente 7 valores (em graus).');

% --- Converte p/ rad e envia continuamente por alguns segundos ---
q_rad = deg2rad(q_deg);
msg.Data = q_rad;

holdSeconds = 5;      % tempo de streaming (ajuste conforme seu controlador)
dt = 0.05;            % ~20 Hz
r = rateControl(1/dt);

fprintf('Enviando pose fixa por %.1f s a %.0f Hz...\n', holdSeconds, 1/dt); %[output:84be4339]
t0 = tic;
while toc(t0) < holdSeconds
    send(pub, msg);
    waitfor(r);
end
fprintf('Concluído.\n'); %[output:771e7a1d]

% --- (Opcional) Visualização local rápida do modelo ---
%{
iiwa = loadrobot("kukaIiwa14", DataFormat="row");
figure('Name','Pose enviada (visual local)');
show(iiwa, q_rad, 'Frames','off'); view(120,20); axis equal;
%}


%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright"}
%---
%[output:76a57d93]
%   data: {"dataType":"text","outputData":{"text":"Shutting down global node \/matlab_global_node_04937 with NodeURI http:\/\/iafa-Precision-3660:41377\/ and MasterURI http:\/\/localhost:11311.\n","truncated":false}}
%---
%[output:2aef585c]
%   data: {"dataType":"text","outputData":{"text":"The value of the ROS_MASTER_URI environment variable, http:\/\/localhost:11311, will be used to connect to the ROS master.\nInitializing global node \/matlab_global_node_63348 with NodeURI http:\/\/iafa-Precision-3660:41935\/ and MasterURI http:\/\/localhost:11311.\n","truncated":false}}
%---
%[output:84be4339]
%   data: {"dataType":"text","outputData":{"text":"Enviando pose fixa por 5.0 s a 20 Hz...\n","truncated":false}}
%---
%[output:771e7a1d]
%   data: {"dataType":"text","outputData":{"text":"Concluído.\n","truncated":false}}
%---
