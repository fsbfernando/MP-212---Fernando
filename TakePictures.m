% captura_realsense_interativo.m
% Loop interativo: publica /capture_realsense (std_msgs/Bool) até o usuário digitar 'n'

clear; clc;

TOPIC = '/capture_realsense';
PAUSE_AFTER_SEC = 6;   % tempo p/ Coppelia salvar arquivos (ajuste se quiser)

% (Re)inicia sessão ROS
try, rosshutdown; end
rosinit;  % assume que o CoppeliaSim (rosInterface) está acessível %[output:25a75ebe]

% Publisher
pub = rospublisher(TOPIC, 'std_msgs/Bool');
msg = rosmessage(pub);

fprintf('Publicador pronto em %s.\n', TOPIC); %[output:89f7e275]

while true %[output:group:06e7ae7b]
    resp = lower(strtrim(input('Tirar foto agora? (s/n): ', 's')));
    if strcmp(resp,'s')
        msg.Data = true;
        send(pub, msg);
        fprintf('🟢 Captura sinalizada em %s.\n', TOPIC); %[output:03ee3ee5]
        pause(PAUSE_AFTER_SEC);  % dá tempo do Coppelia salvar RGB/PLY
    elseif strcmp(resp,'n')
        fprintf('Encerrando.\n');
        break
    else
        fprintf('Entrada inválida. Digite "s" ou "n".\n');
    end
end %[output:group:06e7ae7b]

rosshutdown;


%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright"}
%---
%[output:25a75ebe]
%   data: {"dataType":"text","outputData":{"text":"The value of the ROS_MASTER_URI environment variable, http:\/\/localhost:11311, will be used to connect to the ROS master.\nInitializing global node \/matlab_global_node_36902 with NodeURI http:\/\/iafa-Precision-3660:36803\/ and MasterURI http:\/\/localhost:11311.\n","truncated":false}}
%---
%[output:89f7e275]
%   data: {"dataType":"text","outputData":{"text":"Publicador pronto em \/capture_realsense.\n","truncated":false}}
%---
%[output:03ee3ee5]
%   data: {"dataType":"text","outputData":{"text":"🟢 Captura sinalizada em \/capture_realsense.\n🟢 Captura sinalizada em \/capture_realsense.\n","truncated":false}}
%---
