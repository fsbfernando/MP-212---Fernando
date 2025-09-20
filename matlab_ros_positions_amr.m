rosshutdown; %[output:3c5bf763]
rosinit; %[output:1e63a43e]

pub = rospublisher('/amr_cmd','std_msgs/Float64MultiArray');
msg = rosmessage(pub);
msg.Data = [-0.10, 0.30];   % Target de deslocamento
send(pub,msg);

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright"}
%---
%[output:3c5bf763]
%   data: {"dataType":"text","outputData":{"text":"Shutting down global node \/matlab_global_node_73200 with NodeURI http:\/\/iafa-Precision-3660:44273\/ and MasterURI http:\/\/localhost:11311.\n","truncated":false}}
%---
%[output:1e63a43e]
%   data: {"dataType":"text","outputData":{"text":"The value of the ROS_MASTER_URI environment variable, http:\/\/localhost:11311, will be used to connect to the ROS master.\nInitializing global node \/matlab_global_node_16293 with NodeURI http:\/\/iafa-Precision-3660:38363\/ and MasterURI http:\/\/localhost:11311.\n","truncated":false}}
%---
