%Deve atualizar essa função sempre que fizer outro processo de rotulação na
%RoboFlow: por exemplo, se gerou nova base de dados com multiclasses em que
%uma única imagem tem 10 exemplos de objetos a serem detectados.
%Dados de entrada:
%          1. nomedaPasta = 'CableShark_MultiClass.v1i.yolov4pytorch_Multiclass'; %Sempre atualizar o nome de acordo com o nome da pasta da RoboFlow
%          2. op = 'train' ou 'valid' ou 'test'

function [clsobj,annotations]=readDadosFilePytorch(nomedaPasta,op)
pastaRoboFlow = fullfile(pwd,nomedaPasta); 
addpath(genpath(pastaRoboFlow))

%Importa os dados do arquivo de anotação da roboFlow (_annotations.txt) - Gerar o trecho do Script pelo importaDados do Matlab
%Set up the Import Options and import the data
opts = delimitedTextImportOptions("NumVariables", 46);
% Specify range and delimiter
opts.DataLines = [1, Inf];
opts.Delimiter = [" ", ","];
% Specify column names and types
opts.VariableNames = ["imageFileName", "VarName2", "VarName3", "VarName4", "VarName5", "VarName6", "VarName7", "VarName8", "VarName9", "VarName10", "VarName11", "VarName12", "VarName13", "VarName14", "VarName15", "VarName16", "VarName17", "VarName18", "VarName19", "VarName20", "VarName21", "VarName22", "VarName23", "VarName24", "VarName25", "VarName26", "VarName27", "VarName28", "VarName29", "VarName30", "VarName31", "VarName32", "VarName33", "VarName34", "VarName35", "VarName36", "VarName37", "VarName38", "VarName39", "VarName40", "VarName41", "VarName42", "VarName43", "VarName44", "VarName45", "VarName46"];
opts.VariableTypes = ["string", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double"];
% Specify file level properties
opts.ExtraColumnsRule = "ignore";
opts.EmptyLineRule = "read";
% Specify variable properties
opts = setvaropts(opts, "imageFileName", "WhitespaceRule", "preserve");
opts = setvaropts(opts, "imageFileName", "EmptyFieldRule", "auto");
% Import the data
annotations = readtable(fullfile(pastaRoboFlow,strcat('/',op,'/_annotations.txt')), opts);
% Clear temporary variables
clear opts
%%Até aqui importação automática do Matlab

%Importa os dados do arquivo de classes da roboFlow (_classes.txt) - Gerar o trecho do Script pelo importaDados do Matlab
%%Set up the Import Options and import the data
opts = delimitedTextImportOptions("NumVariables", 1);
% Specify range and delimiter
opts.DataLines = [1, Inf];
opts.Delimiter = "";
% Specify column names and types
opts.VariableNames = "Classes";
opts.VariableTypes = "string";
% Specify file level properties
opts.ExtraColumnsRule = "ignore";
opts.EmptyLineRule = "read";
% Specify variable properties
opts = setvaropts(opts, "Classes", "WhitespaceRule", "preserve");
opts = setvaropts(opts, "Classes", "EmptyFieldRule", "auto");
% Import the data
clsobj = readtable(fullfile(pastaRoboFlow,strcat('/',op,'/_classes.txt')), opts);
%%Até aqui importação automática do Matlab

end