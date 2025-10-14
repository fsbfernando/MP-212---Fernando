%%Script que transforma database "Pytorch" da RoboFlow para formato da biblioteca do Matlab
%Versao V4: Multiclasses, corrigindo os bounding boxes, pega dados de treinamento, validação e testes
%Notas: tem que adicionar os nomes das pastas que contém com as imagens
%anotadas de cada classe. Nessa versao consegue trabalhar com imagens de
%rotulação multipla.


% Descrição: Use validateInputData to detect invalid images, bounding boxes or labels i.e.,
% Samples with invalid image format or containing NaNs
% Bounding boxes containing zeros/NaNs/Infs/empty
% Missing/non-categorical labels.
%
% The values of the bounding boxes must be finite positive integers and must not be NaN. 
% The height and the width of the bounding box values must be positive and lie within the image boundary.
% x and y specify the upper-left corner of the rectangle.
% w specifies the width of the rectangle, which is its length along the x-axis.
% h specifies the height of the rectangle, which is its length along the y-axis.
clear, clc,close all

%% Primeira Pasta (se houver mais, vá colando o código abaixo e alterando os nomes)
%Primeiro com a pasta MultiClasse - Imagens com mais de uma rotulaçao

%!!!!!!!!!!!! ALTERAR AQUI DE ACORDO COM A PASTA !!!!!!!!!!!!!!!!!!!!
nomedaPasta = 'OVHD.v2i.yolov4pytorch';%Sempre atualizar o nome de acordo com o nome da pasta da RoboFlow

[clsobj,annotations]=readDadosFilePytorch(nomedaPasta,'train');
tabelaMulticlass_train = formaTable(clsobj,annotations,nomedaPasta,'train'); %!!! ALTERAR O NOME DA VARIAVEL DE ACORDO COM A PASTA !!!
%>> Repete o processo para a pasta de validação
[clsobj,annotations]=readDadosFilePytorch(nomedaPasta,'valid');
tabelaMulticlass_valid = formaTable(clsobj,annotations,nomedaPasta,'valid'); %!!! ALTERAR O NOME DA VARIAVELDE ACORDO COM A PASTA !!!
%>> Repete o processo para a pasta de teste
[clsobj,annotations]=readDadosFilePytorch(nomedaPasta,'test');
tabelaMulticlass_test = formaTable(clsobj,annotations,nomedaPasta,'test');   %!!! ALTERAR O NOME DA VARIAVELDE ACORDO COM A PASTA !!!

analiseGroundThruthTable(tabelaMulticlass_train,tabelaMulticlass_valid,tabelaMulticlass_test) %!!! ALTERAR O NOME DA VARIAVELDE ACORDO COM A PASTA !!!
disp('Pressione qualquer  tecla para prosseguir...')
pause;

clear clsobj annotations nomedaPasta

% %% Segunda Pasta (se houver mais, vá colando o código abaixo e alterando os nomes)
% %!!!!!!!!!!!! ALTERAR AQUI DE ACORDO COM A PASTA !!!!!!!!!!!!!!!!!!!!
% nomedaPasta = 'Cable Shark V2.v1i.yolov4pytorch_Housing';%Sempre atualizar o nome de acordo com o nome da pasta da RoboFlow
% 
% [clsobj,annotations]=readDadosFilePytorch(nomedaPasta,'train');
% tabelaHousing_train = formaTable(clsobj,annotations,nomedaPasta,'train'); %!!! ALTERAR O NOME DA VARIAVEL DE ACORDO COM A PASTA !!!
% %>> Repete o processo para a pasta de validação
% [clsobj,annotations]=readDadosFilePytorch(nomedaPasta,'valid');
% tabelaHousing_valid = formaTable(clsobj,annotations,nomedaPasta,'valid'); %!!! ALTERAR O NOME DA VARIAVELDE ACORDO COM A PASTA !!!
% %>> Repete o processo para a pasta de teste
% [clsobj,annotations]=readDadosFilePytorch(nomedaPasta,'test');
% tabelaHousing_test = formaTable(clsobj,annotations,nomedaPasta,'test');   %!!! ALTERAR O NOME DA VARIAVELDE ACORDO COM A PASTA !!!
% 
% analiseGroundThruthTable(tabelaHousing_train,tabelaHousing_valid,tabelaHousing_test) %!!! ALTERAR O NOME DA VARIAVELDE ACORDO COM A PASTA !!!
% disp('Pressione qualquer  tecla para prosseguir...')
% pause;
% 
% %% Terceira Pasta(se houver mais, vá colando o código abaixo e alterando os nomes)
% %!!!!!!!!!!!! ALTERAR AQUI DE ACORDO COM A PASTA !!!!!!!!!!!!!!!!!!!!
% nomedaPasta = 'Cable Shark V2.v1i.yolov4pytorch_Cap';%Sempre atualizar o nome de acordo com o nome da pasta da RoboFlow
% 
% [clsobj,annotations]=readDadosFilePytorch(nomedaPasta,'train');
% tabelaCap_train = formaTable(clsobj,annotations,nomedaPasta,'train'); %!!! ALTERAR O NOME DA VARIAVEL DE ACORDO COM A PASTA !!!
% %>> Repete o processo para a pasta de validação
% [clsobj,annotations]=readDadosFilePytorch(nomedaPasta,'valid');
% tabelaCap_valid = formaTable(clsobj,annotations,nomedaPasta,'valid'); %!!! ALTERAR O NOME DA VARIAVELDE ACORDO COM A PASTA !!!
% %>> Repete o processo para a pasta de teste
% [clsobj,annotations]=readDadosFilePytorch(nomedaPasta,'test');
% tabelaCap_test = formaTable(clsobj,annotations,nomedaPasta,'test');   %!!! ALTERAR O NOME DA VARIAVELDE ACORDO COM A PASTA !!!
% 
% analiseGroundThruthTable(tabelaCap_train,tabelaCap_valid,tabelaCap_test) %!!! ALTERAR O NOME DA VARIAVELDE ACORDO COM A PASTA !!!
% disp('Pressione qualquer  tecla para prosseguir...')
% pause;
% 
% clear clsobj annotations nomedaPasta
% 
% %% Quarta Pasta (se houver mais, vá colando o código abaixo e alterando os nomes)
% %!!!!!!!!!!!! ALTERAR AQUI DE ACORDO COM A PASTA !!!!!!!!!!!!!!!!!!!!
% nomedaPasta = 'Cable Shark V2.v1i.yolov4pytorch_Spring';%Sempre atualizar o nome de acordo com o nome da pasta da RoboFlow
% 
% [clsobj,annotations]=readDadosFilePytorch(nomedaPasta,'train');
% tabelaSpring_train = formaTable(clsobj,annotations,nomedaPasta,'train'); %!!! ALTERAR O NOME DA VARIAVEL DE ACORDO COM A PASTA !!!
% %>> Repete o processo para a pasta de validação
% [clsobj,annotations]=readDadosFilePytorch(nomedaPasta,'valid');
% tabelaSpring_valid = formaTable(clsobj,annotations,nomedaPasta,'valid'); %!!! ALTERAR O NOME DA VARIAVELDE ACORDO COM A PASTA !!!
% %>> Repete o processo para a pasta de teste
% [clsobj,annotations]=readDadosFilePytorch(nomedaPasta,'test');
% tabelaSpring_test = formaTable(clsobj,annotations,nomedaPasta,'test');   %!!! ALTERAR O NOME DA VARIAVELDE ACORDO COM A PASTA !!!
% 
% analiseGroundThruthTable(tabelaSpring_train,tabelaSpring_valid,tabelaSpring_test) %!!! ALTERAR O NOME DA VARIAVELDE ACORDO COM A PASTA !!!
% disp('Pressione qualquer  tecla para prosseguir...')
% pause;
% 
% clear clsobj annotations nomedaPasta
% 
% %% Quinta Pasta (se houver mais, vá colando o código abaixo e alterando os nomes)
% %!!!!!!!!!!!! ALTERAR AQUI DE ACORDO COM A PASTA !!!!!!!!!!!!!!!!!!!!
% nomedaPasta = 'Cable Shark V2.v1i.yolov4pytorch_Wedge';%Sempre atualizar o nome de acordo com o nome da pasta da RoboFlow
% 
% [clsobj,annotations]=readDadosFilePytorch(nomedaPasta,'train');
% tabelaWedge_train = formaTable(clsobj,annotations,nomedaPasta,'train'); %!!! ALTERAR O NOME DA VARIAVEL DE ACORDO COM A PASTA !!!
% %>> Repete o processo para a pasta de validação
% [clsobj,annotations]=readDadosFilePytorch(nomedaPasta,'valid');
% tabelaWedge_valid = formaTable(clsobj,annotations,nomedaPasta,'valid'); %!!! ALTERAR O NOME DA VARIAVELDE ACORDO COM A PASTA !!!
% %>> Repete o processo para a pasta de teste
% [clsobj,annotations]=readDadosFilePytorch(nomedaPasta,'test');
% tabelaWedge_test = formaTable(clsobj,annotations,nomedaPasta,'test');   %!!! ALTERAR O NOME DA VARIAVELDE ACORDO COM A PASTA !!!
% 
% analiseGroundThruthTable(tabelaWedge_train,tabelaWedge_valid,tabelaWedge_test) %!!! ALTERAR O NOME DA VARIAVELDE ACORDO COM A PASTA !!!
% disp('Pressione qualquer  tecla para prosseguir...')
% pause;
% 
% clear clsobj annotations nomedaPasta

%% DATASTORE: Mescla das bases de dados para treinar multiclasse - Tem que chamar todas tabelas!!

%!!!!!!!!!!!! ALTERAR AQUI PARA CHAMAR TODAS AS TABELAS DE CLASSES !!!!!!!!!!!!!!!!!!!!
%Treinamento: MANTENHA A MESMA SEQUÊNCIA !!
imdstrain = imageDatastore([tabelaMulticlass_train{:,"imageFilename"}]);
bldstrain = boxLabelDatastore([table(tabelaMulticlass_train.Boxes,tabelaMulticlass_train.Labels,'VariableNames',["Boxes","Labels"])]); 
%Validação: MANTENHA A MESMA SEQUÊNCIA !!
imdsvalid = imageDatastore([tabelaMulticlass_valid{:,"imageFilename"};]);
bldsvalid = boxLabelDatastore([table(tabelaMulticlass_valid.Boxes,tabelaMulticlass_valid.Labels,'VariableNames',["Boxes","Labels"]) ...
                              ]); 

%Teste: MANTENHA A MESMA SEQUÊNCIA !!
imdstest = imageDatastore([tabelaMulticlass_test{:,"imageFilename"};]);
bldstest = boxLabelDatastore([table(tabelaMulticlass_test.Boxes,tabelaMulticlass_test.Labels,'VariableNames',["Boxes","Labels"]) ...
                              ]); 

%%Combina as imagens e os bouding box
trainingData = combine(imdstrain,bldstrain);
validationData = combine(imdsvalid,bldsvalid);
testData = combine(imdstest,bldstest);
%%Valida dados 
validateInputData(trainingData);
validateInputData(validationData);
validateInputData(testData);

%% GROUND TRUTH DATA ANALISYS: Mostra o estado dos bounding boxes de todas as imagens (ground truth data)
disp(' ')
disp('Vamos visualizar como ficou a rotulação da roboFlow para cada conjunto ...')
disp('Pressione qualquer  tecla para prosseguir...')
pause;
disp('Começando pelos dados de test ...')
i=1;
figure;
while(hasdata(testData))
    data = read(testData);
    imshow(insertObjectAnnotation(data{1},"rectangle",data{2},data{3},LineWidth=3,FontSize=12));
    title(strcat('Datastore: Imagem Rotulada #',num2str(i)));
    i=i+1;
    pause(0.050);
end
clear data i;
reset(testData);

disp(' ')
disp('Pressione qualquer  tecla para prosseguir...')
pause;
disp('Dados de validação ...')
i=1;
figure;
while(hasdata(validationData))
    data = read(validationData);
    imshow(insertObjectAnnotation(data{1},"rectangle",data{2},data{3},LineWidth=3,FontSize=12));
    title(strcat('Datastore: Imagem Rotulada #',num2str(i)));
    i=i+1;
    pause(0.050);
end
clear data i;
reset(validationData);

disp(' ')
disp('Pressione qualquer  tecla para prosseguir...')
pause;
disp('Dados de treinamento ...')
op=input('Digite Y para plotar ou qualquer tecla caso contrário: ');
if(op=='Y')
    i=1;
    figure;
    while(hasdata(trainingData))
        data = read(trainingData);
        imshow(insertObjectAnnotation(data{1},"rectangle",data{2},data{3},LineWidth=3,FontSize=12));
        title(strcat('Datastore: Imagem Rotulada #',num2str(i)));
        i=i+1;
        pause(0.120);
    end
    clear data i;
    reset(trainingData);
end
clear data i op ans;

%Analise Agregada dos dados pré-treinamento
disp(' ')
disp('Vamos iniciar uma análise agregada do Ground Truth Data de treinamento ...')
disp('Distribuição dos rótulos de classe ...')
tbl = countEachLabel(bldstrain);
figure
bar(tbl.Label,tbl.Count)
ylabel("Frequency")
%Analyze Object Sizes and Choose Object Detector
data = readall(bldstrain);
bboxes = vertcat(data{:,1});
labels = vertcat(data{:,2});
diagonalLength = hypot(bboxes(:,3),bboxes(:,4));
G = findgroups(labels);
groupedDiagonalLength = splitapply(@(x){x},diagonalLength,G);
figure
clsobj = tbl.Label;
numClasses = numel(clsobj);
for i = 1:numClasses
    len = groupedDiagonalLength{i};
    x = repelem(i,numel(len),1);
    plot(x,len,"o");
    hold on
end
hold off
ylabel("Object extent (pixels)")
xticks(1:numClasses)
xticklabels(clsobj)

%Reinicializa os datastores para seguir no treinamento
reset(testData);
reset(validationData);
reset(trainingData);

clear imdstrain bldstrain imdsvalid bldsvalid imdstest bldstest G tbl data bboxes labels diagonalLength groupedDiagonalLength clsobj numClasses x i len;




%% ------------ LEAI-ME PARA ENTENDER AS ANÁLISES DESSE CÓDIGO ------------

% This visualization highlights the important data set attributes that help
% you determine which type of object detector to configure:
% The object size variance within each class
% The object size variance across classes
% 
% In this data set, the there is a good amount of overlap between the size 
% ranges across classes. In addition, the size variation within each class 
% is not very large. This means that one multiclass detector can be trained 
% to handle a range of object sizes. If the size ranges do not overlap or if 
% the range of object sizes is more than 10 times apart, then training 
% multiple detectors for different size ranges is more practical.
% 
% You can determine which object detector to train based on the size 
% variance. When size variance within each class is small, use a 
% single-scale object detector such as YOLO v2. If there is large variance
% within each class, choose a multi-scale object detector such as YOLO v4
% or SSD. Since the object sizes in this data set are within the same order
% of magnitude, use YOLO v2 to start. Although advanced multi-scale 
% detectors may perform better, training may consume more time and 
% resources compared to YOLO v2. Use more advanced detectors when simpler 
% solutions do not meet your performance requirements.
% 
% Use the size distribution information to select the training image size,
% which is typically fixed to enable batch processing during training. The 
% training image size dictates how large the batch size can be, based on
% the resource constraints of your training environment such as GPU memory.
% Process larger batches of data to improve throughput and reduce training 
% time, especially when using a GPU. However, the training image size may 
% impact the resolution of objects if the original data is drastically 
% resized to a smaller size.
% 
