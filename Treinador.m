%% Versão atualizada do Estudo YOLO     Data: Outubro de 2025
% Descrição: Treina YOLOv4 (3 heads) usando diretamente os CombinedDatastores:
%            trainingData, validationData e testData já existentes no workspace.
% Requisitos:
%  - Computer Vision Toolbox + "Computer Vision Toolbox Model for YOLO v4 Object Detection"
%  - Variáveis: trainingData, validationData, testData (CombinedDatastore: {imageDatastore, boxLabelDatastore})

clc; close all

%% (Opcional) Verificação do suporte YOLOv4 instalado
% visionSupportPackages  % descomente para abrir o Add-On Explorer

%% Configurações básicas
inputSize = [416 416 3];
classesNames = "OverheadBin";   % ✅ nome da sua única classe

%% Padroniza labels dos 3 datastores (train/val/test)
standardize = @(D) transform(D, @(data) forceOneClass(data, classesNames));
trainingData   = standardize(trainingData);
validationData = standardize(validationData);
testData       = standardize(testData);

%% Verificação rápida (sanity check)
d = read(trainingData); reset(trainingData);
disp("Categorias detectadas:"); disp(categories(d{3}));

%% Pré-processamento para estimar anchors
rng("default")
trainingDataForEstimation = transform(trainingData, @(data) preprocessData(data, inputSize));

%% Estimar anchors (12 -> 4 por head)
numAnchors = 12;
[anchors, meanIoU] = estimateAnchorBoxes(trainingDataForEstimation, numAnchors);
disp("meanIoU (anchors):"); disp(meanIoU);

% Ordenar por área e distribuir 4/4/4 nos 3 heads
area = anchors(:,1) .* anchors(:,2);
[~, idx] = sort(area, "descend");
anchors = anchors(idx, :);
anchorBoxes = {anchors(1:4,:), anchors(5:8,:), anchors(9:12,:)};

%% Criar detector YOLOv4 (transfer learning com CSP-Darknet53)
detector = yolov4ObjectDetector("csp-darknet53-coco", classesNames, anchorBoxes, ...
                                InputSize = inputSize);

%% Data augmentation somente no treinamento
augmentedTrainingData = transform(trainingData, @augmentData);

%% Visualização rápida de 4 amostras aumentadas
augmentedData = cell(4,1);
for k = 1:4
    data = read(augmentedTrainingData);
    Ivis = insertShape(data{1}, "rectangle", data{2});
    augmentedData{k} = Ivis;
end
reset(augmentedTrainingData);
figure, montage(augmentedData, BorderSize=10), title('Amostras aumentadas')

%% Opções de treinamento
checkpath_dir = fullfile(pwd, 'Redes_Parciais'); 
if ~exist(checkpath_dir, 'dir'), mkdir(checkpath_dir); end
addpath(genpath(checkpath_dir));

options = trainingOptions("adam", ...
    GradientDecayFactor=0.9, ...
    SquaredGradientDecayFactor=0.999, ...
    InitialLearnRate=1e-4, ...
    LearnRateSchedule="none", ...
    MiniBatchSize=5, ...
    L2Regularization=5e-4, ...
    ResetInputNormalization=0, ...
    MaxEpochs=100, ...
    BatchNormalizationStatistics="moving", ...
    DispatchInBackground=falsepara, ...
    Shuffle="every-epoch", ...
    Verbose=1, ...
    VerboseFrequency=5, ...
    ValidationFrequency=20, ...
    CheckpointPath=checkpath_dir, ...
    CheckpointFrequency=200, ...
    CheckpointFrequencyUnit="iteration", ...
    ValidationData=validationData, ...
    OutputNetwork="best-validation-loss", ...
    Plots="training-progress");

%% 🧠 Treinamento
[detector, info] = trainYOLOv4ObjectDetector(augmentedTrainingData, detector, options);
save("RedeYOLOv4_Treinada.mat", 'detector', 'info')

%% 🧪 Teste rápido no conjunto de teste
reset(testData);
numToShow = 4;
for i = 1:numToShow
    if ~hasdata(testData), break; end
    data = read(testData);
    I = data{1};
    [bboxes, scores, labels] = detect(detector, I);
    if ~isempty(bboxes)
        I = insertObjectAnnotation(I, "rectangle", bboxes, ...
            strcat(string(labels), "=", compose("%.2f", scores)), ...
            LineWidth=3, FontSize=10);
    end
    figure, imshow(I), title(sprintf('Detecções em teste #%d', i))
end
reset(testData);

%% 📊 Avaliação completa
detectionResults = detect(detector, testData, Threshold=0.01);
metrics = evaluateObjectDetection(detectionResults, testData);
disp(metrics)

% Curva P-R da classe única
if ~isempty(metrics.ClassMetrics)
    classID = 1;
    precision = metrics.ClassMetrics.Precision{classID};
    recall    = metrics.ClassMetrics.Recall{classID};
    figure
    plot(recall, precision, 'LineWidth', 2)
    xlabel("Recall"), ylabel("Precision"), grid on
    ttl = sprintf("Average Precision (%s) = %.2f", ...
                  string(classesNames), metrics.ClassMetrics.mAP(classID));
    title(ttl)
end

%% ======= Funções de suporte =============================================

function data = forceOneClass(data, className)
% Converte labels para categorical com 1 única classe
for ii = 1:size(data,1)
    I   = data{ii,1};
    box = data{ii,2};
    lbl = data{ii,3};

    if isstring(lbl); lbl = cellstr(lbl); end
    if ischar(lbl);   lbl = {lbl};       end
    if iscell(lbl) && ~isempty(lbl) && ~iscellstr(lbl)
        lbl = cellstr(string(lbl));
    end
    if isempty(lbl)
        lbl = categorical([], {char(className)});
    else
        lbl = categorical(lbl, {char(className)});
    end

    data(ii,:) = {I, box, lbl};
end
end

function data = augmentData(A)
% Aumenta dados: jitter, flip e leve variação geométrica
data = cell(size(A));
for ii = 1:size(A,1)
    I       = A{ii,1};
    bboxes  = A{ii,2};
    labels  = A{ii,3};

    sz = size(I);
    if numel(sz) == 3 && sz(3) == 3
        I = jitterColorHSV(I, ...
            contrast=0.0, Hue=0.1, Saturation=0.2, Brightness=0.2);
    end

    tform = randomAffine2d(XReflection=true, Scale=[1 1.05]); % ajuste mais seguro
    rout  = affineOutputView(sz, tform, BoundsStyle="centerOutput");
    I     = imwarp(I, tform, OutputView=rout);

    [bboxes, indices] = bboxwarp(bboxes, tform, rout, OverlapThreshold=0.10);
    labels = labels(indices);

    if isempty(indices)
        data(ii,:) = A(ii,:);
    else
        data(ii,:) = {I, bboxes, labels};
    end
end
end

function data = preprocessData(data, targetSize)
% Redimensiona imagem e caixas para targetSize; normaliza pixels [0,1]
for ii = 1:size(data,1)
    I      = data{ii,1};
    bboxes = data{ii,2};
    imgSz  = size(I);

    I = im2single(imresize(I, targetSize(1:2)));
    scale = targetSize(1:2) ./ imgSz(1:2);
    bboxes = bboxresize(bboxes, scale);

    data(ii,1:2) = {I, bboxes};
end
end
