%% Realtime_Realsense_YOLO.m — CoppeliaSim (ROS1) + MATLAB + YOLO overlay (robusto)
clear; clc;

%% ========= CONFIG =========
sensorName       = "Realsense";
detectorMatPath  = "OverheadBin_yolov4_detector.mat";
detectorVarName  = "detector";
maxFPS           = 30;
showDepth        = true;
scoreThresh      = 0.20;
noFrameWarnSec   = 5;
%% =========================

%% 0) Detector
assert(isfile(detectorMatPath), "Arquivo do detector não encontrado: %s", detectorMatPath);
S = load(detectorMatPath);
assert(isfield(S, detectorVarName), "Variável '%s' não encontrada em %s.", detectorVarName, detectorMatPath);
detector = S.(detectorVarName);
fprintf("Detector carregado de %s (classe: %s)\n", detectorMatPath, class(detector));

%% 1) ROS
try, rosshutdown; end
rosinit;

tmp = rostopic('list'); allTopics = string(tmp);
cands = allTopics( contains(lower(allTopics), lower("/"+sensorName)) );

rgbTopic = pickRgbTopic(cands);                    % prefere sem 'depth'
if showDepth
    depthTopic = pickDepthTopic(cands);           % só escolhe se solicitado
else
    depthTopic = "";
end
if depthTopic == ""  % se não achou, desativa
    showDepth = false;
end

assert(strlength(rgbTopic) > 0, "Não encontrei tópico RGB para '%s'.", sensorName);
fprintf("Tópico RGB  selecionado: %s\n", char(rgbTopic));
if showDepth
    fprintf("Tópico Depth selecionado: %s\n", char(depthTopic));
else
    fprintf("Tópico Depth: não utilizado.\n");
end

subRGB = rossubscriber(char(rgbTopic), 'BufferSize', 5);
if showDepth
    subDEP = rossubscriber(char(depthTopic), 'BufferSize', 5);
else
    subDEP = [];
end

%% 2) UI
ttl = sprintf("Realsense — %s (ROS)", sensorName);
hFig = figure('Name', ttl, 'Color','w');
tiledlayout(hFig, 1 + (~isempty(subDEP)), 1, 'Padding','compact','TileSpacing','compact');
ax1 = nexttile; title(ax1,'RGB + YOLO'); axis(ax1,'image'); axis(ax1,'off');
if ~isempty(subDEP)
    ax2 = nexttile; title(ax2,'Depth'); axis(ax2,'image'); axis(ax2,'off');
end

rc = rateControl(maxFPS);
frameCount = 0; tStart = tic;
tLastRgb = tic; tLastDep = tic;

fprintf("Streaming com YOLO ativo... (feche a janela para encerrar)\n");

%% 3) Loop principal
while ishandle(hFig)
    % --- RGB ---
    msgRGB = subRGB.LatestMessage;
    if ~isempty(msgRGB)
        IRGB = safeRosReadImage(msgRGB);
        tLastRgb = tic;

        % YOLO
        try
            [bboxes, scores, labels] = detect(detector, IRGB);
            if ~isempty(scores)
                keep = scores >= scoreThresh;
                bboxes = bboxes(keep,:); scores = scores(keep); labels = labels(keep);
            end
            if ~isempty(bboxes)
                ann = arrayfun(@(l,s) sprintf('%s (%.2f)', string(l), s), labels, scores, 'UniformOutput', false);
                IRGB = insertObjectAnnotation(IRGB,'rectangle',bboxes,ann);
            end
        catch ME
            warning("detect() falhou: %s", ME.message);
        end

        imshow(IRGB, 'Parent', ax1);
    else
        if toc(tLastRgb) > noFrameWarnSec
            fprintf(1,"[RGB] Sem frames há %.1fs...\n", toc(tLastRgb));
            tLastRgb = tic;
        end
    end

    % --- Depth (opcional) ---
    if ~isempty(subDEP)
        msgDEP = subDEP.LatestMessage;
        if ~isempty(msgDEP)
            D = safeRosReadImage(msgDEP);
            tLastDep = tic;

            Dviz = double(D);
            lo = prctile(Dviz(:),2); hi = prctile(Dviz(:),98);
            if isfinite(lo) && isfinite(hi) && hi > lo
                Dviz = mat2gray(Dviz,[lo,hi]);
            else
                Dviz = mat2gray(Dviz);
            end
            imshow(Dviz, 'Parent', ax2);
        else
            if toc(tLastDep) > noFrameWarnSec
                fprintf(1,"[DEPTH] Sem frames há %.1fs...\n", toc(tLastDep));
                tLastDep = tic;
            end
        end
    end

    % HUD: FPS
    frameCount = frameCount + 1;
    if mod(frameCount, 10) == 0
        fps = frameCount / toc(tStart);
        if ishandle(hFig)
            hFig.Name = sprintf('%s — FPS: %.1f', ttl, fps);
        end
    end

    drawnow limitrate nocallbacks;
    waitfor(rc);
end

%% 4) Encerramento
try, clear subRGB subDEP; end
try, rosshutdown; end
disp('Encerrado.');

%% ===== Funções locais =====
function t = pickRgbTopic(cands)
    cands = string(cands);
    maskBasic = contains(lower(cands),["image_raw","image","rgb","color"]);
    maskNoDepth = ~contains(lower(cands),["depth","aligned_depth","z16"]);
    pool = cands(maskBasic & maskNoDepth);
    if isempty(pool), pool = cands(maskBasic); end
    if isempty(pool), t = ""; return; end
    prefs = ["image_raw","rgb","color","image"];
    t = chooseByPrefs(pool, prefs);
end

function t = pickDepthTopic(cands)
    cands = string(cands);
    maskDepth = contains(lower(cands),["depth","aligned_depth","z16"]);
    pool = cands(maskDepth);
    if isempty(pool), t = ""; return; end
    prefs = ["aligned_depth","depth","z16","image_raw","image"];
    t = chooseByPrefs(pool, prefs);
end

function t = chooseByPrefs(cands, prefs)
    cands = string(cands);
    hit = "";
    for p = prefs
        idx = find(contains(lower(cands), lower(p)), 1, 'first');
        if ~isempty(idx), hit = cands(idx); break; end
    end
    if hit == "", t = cands(1); else, t = hit; end
end

function I = safeRosReadImage(msg)
    try
        I = rosReadImage(msg);
    catch
        try
            I = readImage(msg);
        catch
            if isfield(msg,'Width') && isfield(msg,'Height')
                I = zeros(double(msg.Height), double(msg.Width), 3, 'uint8');
            else
                I = zeros(480,640,3,'uint8');
            end
        end
    end
end
