%% Realtime_Realsense_YOLO.m — CoppeliaSim (ROS1) + MATLAB + YOLO overlay (robusto, COM VISUALIZAÇÃO DE MÁSCARA)
clear; clc; clf;

%% ========= CONFIG =========
sensorName       = "Realsense";
detectorMatPath  = "~/Downloads/OverheadBin_yolov4_detector.mat";
detectorVarName  = "detector";
maxFPS           = 10;
showDepth        = true;
scoreThresh      = 0.70;
noFrameWarnSec   = 5;

% [NOVO] nome do tópico de pose da câmera em relação à base do robô
poseTopic        = "/camera_pose_in_kuka_base";

% [NOVO] Intrínsecos aproximados da câmera simulada
resX_expected = 1280;
resY_expected = 720;
HFOV_deg      = 86;
HFOV_rad      = deg2rad(HFOV_deg);

fx_est = (resX_expected/2) / tan(HFOV_rad/2);          % focal length px eixo x
fy_est = fx_est * (resY_expected/resX_expected);        % assume pixels quadrados
cx_est = resX_expected/2;
cy_est = resY_expected/2;

camIntrinsics.fx = fx_est;
camIntrinsics.fy = fy_est;
camIntrinsics.cx = cx_est;
camIntrinsics.cy = cy_est;
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
cands = allTopics(contains(lower(allTopics), lower("/"+sensorName)));

rgbTopic = pickRgbTopic(cands);                   % prefere sem 'depth'
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

% [NOVO] subscriber para a pose da câmera no frame base do robô publicado pelo Lua
subPose = rossubscriber(poseTopic, 'BufferSize', 5);

%% 2) UI
ttl = sprintf("Realsense — %s (ROS)", sensorName);
hFig = figure('Name', ttl, 'Color','w');
tiledlayout(hFig, 1 + (~isempty(subDEP)), 2, 'Padding','compact','TileSpacing','compact');
ax1 = nexttile; title(ax1,'RGB + YOLO'); axis(ax1,'image'); axis(ax1,'off');
if ~isempty(subDEP)
    ax2 = nexttile; title(ax2,'Depth / Máscara PCA'); axis(ax2,'image'); axis(ax2,'off');
    ax3 = nexttile([1 2]); title(ax3,'Frame de Coordenadas 3D'); 
    axis(ax3,'equal'); grid(ax3,'on'); hold(ax3,'on');
    xlabel(ax3,'X (m)'); ylabel(ax3,'Y (m)'); zlabel(ax3,'Z (m)');
    view(ax3, 3);
else
    ax3 = nexttile; title(ax3,'Frame de Coordenadas 3D'); 
    axis(ax3,'equal'); grid(ax3,'on'); hold(ax3,'on');
    xlabel(ax3,'X (m)'); ylabel(ax3,'Y (m)'); zlabel(ax3,'Z (m)');
    view(ax3, 3);
    ax2 = []; % garante variável mesmo sem depth
end

% Inicializa handles para os frames / pontos / overlays
h_frame_cam   = [];
h_frame_base  = [];
h_points      = [];
h_maskOverlay = []; % [NOVO] overlay da máscara no depth
h_bboxDepth   = []; % [NOVO] bbox desenhado no depth
h_centerPoint = []; % [NOVO] ponto central do bagageiro no depth view

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
        bboxes = [];
        scores = [];
        labels = [];
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

        % Atualiza visual RGB no painel PRIMEIRO (garante display)
        imshow(IRGB, 'Parent', ax1);

        % --- DEPTH SYNC + 3D do centro do bagageiro ---
        if ~isempty(bboxes) && ~isempty(subDEP)
            msgDEP_now = subDEP.LatestMessage;
            if ~isempty(msgDEP_now)
                [depthImage, depthValid] = rebuildDepthImage(msgDEP_now);

                if depthValid
                    % pega APENAS o primeiro bbox considerado válido
                    bb = bboxes(1,:);  % [x y w h] em pixels

                    % centro do bbox (u,v)
                    u = bb(1) + bb(3)/2;
                    v = bb(2) + bb(4)/2;

                    % garante dentro da imagem
                    H = size(depthImage,1);
                    W = size(depthImage,2);
                    uu = min(max(round(u),1), W);
                    vv = min(max(round(v),1), H);

                    % profundidade naquele pixel (em metros)
                    Z = depthImage(vv,uu);

                    % fallback local se inválido
                    if ~isfinite(Z) || Z<=0
                        vRange = max(vv-1,1):min(vv+1,H);
                        uRange = max(uu-1,1):min(uu+1,W);
                        neigh = depthImage(vRange,uRange);
                        Z = median(neigh(:),'omitnan');
                    end
                    
                    % === DEBUG 1: verificar unidade do Z central ===
                    fprintf('[DEBUG] Zc (centro BB) = %.6f\n', Z);

                    if isfinite(Z) && Z>0
                        % intrínsecos
                        fx = camIntrinsics.fx;
                        fy = camIntrinsics.fy;
                        cx = camIntrinsics.cx;
                        cy = camIntrinsics.cy;

                        % ponto 3D no frame da câmera
                        Xc = ((u - cx)/fx) * Z;
                        Yc = ((v - cy)/fy) * Z;
                        Zc = Z;

                        % imprime sempre o frame da câmera (COMO JÁ FAZIA)
                        fprintf('[3D bagageiro] CamFrame XYZ = [%.3f  %.3f  %.3f] m\n', Xc, Yc, Zc);

                        % ================================
                        % Transformar para frame base do robô (MESMO CÓDIGO DE ANTES)
                        % ================================
                        poseMsg = subPose.LatestMessage;
                        if ~isempty(poseMsg)
                            % posição da câmera no frame base
                            p_cam_in_base = [ ...
                                poseMsg.Pose.Position.X; ...
                                poseMsg.Pose.Position.Y; ...
                                poseMsg.Pose.Position.Z];

                            % orientação da câmera no frame base (quaternion base<-cam)
                            q_wxyz = [ ...
                                poseMsg.Pose.Orientation.W, ...
                                poseMsg.Pose.Orientation.X, ...
                                poseMsg.Pose.Orientation.Y, ...
                                poseMsg.Pose.Orientation.Z];

                            R_base_cam = quat2rotm(q_wxyz);  % 3x3
                            T_base_cam = [R_base_cam, p_cam_in_base; 0 0 0 1];

                            % ponto em coords homogêneas da câmera
                            Pc_cam = [Xc; Yc; Zc; 1];

                            % transforma para o frame base
                            Pc_base = T_base_cam * Pc_cam;
                            Xb = Pc_base(1);
                            Yb = Pc_base(2);
                            Zb = Pc_base(3);

                            % imprime também no frame base do robô
                            fprintf('[3D bagageiro] BaseFrame XYZ = [%.3f  %.3f  %.3f] m\n', Xb, Yb, Zb);

                            % ================================
                            % PCA SIMPLIFICADO - apenas pontos essenciais
                            % ================================
                            
                            x_min = max(1, round(bb(1)));
                            y_min = max(1, round(bb(2)));
                            x_max = min(W, round(bb(1) + bb(3)));
                            y_max = min(H, round(bb(2) + bb(4)));

                            step = 4; % pula pixels para ser mais rápido
                            points_3d_cam = [];
                            
                            % [NOVO] também vamos registrar uma máscara lógica dos pixels válidos
                            mask_valid_bbox = false(H,W); % tudo falso por padrão

                            for iPix = x_min:step:x_max
                                for jPix = y_min:step:y_max
                                    Z_ij = depthImage(jPix, iPix);
                                    if isfinite(Z_ij) && Z_ij > 0 && abs(Z_ij - Z) < 0.8 % filtro simples
                                        X_ij = ((iPix - cx)/fx) * Z_ij;
                                        Y_ij = ((jPix - cy)/fy) * Z_ij;
                                        points_3d_cam = [points_3d_cam; X_ij, Y_ij, Z_ij];

                                        % marca pixel como válido na máscara
                                        mask_valid_bbox(jPix, iPix) = true;
                                    end
                                end
                            end
                            
                            % === DEBUG 2: estatística da profundidade no BB ===
                            BB = depthImage(y_min:y_max, x_min:x_max);
                            valid = isfinite(BB) & (BB>0);
                            fprintf('[DEBUG] BB valid px: %d / %d (%.1f%% válidos)\n', nnz(valid), numel(BB), 100*nnz(valid)/numel(BB));
                            if nnz(valid)>0
                                zvals = double(BB(valid));
                                fprintf('[DEBUG] Z[min, med, max] = [%.6f, %.6f, %.6f]\n', min(zvals), median(zvals), max(zvals));
                            end

                            if size(points_3d_cam, 1) > 20
                            % [% NOVO-KMEANS %]
                            pts_for_pca = points_3d_cam;  % fallback
                            try
                                K = 2;
                                Xkm = double(points_3d_cam);  % evita erro com 'single'
                            
                                if exist('kmeans','file') == 2
                                    idxKM = kmeans(Xkm, K, 'Replicates',3, 'MaxIter',100, 'OnlinePhase','off');
                                else
                                    idxKM = kmeans2_local(Xkm, K, 20);  % fallback local
                                end
                            
                                countsKM = accumarray(idxKM, 1, [K,1]);
                                [~, mainCluster] = max(countsKM);
                                pts_for_pca = points_3d_cam(idxKM == mainCluster, :);
                            
                                if size(pts_for_pca,1) < 20
                                    pts_for_pca = points_3d_cam;
                                end
                            catch ME
                                warning('KMEANS:Failed','[KMEANS] Falhou (%s). Usando todos os pontos no PCA.', ME.message);
                            end




                                % Centraliza os pontos
                                centroid = mean(pts_for_pca, 1);
                                points_centered = pts_for_pca - centroid;
                                
                                % Matriz de covariância
                                cov_matrix = cov(points_centered);
                                
                                % Autovetores e autovalores
                                [eigenvectors, eigenvalues] = eig(cov_matrix);
                                eigenvalues = diag(eigenvalues);
                                
                                % Ordena por autovalor decrescente
                                [~, sort_idx] = sort(eigenvalues, 'descend');
                                eigenvectors_sorted = eigenvectors(:, sort_idx);
                                
                                % Eixos principais no frame da câmera
                                eixo_longo_cam  = eigenvectors_sorted(:, 1)';   % maior variância
                                eixo_normal_cam = eigenvectors_sorted(:, 3)';   % menor variância (normal)
                                
                                % Transforma para frame base
                                eixo_longo_base  = R_base_cam * eixo_longo_cam';
                                eixo_normal_base = R_base_cam * eixo_normal_cam';
                                
                                % Normaliza
                                eixo_longo_base  = eixo_longo_base  / norm(eixo_longo_base);
                                eixo_normal_base = eixo_normal_base / norm(eixo_normal_base);
                                eixo_longo_cam   = eixo_longo_cam   / norm(eixo_longo_cam);
                                eixo_normal_cam  = eixo_normal_cam  / norm(eixo_normal_cam);
                                
                                % Saída PCA (prints originais mantidos)
                                fprintf('[ORIENT PCA] eixoLongo_cam = [%.3f, %.3f, %.3f]\n', ...
                                    eixo_longo_cam(1), eixo_longo_cam(2), eixo_longo_cam(3));
                                fprintf('[ORIENT PCA] normal_cam = [%.3f, %.3f, %.3f]\n', ...
                                    eixo_normal_cam(1), eixo_normal_cam(2), eixo_normal_cam(3));
                                fprintf('[ORIENT PCA] eixoLongo_base = [%.3f, %.3f, %.3f]\n', ...
                                    eixo_longo_base(1), eixo_longo_base(2), eixo_longo_base(3));
                                fprintf('[ORIENT PCA] normal_base = [%.3f, %.3f, %.3f]\n', ...
                                    eixo_normal_base(1), eixo_normal_base(2), eixo_normal_base(3));
                                fprintf('---\n');
                                
                                % ================================
                                % VISUALIZAÇÃO MELHORADA (2D e 3D)
                                % ================================
                                
                                % ---- (A) Painel DEPTH (ax2) ----
                                if ~isempty(ax2) && isvalid(ax2)
                                    % Visual base do depth, como antes:
                                    Dviz = double(depthImage);
                                    lo = prctile(Dviz(:),2); hi = prctile(Dviz(:),98);
                                    if isfinite(lo) && isfinite(hi) && hi > lo
                                        Dviz = mat2gray(Dviz,[lo,hi]);
                                    else
                                        Dviz = mat2gray(Dviz);
                                    end

                                    % mostra depth normalizado
                                    imshow(Dviz, 'Parent', ax2); hold(ax2,'on');

                                    % ponto central do bbox no depth (uu,vv)
                                    if ~isempty(h_centerPoint) && isvalidhandle(h_centerPoint)
                                        try delete(h_centerPoint); end
                                    end
                                    h_centerPoint = plot(ax2, uu, vv, 'ro', 'MarkerSize',8, 'LineWidth',2);

                                    % desenha bbox no depth (mesma posição do bb)
                                    if ~isempty(h_bboxDepth)
                                        deleteValidHandles(h_bboxDepth);
                                    end
                                    rect_x = bb(1); 
                                    rect_y = bb(2); 
                                    rect_w = bb(3); 
                                    rect_h = bb(4);
                                    h_bboxDepth = rectangle(ax2,'Position',[rect_x rect_y rect_w rect_h], ...
                                        'EdgeColor',[1 1 0], 'LineWidth',2); % amarelo

                                    % gera overlay semialfa mostrando pixels usados na PCA
                                    % criamos uma máscara RGB fake: verde onde mask_valid_bbox=true
                                    maskRGB = cat(3, zeros(H,W), ones(H,W), zeros(H,W)); % verde puro
                                    alphaMask = 0.3 * mask_valid_bbox; % alpha só nos válidos
                                    
                                    % desenha overlay
                                    h_maskOverlay = imshow(maskRGB, 'Parent', ax2);
                                    set(h_maskOverlay, 'AlphaData', alphaMask);

                                    hold(ax2,'off');
                                end
                                
                                % ---- (B) Painel 3D (ax3) ----
                                % Limpa plots anteriores
                                if ~isempty(h_frame_cam)
                                    deleteValidHandles(h_frame_cam);
                                end
                                if ~isempty(h_frame_base)
                                    deleteValidHandles(h_frame_base);
                                end
                                if ~isempty(h_points) && isvalid(h_points)
                                    delete(h_points);
                                end
                                
                                % Escala dos eixos
                                axis_length = 0.1; % 10cm
                                
                                % --- Frame da Câmera (cores fortes RGB) ---
                                origin_cam = [0, 0, 0];
                                x_axis_cam = [axis_length, 0, 0];
                                y_axis_cam = [0, axis_length, 0]; 
                                z_axis_cam = [0, 0, axis_length];
                                
                                h_frame_cam(1) = plot3(ax3, [origin_cam(1), x_axis_cam(1)], ...
                                    [origin_cam(2), x_axis_cam(2)], [origin_cam(3), x_axis_cam(3)], ...
                                    'r-', 'LineWidth', 3, 'Color', [1 0 0]);
                                h_frame_cam(2) = plot3(ax3, [origin_cam(1), y_axis_cam(1)], ...
                                    [origin_cam(2), y_axis_cam(2)], [origin_cam(3), y_axis_cam(3)], ...
                                    'g-', 'LineWidth', 3, 'Color', [0 0.8 0]);
                                h_frame_cam(3) = plot3(ax3, [origin_cam(1), z_axis_cam(1)], ...
                                    [origin_cam(2), z_axis_cam(2)], [origin_cam(3), z_axis_cam(3)], ...
                                    'b-', 'LineWidth', 3, 'Color', [0 0 1]);
                                
                                text(ax3, x_axis_cam(1), x_axis_cam(2), x_axis_cam(3), 'X_c', ...
                                    'Color', [1 0 0], 'FontSize', 12, 'FontWeight', 'bold');
                                text(ax3, y_axis_cam(1), y_axis_cam(2), y_axis_cam(3), 'Y_c', ...
                                    'Color', [0 0.8 0], 'FontSize', 12, 'FontWeight', 'bold');
                                text(ax3, z_axis_cam(1), z_axis_cam(2), z_axis_cam(3), 'Z_c', ...
                                    'Color', [0 0 1], 'FontSize', 12, 'FontWeight', 'bold');
                                
                                % --- Frame do Bagageiro (cores distintas) ---
                                origin_bagageiro = [Xb, Yb, Zb];
                                
                                x_axis_bag = origin_bagageiro + (R_base_cam * eixo_longo_cam')' * axis_length;
                                % y_axis_bag: perpendicular aproximada = cross(normal, eixo_longo)
                                ytemp = cross(eixo_normal_base, eixo_longo_base);
                                if norm(ytemp) < 1e-9
                                    ytemp = [0 0 0];
                                end
                                ytemp = ytemp / max(norm(ytemp),1e-9);
                                y_axis_bag = origin_bagageiro + ytemp * axis_length;
                                z_axis_bag = origin_bagageiro + eixo_normal_base' * axis_length;
                                
                                h_frame_base(1) = plot3(ax3, [origin_bagageiro(1), x_axis_bag(1)], ...
                                    [origin_bagageiro(2), x_axis_bag(2)], [origin_bagageiro(3), x_axis_bag(3)], ...
                                    '-', 'LineWidth', 3, 'Color', [1 0.5 0]); % laranja
                                h_frame_base(2) = plot3(ax3, [origin_bagageiro(1), y_axis_bag(1)], ...
                                    [origin_bagageiro(2), y_axis_bag(2)], [origin_bagageiro(3), y_axis_bag(3)], ...
                                    '-', 'LineWidth', 3, 'Color', [0.8 0 0.8]); % magenta
                                h_frame_base(3) = plot3(ax3, [origin_bagageiro(1), z_axis_bag(1)], ...
                                    [origin_bagageiro(2), z_axis_bag(2)], [origin_bagageiro(3), z_axis_bag(3)], ...
                                    '-', 'LineWidth', 3, 'Color', [0 0.7 0.7]); % ciano
                                
                                text(ax3, x_axis_bag(1), x_axis_bag(2), x_axis_bag(3), 'X_b', ...
                                    'Color', [1 0.5 0], 'FontSize', 12, 'FontWeight', 'bold');
                                text(ax3, y_axis_bag(1), y_axis_bag(2), y_axis_bag(3), 'Y_b', ...
                                    'Color', [0.8 0 0.8], 'FontSize', 12, 'FontWeight', 'bold');
                                text(ax3, z_axis_bag(1), z_axis_bag(2), z_axis_bag(3), 'Z_b', ...
                                    'Color', [0 0.7 0.7], 'FontSize', 12, 'FontWeight', 'bold');
                                
                                % ---- pontos 3D transformados para base (agora, do cluster escolhido) ----
                                points_3d_base = (T_base_cam * [pts_for_pca, ones(size(pts_for_pca,1),1)]')';
                                points_3d_base = points_3d_base(:,1:3);

                                h_points = scatter3(ax3, points_3d_base(:,1), points_3d_base(:,2), points_3d_base(:,3), ...
                                    8, 'filled', ...
                                    'MarkerFaceColor', [0.3 0.9 0.3], ...
                                    'MarkerEdgeColor', 'none', ...
                                    'MarkerFaceAlpha', 0.3);

                                % Marca centro do bagageiro
                                scatter3(ax3, Xb, Yb, Zb, 80, 'ro', 'filled', ...
                                    'MarkerFaceColor', [1 0 0], ...
                                    'MarkerEdgeColor', [0 0 0], 'LineWidth', 2);

                                % Atualiza limites do gráfico
                                all_points = [points_3d_base; origin_cam; origin_bagageiro];
                                margin = 0.2;
                                xlim(ax3, [min(all_points(:,1))-margin, max(all_points(:,1))+margin]);
                                ylim(ax3, [min(all_points(:,2))-margin, max(all_points(:,2))+margin]);
                                zlim(ax3, [min(all_points(:,3))-margin, max(all_points(:,3))+margin]);
                                
                                % Legenda melhorada (mantida ideia original)
                                legend(ax3, [h_frame_cam(1), h_frame_base(1)], ...
                                    {'Frame Câmera', 'Frame Bagageiro'}, 'Location', 'best', 'FontSize', 10);
                                
                                % Título dinâmico
                                title(ax3, sprintf('Frame de Coordenadas 3D - %d pontos', size(points_3d_base,1)));
                                
                                drawnow;
                            end % fim if >20 pontos
                        end % fim if ~isempty(poseMsg)
                        % ================================

                    else
                        fprintf('[3D bagageiro] profundidade inválida no centro do bbox.\n');
                    end
                end
            end
        end

    else
        if toc(tLastRgb) > noFrameWarnSec
            fprintf(1,"[RGB] Sem frames há %.1fs...\n", toc(tLastRgb));
            tLastRgb = tic;
        end
    end

    % --- Depth (visualização fallback contínua, se não fizemos overlay acima) ---
    if ~isempty(subDEP)
        msgDEP = subDEP.LatestMessage;
        if ~isempty(msgDEP)
            D = safeRosReadImage(msgDEP);
            tLastDep = tic;

            if ~isempty(ax2) && isvalid(ax2)
                % Só atualiza aqui se a gente NÃO atualizou ax2 dentro da parte PCA.
                % (Heurística simples: se h_maskOverlay ainda está vazio, quer dizer que
                %   não passamos pelo fluxo PCA válido ainda.)
                if isempty(h_maskOverlay) || ~isvalidhandle(h_maskOverlay)
                    Dviz2 = double(D);
                    lo2 = prctile(Dviz2(:),2); hi2 = prctile(Dviz2(:),98);
                    if isfinite(lo2) && isfinite(hi2) && hi2 > lo2
                        Dviz2 = mat2gray(Dviz2,[lo2,hi2]);
                    else
                        Dviz2 = mat2gray(Dviz2);
                    end
                    imshow(Dviz2, 'Parent', ax2);
                end
            end
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
try, clear subRGB subDEP subPose; end
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
    % Tenta ler sensor_msgs/Image (RGB8 ou 32FC1) de forma robusta
    try
        I = rosReadImage(msg);  % Robotics System Toolbox mais recente
        return;
    catch
    end
    try
        I = readImage(msg);     % compatibilidade com versões mais antigas
        return;
    catch
    end

    % Fallback manual (caso extremo): constrói array vazio do tamanho certo
    if isfield(msg,'Width') && isfield(msg,'Height')
        W = double(msg.Width);
        H = double(msg.Height);
        if isfield(msg,'Encoding') && contains(lower(string(msg.Encoding)),"32fc1")
            I = zeros(H,W,'single');
        else
            I = zeros(H,W,3,'uint8');
        end
    else
        I = zeros(480,640,3,'uint8');
    end
end

function [D, ok] = rebuildDepthImage(msgDEP)
    % Constrói matriz depth [H x W] em 'single' a partir de sensor_msgs/Image "32FC1"
    ok = false;
    D = [];

    % tenta caminho direto
    try
        Dtry = safeRosReadImage(msgDEP);
        % se já veio matriz 2D single, ótimo
        if ndims(Dtry)==2
            D = single(Dtry);
            ok = true;
            return;
        end
    catch
        % continua pra leitura manual
    end

    % leitura manual da mensagem raw
    if isfield(msgDEP,'Data') && isfield(msgDEP,'Width') && isfield(msgDEP,'Height')
        W = double(msgDEP.Width);
        H = double(msgDEP.Height);

        % msgDEP.Data é uint8 linearizado com floats 32-bit little-endian
        rawSingle = typecast(msgDEP.Data, 'single');  % vetor float32
        if numel(rawSingle) == W*H
            D = reshape(rawSingle, [W H]).';
            D = single(D);
            ok = true;
            return;
        end
    end
end

% [NOVO] helpers simples para lidar com handles que podem ter virado inválidos
function tf = isvalidhandle(h)
    % retorna true se h existe e não foi deletado
    tf = ~(isempty(h) || ~all(ishandle(h)));
end

function deleteValidHandles(hvec)
    for kk = 1:numel(hvec)
        if ishandle(hvec(kk))
            try
                delete(hvec(kk));
            end
        end
    end
end

function idx = kmeans2_local(X, K, maxIter)
% kmeans2_local - k-means bem simples (Lloyd), só p/ K pequeno (ex.: 2)
% X: [N x d] double, K: clusters, maxIter: iterações (ex.: 20)
    if nargin < 3, maxIter = 20; end
    [N, ~] = size(X);
    rng('default');  % determinístico
    C = X(randperm(N, K), :);  % centros iniciais
    idx = ones(N,1);

    for it = 1:maxIter
        % atribuição
        D = zeros(N, K);
        for k = 1:K
            diff = X - C(k,:);
            D(:,k) = sum(diff.^2, 2);
        end
        [~, newIdx] = min(D, [], 2);

        % atualização
        newC = C;
        for k = 1:K
            sel = (newIdx == k);
            if any(sel)
                newC(k,:) = mean(X(sel,:), 1);
            else
                newC(k,:) = X(randi(N),:); % reinit se cluster vazio
            end
        end

        % critério de parada
        if all(newIdx == idx) || max(vecnorm(newC - C, 2, 2)) < 1e-6
            idx = newIdx; C = newC; 
            break;
        end
        idx = newIdx; C = newC;
    end
end
