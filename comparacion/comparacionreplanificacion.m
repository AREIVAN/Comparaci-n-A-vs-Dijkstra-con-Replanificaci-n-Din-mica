clc;
clear;
close all;

%% ==========================================================
%  COMPARACIÓN A* VS DIJKSTRA CON REPLANIFICACIÓN DINÁMICA
%  ==========================================================

rows = 20;
cols = 30;

mapBase = zeros(rows, cols);

%% Obstáculos tipo almacén
mapBase(4:16, 8) = 1;
mapBase(4:16, 14) = 1;
mapBase(4:16, 20) = 1;

mapBase(4, 8:20) = 1;
mapBase(16, 8:20) = 1;

% Aberturas / pasillos
mapBase(10, 8) = 0;
mapBase(6, 14) = 0;
mapBase(13, 20) = 0;
mapBase(16, 17) = 0;

%% Inicio y meta
startNode = [18, 3];
goalNode  = [3, 27];

%% Crear figura
fig = figure('Name', 'A* vs Dijkstra con replanificación dinámica');
set(fig, 'Position', [80 100 1500 650]);

%% Crear video MP4
video = VideoWriter('astar_vs_dijkstra_replanificacion.mp4', 'MPEG-4');
video.FrameRate = 30;
video.Quality = 95;
open(video);

%% Ejecutar simulación A*
subplot(1,2,1);
resultA = runDynamicReplanningDemo( ...
    mapBase, startNode, goalNode, "astar", ...
    video, fig, [0.2 0.6 1], 'r', 'b');

%% Ejecutar simulación Dijkstra
subplot(1,2,2);
resultD = runDynamicReplanningDemo( ...
    mapBase, startNode, goalNode, "dijkstra", ...
    video, fig, [1 0.55 0], 'r', 'b');

%% Mostrar comparación final en consola
fprintf('\n=============================================================\n');
fprintf('COMPARACIÓN A* VS DIJKSTRA CON REPLANIFICACIÓN DINÁMICA\n');
fprintf('=============================================================\n');

fprintf('\n%-15s %-15s %-15s %-15s %-15s %-15s %-15s\n', ...
    'Algoritmo', 'Costo inicial', 'Costo repl.', 'Costo total', ...
    'Explorados', 'Tiempo total', 'Replan');

fprintf('%-15s %-15.2f %-15.2f %-15.2f %-15d %-15.6f %-15s\n', ...
    'A*', ...
    resultA.initialCost, resultA.replanCost, resultA.totalCost, ...
    resultA.totalExplored, resultA.totalTime, string(resultA.replanned));

fprintf('%-15s %-15.2f %-15.2f %-15.2f %-15d %-15.6f %-15s\n', ...
    'Dijkstra', ...
    resultD.initialCost, resultD.replanCost, resultD.totalCost, ...
    resultD.totalExplored, resultD.totalTime, string(resultD.replanned));

fprintf('\nInterpretación:\n');
fprintf('- Ambos algoritmos calculan una ruta inicial.\n');
fprintf('- Después aparece el mismo obstáculo dinámico sobre la ruta.\n');
fprintf('- Ambos replanifican desde la posición actual del robot.\n');
fprintf('- A* normalmente explora menos nodos por usar heurística.\n');
fprintf('- Dijkstra suele explorar más porque solo usa costo acumulado.\n');

%% Congelar imagen final en video
for k = 1:60
    frame = getframe(fig);
    writeVideo(video, frame);
end

%% Cerrar video
close(video);
fprintf('\nVideo guardado como: astar_vs_dijkstra_replanificacion.mp4\n');


%% =================================================================
%  FUNCIÓN PRINCIPAL DE DEMO CON REPLANIFICACIÓN
%  =================================================================
function result = runDynamicReplanningDemo( ...
    mapBase, startNode, goalNode, algorithm, video, fig, exploredColor, initialColor, replanColor)

    map = mapBase;

    %% Dibujar mapa
    imagesc(map);
    colormap(flipud(gray));
    axis equal tight;
    grid on;
    hold on;

    if algorithm == "astar"
        title('A* con replanificación dinámica');
        algName = 'A*';
    else
        title('Dijkstra con replanificación dinámica');
        algName = 'Dijkstra';
    end

    xlabel('Columnas');
    ylabel('Filas');

    plot(startNode(2), startNode(1), 'go', ...
        'MarkerSize', 12, 'MarkerFaceColor', 'g');

    plot(goalNode(2), goalNode(1), 'mo', ...
        'MarkerSize', 12, 'MarkerFaceColor', 'm');

    text(startNode(2)+0.3, startNode(1), 'Inicio', ...
        'Color', 'g', 'FontWeight', 'bold');

    text(goalNode(2)+0.3, goalNode(1), 'Meta', ...
        'Color', 'm', 'FontWeight', 'bold');

    for k = 1:20
        frame = getframe(fig);
        writeVideo(video, frame);
    end

    %% Planeación inicial
    tic;
    [pathInitial, exploredInitial, initialCost, successInitial] = pathPlannerAnimated( ...
        map, startNode, goalNode, algorithm, video, fig, exploredColor);
    initialTime = toc;

    if ~successInitial
        error('%s no encontró ruta inicial.', algName);
    end

    %% Dibujar ruta inicial
    initialPathPlot = plot(pathInitial(:,2), pathInitial(:,1), ...
        '-', 'Color', initialColor, 'LineWidth', 3);

    initialPointsPlot = plot(pathInitial(:,2), pathInitial(:,1), ...
        'o', 'Color', initialColor, ...
        'MarkerSize', 5, 'MarkerFaceColor', initialColor);

    title(sprintf('%s - Ruta inicial encontrada', algName));

    for k = 1:30
        frame = getframe(fig);
        writeVideo(video, frame);
    end

    %% Crear robot
    robot = plot(startNode(2), startNode(1), 'bo', ...
        'MarkerSize', 14, 'MarkerFaceColor', 'b');

    %% Configuración de replanificación
    triggerStep = round(size(pathInitial, 1) * 0.45);
    replanned = false;

    path = pathInitial;
    totalTravelCostBeforeReplan = 0;
    replanCost = 0;
    replanTime = 0;
    exploredReplan = [];
    pathReplan = [];

    i = 1;

    while i <= size(path, 1)

        currentNode = path(i, :);

        set(robot, 'XData', currentNode(2), 'YData', currentNode(1));
        title(sprintf('%s - Robot siguiendo ruta inicial', algName));

        drawnow;

        frame = getframe(fig);
        writeVideo(video, frame);

        pause(0.08);

        %% Acumular costo recorrido antes de replanificar
        if i > 1 && ~replanned
            totalTravelCostBeforeReplan = totalTravelCostBeforeReplan + ...
                distanceCost(path(i-1,:), path(i,:));
        end

        %% Obstáculo dinámico
        if i == triggerStep && ~replanned

            title(sprintf('%s - Obstáculo detectado, replanificando', algName));
            drawnow;

            %% Crear obstáculo sobre la ruta futura
            blockStart = min(i + 4, size(path, 1));
            blockEnd   = min(i + 8, size(path, 1));

            dynamicObstacleCells = path(blockStart:blockEnd, :);

            for j = 1:size(dynamicObstacleCells, 1)
                r = dynamicObstacleCells(j, 1);
                c = dynamicObstacleCells(j, 2);

                if ~(r == goalNode(1) && c == goalNode(2))
                    map(r, c) = 1;
                end
            end

            %% Dibujar obstáculo dinámico
            plot(dynamicObstacleCells(:,2), dynamicObstacleCells(:,1), 's', ...
                'MarkerSize', 15, ...
                'MarkerFaceColor', 'y', ...
                'MarkerEdgeColor', 'k');

            text(currentNode(2)+0.5, currentNode(1)-0.5, ...
                'Obstáculo dinámico', ...
                'Color', 'k', ...
                'FontWeight', 'bold', ...
                'BackgroundColor', 'y');

            for k = 1:35
                frame = getframe(fig);
                writeVideo(video, frame);
            end

            %% Marcar ruta inicial como obsoleta
            set(initialPathPlot, 'Color', [1 0.65 0.65], 'LineStyle', '--');
            set(initialPointsPlot, ...
                'MarkerFaceColor', [1 0.65 0.65], ...
                'MarkerEdgeColor', [1 0.65 0.65]);

            %% Replanificar desde posición actual
            tic;
            [pathReplan, exploredReplan, replanCost, successReplan] = pathPlannerAnimated( ...
                map, currentNode, goalNode, algorithm, video, fig, exploredColor);
            replanTime = toc;

            if ~successReplan
                error('%s no pudo replanificar.', algName);
            end

            %% Dibujar nueva ruta
            plot(pathReplan(:,2), pathReplan(:,1), ...
                '-', 'Color', replanColor, 'LineWidth', 3);

            plot(pathReplan(:,2), pathReplan(:,1), ...
                'o', 'Color', replanColor, ...
                'MarkerSize', 5, 'MarkerFaceColor', replanColor);

            title(sprintf('%s - Nueva ruta generada', algName));

            for k = 1:35
                frame = getframe(fig);
                writeVideo(video, frame);
            end

            %% Seguir nueva ruta
            path = pathReplan;
            i = 1;
            replanned = true;
            continue;
        end

        i = i + 1;
    end

    %% Llegada a meta
    title(sprintf('%s - Robot llegó a la meta', algName));

    for k = 1:25
        frame = getframe(fig);
        writeVideo(video, frame);
    end

    %% Costo total
    if replanned
        totalCost = totalTravelCostBeforeReplan + replanCost;
    else
        totalCost = initialCost;
    end

    totalExplored = size(exploredInitial, 1) + size(exploredReplan, 1);
    totalTime = initialTime + replanTime;

    %% Cuadro de métricas
    infoText = sprintf([ ...
        '%s\n' ...
        'Costo inicial: %.2f\n' ...
        'Costo replanificado: %.2f\n' ...
        'Costo total: %.2f\n' ...
        'Explorados inicial: %d\n' ...
        'Explorados replan: %d\n' ...
        'Explorados total: %d\n' ...
        'Tiempo total: %.6f s'], ...
        algName, ...
        initialCost, ...
        replanCost, ...
        totalCost, ...
        size(exploredInitial,1), ...
        size(exploredReplan,1), ...
        totalExplored, ...
        totalTime);

    text(1, 2, infoText, ...
        'BackgroundColor', 'white', ...
        'EdgeColor', 'black', ...
        'FontSize', 8);

    for k = 1:40
        frame = getframe(fig);
        writeVideo(video, frame);
    end

    %% Guardar resultados
    result.algorithm = algName;
    result.initialCost = initialCost;
    result.replanCost = replanCost;
    result.totalCost = totalCost;
    result.initialExplored = size(exploredInitial, 1);
    result.replanExplored = size(exploredReplan, 1);
    result.totalExplored = totalExplored;
    result.initialTime = initialTime;
    result.replanTime = replanTime;
    result.totalTime = totalTime;
    result.replanned = replanned;
end


%% =================================================================
%  FUNCIÓN ANIMADA: A* O DIJKSTRA
%  =================================================================
function [path, exploredNodes, finalCost, success] = pathPlannerAnimated( ...
    map, startNode, goalNode, algorithm, video, fig, pointColor)

    [rows, cols] = size(map);

    movements = [
        -1,  0, 1.0;
         1,  0, 1.0;
         0, -1, 1.0;
         0,  1, 1.0;
        -1, -1, sqrt(2);
        -1,  1, sqrt(2);
         1, -1, sqrt(2);
         1,  1, sqrt(2)
    ];

    gCost = inf(rows, cols);
    fCost = inf(rows, cols);

    parentRow = zeros(rows, cols);
    parentCol = zeros(rows, cols);

    closedList = false(rows, cols);
    openList = [];

    sr = startNode(1);
    sc = startNode(2);

    gr = goalNode(1);
    gc = goalNode(2);

    gCost(sr, sc) = 0;

    if algorithm == "astar"
        fCost(sr, sc) = heuristic(startNode, goalNode);
    elseif algorithm == "dijkstra"
        fCost(sr, sc) = 0;
    else
        error('Algoritmo no válido.');
    end

    openList = [sr, sc, fCost(sr, sc)];

    exploredNodes = [];
    path = [];
    finalCost = inf;
    success = false;

    while ~isempty(openList)

        [~, idx] = min(openList(:,3));

        current = openList(idx, :);
        currentRow = current(1);
        currentCol = current(2);

        openList(idx, :) = [];

        if closedList(currentRow, currentCol)
            continue;
        end

        closedList(currentRow, currentCol) = true;
        exploredNodes = [exploredNodes; currentRow, currentCol];

        plot(currentCol, currentRow, '.', ...
            'Color', pointColor, ...
            'MarkerSize', 9);

        drawnow;

        frame = getframe(fig);
        writeVideo(video, frame);

        pause(0.004);

        if currentRow == gr && currentCol == gc
            success = true;
            finalCost = gCost(gr, gc);
            path = reconstructPath(parentRow, parentCol, startNode, goalNode);
            return;
        end

        for i = 1:size(movements, 1)

            newRow = currentRow + movements(i, 1);
            newCol = currentCol + movements(i, 2);
            moveCost = movements(i, 3);

            if newRow < 1 || newRow > rows || newCol < 1 || newCol > cols
                continue;
            end

            if map(newRow, newCol) == 1
                continue;
            end

            if closedList(newRow, newCol)
                continue;
            end

            tentativeG = gCost(currentRow, currentCol) + moveCost;

            if tentativeG < gCost(newRow, newCol)

                parentRow(newRow, newCol) = currentRow;
                parentCol(newRow, newCol) = currentCol;

                gCost(newRow, newCol) = tentativeG;

                if algorithm == "astar"
                    h = heuristic([newRow, newCol], goalNode);
                else
                    h = 0;
                end

                fCost(newRow, newCol) = tentativeG + h;

                openList = [openList; newRow, newCol, fCost(newRow, newCol)];
            end
        end
    end
end


%% =================================================================
%  HEURÍSTICA A*
%  =================================================================
function h = heuristic(node, goalNode)

    h = sqrt((node(1) - goalNode(1))^2 + ...
             (node(2) - goalNode(2))^2);
end


%% =================================================================
%  RECONSTRUIR RUTA
%  =================================================================
function path = reconstructPath(parentRow, parentCol, startNode, goalNode)

    path = goalNode;
    current = goalNode;

    while ~(current(1) == startNode(1) && current(2) == startNode(2))

        r = current(1);
        c = current(2);

        pr = parentRow(r, c);
        pc = parentCol(r, c);

        if pr == 0 && pc == 0
            path = [];
            return;
        end

        current = [pr, pc];
        path = [current; path];
    end
end


%% =================================================================
%  COSTO ENTRE DOS NODOS
%  =================================================================
function cost = distanceCost(nodeA, nodeB)

    dr = abs(nodeA(1) - nodeB(1));
    dc = abs(nodeA(2) - nodeB(2));

    if dr == 1 && dc == 1
        cost = sqrt(2);
    else
        cost = 1;
    end
end