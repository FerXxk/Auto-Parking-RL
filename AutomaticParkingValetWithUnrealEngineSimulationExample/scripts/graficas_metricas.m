% =====================================================================
% SCRIPT 2: POST-PROCESADO ESTADÍSTICO Y GENERACIÓN DE DASHBOARD (CON ERRORES)
% =====================================================================
clc; clear; close all;

ficheroDatos = 'SAC_100_Episodios.mat';

% Verificar si existe el archivo antes de procesar
if ~exist(ficheroDatos, 'file')
    error('❌ No se encuentra el archivo "%s". Ejecuta primero el Script 1.', ficheroDatos);
end

fprintf('📂 Cargando métricas históricas de "%s"...\n', ficheroDatos);
load(ficheroDatos);
numEpisodiosTest = length(rewardsTotales);

% ---------------------------------------------------------------------
% 1. CONSOLIDACIÓN MATEMÁTICA DE MÉTRICAS (KPIs)
% ---------------------------------------------------------------------
numExitos = sum(resultadosEpisodios == 1);
porcentajeExito = (numExitos / numEpisodiosTest) * 100;
porcentajeColision = 100 - porcentajeExito;
esfuerzoMedio = mean(variacionesVolanteLocales(variacionesVolanteLocales > 0));
distanciasReales = distanciasMinimasLocales(~isnan(distanciasMinimasLocales) & distanciasMinimasLocales > 0);

% --- NUEVO: Segmentación de errores SOLO para intentos EXITOSOS ---
indicesExito = (resultadosEpisodios == 1);

if any(indicesExito)
    errorPosicionMedioExito = mean(erroresPosicionFinal(indicesExito));
    % Usamos el valor absoluto para la orientación para evitar que desviaciones
    % positivas (derecha) y negativas (izquierda) se cancelen entre sí.
    errorOrientacionMedioExito = mean(abs(erroresOrientacionFinal(indicesExito)));
else
    errorPosicionMedioExito = NaN;
    errorOrientacionMedioExito = NaN;
end

% ---------------------------------------------------------------------
% 2. PRESENTACIÓN DE RESULTADOS POR CONSOLA
% ---------------------------------------------------------------------
fprintf('\n================ MÉTRICAS DE VALIDACIÓN ================\n');
fprintf('Episodios Evaluados                      : %d\n', numEpisodiosTest);
fprintf('Recompensa Media en Test                 : %.2f\n', mean(rewardsTotales));
fprintf('Tasa de Éxito Comercial (Aparcado Seguro): %.1f%%\n', porcentajeExito);
fprintf('Tasa de Siniestralidad (Choques/Fallas)  : %.1f%%\n', porcentajeColision);
fprintf('Tiempo Medio de Maniobra                 : %.1f pasos de control\n', mean(pasosPorEpisodio));
fprintf('Esfuerzo Medio de Control (Volante)      : %.4f rad/paso\n', esfuerzoMedio);
if ~isempty(distanciasReales)
    fprintf('Margen de Seguridad Promedio (Lidar)     : %.2f metros\n', mean(distanciasReales));
else
    fprintf('Margen de Seguridad Promedio (Lidar)     : Fuera de rango de obstáculos\n');
end
fprintf('--------------------------------------------------------\n');
fprintf('📍 PRECISIÓN SÓLO EN EPISODIOS EXITOSOS:\n');
if ~isnan(errorPosicionMedioExito)
    fprintf(' -> Error de Posición Medio al Aparcar   : %.3f metros\n', errorPosicionMedioExito);
    fprintf(' -> Error de Orientación Medio Absoluto  : %.3f rad (%.2f°)\n', ...
        errorOrientacionMedioExito, rad2deg(errorOrientacionMedioExito));
else
    fprintf(' ❌ No hubo episodios exitosos para calcular la precisión final.\n');
end
fprintf('========================================================\n');

% ---------------------------------------------------------------------
% 3. GENERACIÓN DEL DASHBOARD GRÁFICO AMPLIADO (Matriz 2x2)
% ---------------------------------------------------------------------
% Ampliamos el alto de la ventana a 800 píxeles para acomodar los histogramas abajo
figure('Name', 'Dashboard de Validación de Agente SAC con Análisis de Error', ...
       'NumberTitle', 'off', 'Position', [100, 50, 1100, 800]);

% --- Subplot 1: Rendimiento Temporal por Episodio ---
subplot(2, 2, 1);
yyaxis left
plot(rewardsTotales, '-o', 'LineWidth', 1.5, 'Color', [0 0.4470 0.7410]);
ylabel('Recompensa Total Acumulada'); 
ax = gca; ax.YColor = [0 0.4470 0.7410];
yyaxis right
bar(pasosPorEpisodio, 'FaceColor', [0.8500 0.3250 0.0980], 'FaceAlpha', 0.3);
ylabel('Tiempo de Maniobra (Pasos)'); 
ax = gca; ax.YColor = [0.8500 0.3250 0.0980];
grid on; 
xlabel('Número de Episodio de Test'); 
title('Rendimiento del Agente por Episodio');

% --- Subplot 2: Diagrama de Tarta (KPI de Seguridad) ---
subplot(2, 2, 2);
datosQuesito = [porcentajeExito, porcentajeColision];
etiquetas = {['Éxito Seguro (' num2str(porcentajeExito, '%.1f') '%)'], ...
             ['Siniestros/Fallas (' num2str(porcentajeColision, '%.1f') '%)']};
if porcentajeColision == 0
    p = pie(datosQuesito(1), etiquetas(1)); 
    p(1).FaceColor = [0.4660 0.6740 0.1880]; % Verde
else
    p = pie(datosQuesito, etiquetas);
    p(1).FaceColor = [0.4660 0.6740 0.1880]; % Verde Éxito
    p(3).FaceColor = [0.6350 0.0780 0.1840]; % Rojo Fallo
end
title('Tasa de Seguridad Comercial (KPI)');

% --- NUEVO Subplot 3: Histograma del Error de Posición Final ---
subplot(2, 2, 3);
% Graficamos el histograma de todos los datos
histogram(erroresPosicionFinal, 'FaceColor', [0.9290 0.6940 0.1250], 'EdgeColor', 'w');
grid on;
xlabel('Error de Posición Lineal (metros)');
ylabel('Frecuencia (Episodios)');
title('Distribución del Error de Posición Final');

% Si hay éxitos, añadimos una línea vertical que marque la media de los correctos
if ~isnan(errorPosicionMedioExito)
    hold on;
    xline(errorPosicionMedioExito, '--r', 'LineWidth', 2, ...
          'Label', ['Media Éxitos: ' num2str(errorPosicionMedioExito, '%.2f') 'm'], ...
          'LabelVerticalAlignment', 'top', 'LabelHorizontalAlignment', 'right');
    hold off;
end

% --- NUEVO Subplot 4: Histograma del Error de Orientación Final ---
subplot(2, 2, 4);
% Convertimos a grados para que sea más intuitivo de interpretar visualmente
erroresOrientacionGrados = rad2deg(erroresOrientacionFinal);
histogram(erroresOrientacionGrados, 'FaceColor', [0.4940 0.1840 0.5560], 'EdgeColor', 'w');
grid on;
xlabel('Error de Orientación (grados °)');
ylabel('Frecuencia (Episodios)');
title('Distribución del Error de Alineación (Yaw)');

% Si hay éxitos, añadimos la línea vertical indicando el desvío medio en grados
if ~isnan(errorOrientacionMedioExito)
    hold on;
    errorOrientacionGradosMedio = rad2deg(errorOrientacionMedioExito);
    xline(errorOrientacionGradosMedio, '--r', 'LineWidth', 2, ...
          'Label', ['Media Éxitos: ' num2str(errorOrientacionGradosMedio, '%.1f') '°'], ...
          'LabelVerticalAlignment', 'top');
    hold off;
end

% Reactivar avisos por si acaso
warning('on', 'Simulink:Commands:ChangeOnDisk');
fprintf('📊 Dashboard avanzado generado en pantalla con éxito.\n');