function ChillPillLiveLogger(portName, baudRate)
%CHILLPILLLIVELOGGER Stream, visualise, and persist ChillPill telemetry.
%   ChillPillLiveLogger(PORT, BAUD) opens a serial connection to the
%   controller, parses the bracketed diagnostic stream emitted by the
%   firmware, and continuously plots key signals. All parsed samples are
%   cached in memory and exported to CSV, MAT, and raw-text files when the
%   session ends.  Close the figure window or press Ctrl+C in the MATLAB
%   console to stop logging gracefully.
%
%   Usage examples:
%       ChillPillLiveLogger                 % interactive port selection
%       ChillPillLiveLogger("COM5", 115200) % explicit configuration
%
%   The script expects firmware frames with the following format:
%       [hh.mm.ss][T1][T2][T3][motor_rpm][motor_current][compressor_rpm][ms]
%   Temperatures are reported in degrees Celsius, motor current in amps,
%   and rotational speeds in RPM.  The "ms" field is the millisecond tick
%   counter emitted by the controller.
%
%   Generated artefacts are written to Modelling/output/ using the
%   timestamp of the session start:
%       chillpill_data_<timestamp>.csv  - parsed numeric samples
%       chillpill_data_<timestamp>.mat  - MAT file containing the table and logs
%       chillpill_raw_<timestamp>.txt   - raw bracketed frames for replays
%       chillpill_log_<timestamp>.txt   - non-parsed textual output
%
%   Requires MATLAB R2019b or newer for the serialport interface.

% Copyright (c) 2024.

arguments
    portName (1, 1) string = ""
    baudRate (1, 1) double {mustBePositive} = 115200
end

cleanupObjs = {}; %#ok<NASGU> for lint happiness

if portName == ""
    portName = selectSerialPort();
end
portName = string(portName);
portNameChar = char(portName);

info = struct();
info.startTime = datetime('now');
info.timestamp = datestr(info.startTime, 'yyyymmdd_HHMMSS');
info.outputDir = fullfile(fileparts(mfilename('fullpath')), 'output');
if ~exist(info.outputDir, 'dir')
    mkdir(info.outputDir);
end
info.dataCsv = fullfile(info.outputDir, "chillpill_data_" + info.timestamp + ".csv");
info.dataMat = fullfile(info.outputDir, "chillpill_data_" + info.timestamp + ".mat");
info.rawTxt  = fullfile(info.outputDir, "chillpill_raw_"  + info.timestamp + ".txt");
info.logTxt  = fullfile(info.outputDir, "chillpill_log_"  + info.timestamp + ".txt");

fprintf("ChillPill telemetry logger\n=============================\n");
fprintf(" Port      : %s\n", portNameChar);
fprintf(" Baud rate : %d\n", baudRate);
fprintf(" Output dir: %s\n\n", char(info.outputDir));

s = serialport(portNameChar, baudRate, ...
    'DataBits', 8, ...
    'Parity', 'none', ...
    'StopBits', 1);
configureTerminator(s, "CR/LF");
s.Timeout = 2; % seconds
flush(s);
cleanupObjs{end+1} = onCleanup(@()safeSerialClose(s)); %#ok<AGROW>

telemetry = struct();
telemetry.timeMin      = [];
telemetry.tempEvapIn   = [];
telemetry.tempEvapOut  = [];
telemetry.tempBowl     = [];
telemetry.motorRpm     = [];
telemetry.motorCurrent = [];
telemetry.compRpm      = [];
telemetry.tickMs       = [];
telemetry.diffT1T2     = [];
rawFrames = strings(0, 1);
logLines  = strings(0, 1);

fig = figure('Name', 'ChillPill Live Telemetry', ...
             'NumberTitle', 'off', ...
             'Color', 'w');
cleanupObjs{end+1} = onCleanup(@()cleanupFigure(fig)); %#ok<AGROW>

tile = tiledlayout(fig, 3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
nexttile(tile);
axT3 = gca;
plotT3 = animatedline(axT3, 'Color', [0 0.4470 0.7410], 'LineWidth', 1.5);
title(axT3, 'Evaporator Outlet (T3)'); xlabel(axT3, 'Time [min]'); ylabel(axT3, 'Temp [°C]'); grid(axT3, 'on');

nexttile(tile);
axT12 = gca;
plotT1 = animatedline(axT12, 'Color', [0.8500 0.3250 0.0980], 'LineWidth', 1.2);
plotT2 = animatedline(axT12, 'Color', [0.4660 0.6740 0.1880], 'LineWidth', 1.2);
title(axT12, 'Evaporator Inlet vs Bowl'); xlabel(axT12, 'Time [min]'); ylabel(axT12, 'Temp [°C]'); grid(axT12, 'on');
legend(axT12, {'T1 (Evap In)', 'T2 (Bowl)'}, 'Location', 'best');

nexttile(tile);
axDiff = gca;
plotDiff = animatedline(axDiff, 'Color', [0.4940 0.1840 0.5560], 'LineWidth', 1.2);
title(axDiff, 'Temperature Δ (T1 - T2)'); xlabel(axDiff, 'Time [min]'); ylabel(axDiff, 'Temp [°C]'); grid(axDiff, 'on');

nexttile(tile);
axCurrent = gca;
plotCurrent = animatedline(axCurrent, 'Color', [0 0 0], 'LineWidth', 1.2);
title(axCurrent, 'Auger Motor Current'); xlabel(axCurrent, 'Time [min]'); ylabel(axCurrent, 'Current [A]'); grid(axCurrent, 'on');

nexttile(tile);
axComp = gca;
plotComp = animatedline(axComp, 'Color', [0 0.4470 0.7410], 'LineWidth', 1.2);
title(axComp, 'Compressor RPM'); xlabel(axComp, 'Time [min]'); ylabel(axComp, 'RPM'); grid(axComp, 'on');

nexttile(tile);
axMotor = gca;
plotMotor = animatedline(axMotor, 'Color', [0.3010 0.7450 0.9330], 'LineWidth', 1.2);
title(axMotor, 'Auger Motor RPM'); xlabel(axMotor, 'Time [min]'); ylabel(axMotor, 'RPM'); grid(axMotor, 'on');

printHeader();
lastConsolePrint = tic;

try
    while ishandle(fig)
        if s.NumBytesAvailable <= 0
            pause(0.01);
            continue;
        end

        line = readline(s);
        if isempty(line)
            continue;
        end

        frame = string(strtrim(line));
        [parsed, sample] = parseFrame(frame);
        if parsed
            rawFrames(end+1, 1) = frame; %#ok<AGROW>

            telemetry.timeMin(end+1, 1)      = sample.timeMin; %#ok<AGROW>
            telemetry.tempEvapIn(end+1, 1)   = sample.tempEvapIn; %#ok<AGROW>
            telemetry.tempEvapOut(end+1, 1)  = sample.tempEvapOut; %#ok<AGROW>
            telemetry.tempBowl(end+1, 1)     = sample.tempBowl; %#ok<AGROW>
            telemetry.motorRpm(end+1, 1)     = sample.motorRpm; %#ok<AGROW>
            telemetry.motorCurrent(end+1, 1) = sample.motorCurrent; %#ok<AGROW>
            telemetry.compRpm(end+1, 1)      = sample.compRpm; %#ok<AGROW>
            telemetry.tickMs(end+1, 1)       = sample.tickMs; %#ok<AGROW>
            telemetry.diffT1T2(end+1, 1)     = sample.deltaT; %#ok<AGROW>

            addpoints(plotT3,   sample.timeMin, sample.tempEvapOut);
            addpoints(plotT1,   sample.timeMin, sample.tempEvapIn);
            addpoints(plotT2,   sample.timeMin, sample.tempBowl);
            addpoints(plotDiff, sample.timeMin, sample.deltaT);
            addpoints(plotCurrent, sample.timeMin, sample.motorCurrent);
            addpoints(plotComp, sample.timeMin, sample.compRpm);
            addpoints(plotMotor, sample.timeMin, sample.motorRpm);

            axT3.XLim      = autoXLimits(axT3.XLim, sample.timeMin);
            axT12.XLim     = autoXLimits(axT12.XLim, sample.timeMin);
            axDiff.XLim    = autoXLimits(axDiff.XLim, sample.timeMin);
            axCurrent.XLim = autoXLimits(axCurrent.XLim, sample.timeMin);
            axComp.XLim    = autoXLimits(axComp.XLim, sample.timeMin);
            axMotor.XLim   = autoXLimits(axMotor.XLim, sample.timeMin);

            if toc(lastConsolePrint) > 1.0
                printSample(sample);
                lastConsolePrint = tic;
            end

            drawnow limitrate;
        else
            logLines(end+1, 1) = frame; %#ok<AGROW>
            fprintf("%s\n", frame);
        end
    end
catch ME
    fprintf(2, "\nLogging stopped due to: %s\n", ME.message);
end

saveOutputs(telemetry, rawFrames, logLines, info);

end

function portName = selectSerialPort()
    available = serialportlist("available");
    if isempty(available)
        error("No serial ports detected. Connect the USB-UART adapter and retry.");
    end

    fprintf("Available serial ports:\n");
    for idx = 1:numel(available)
        fprintf("  [%d] %s\n", idx, char(available(idx)));
    end

    selection = input("Select port index (press Enter for 1): ");
    if isempty(selection)
        selection = 1;
    end
    if selection < 1 || selection > numel(available)
        error("Invalid port selection.");
    end
    portName = available(selection);
end

function [parsed, sample] = parseFrame(frame)
    parsed = false;
    sample = struct();

    tokens = regexp(frame, '\\[(.*?)\\]', 'tokens');
    tokens = [tokens{:}];
    if numel(tokens) < 8
        return;
    end

    timeParts = strsplit(tokens{1}, '.');
    if numel(timeParts) ~= 3
        return;
    end

    vals = str2double([timeParts, tokens(2:8)]);
    if any(isnan(vals))
        return;
    end

    hh = vals(1); mm = vals(2); ss = vals(3);
    sample.timeMin      = hh * 60 + mm + (ss / 60);
    sample.tempEvapIn   = vals(4);
    sample.tempBowl     = vals(5);
    sample.tempEvapOut  = vals(6);
    sample.motorRpm     = vals(7);
    sample.motorCurrent = vals(8);
    sample.compRpm      = vals(9);
    sample.tickMs       = vals(10);
    sample.deltaT       = sample.tempEvapIn - sample.tempBowl;

    parsed = true;
end

function limits = autoXLimits(limits, newTime)
    if ~isfinite(newTime)
        return;
    end

    if newTime > limits(2)
        span = max(150, newTime + 5);
        limits = [0, span];
    end
end

function printHeader()
    fprintf('\n%-9s %-8s %-8s %-8s %-9s %-9s %-12s %-8s\n', ...
        'Time[min]', 'T1[C]', 'T2[C]', 'T3[C]', 'ΔT[C]', 'Motor[A]', 'Comp[RPM]', 'Auger');
    fprintf('%s\n', repmat('-', 1, 78));
end

function printSample(sample)
    fprintf('%-9.2f %-8.2f %-8.2f %-8.2f %-9.2f %-9.2f %-12.0f %-8.0f\n', ...
        sample.timeMin, sample.tempEvapIn, sample.tempBowl, sample.tempEvapOut, ...
        sample.deltaT, sample.motorCurrent, sample.compRpm, sample.motorRpm);
end

function saveOutputs(telemetry, rawFrames, logLines, info)
    numSamples = numel(telemetry.timeMin);
    fprintf('\nCaptured %d parsed samples.\n', numSamples);

    if numSamples > 0
        T = table(telemetry.timeMin, telemetry.tempEvapIn, telemetry.tempBowl, ...
                 telemetry.tempEvapOut, telemetry.diffT1T2, telemetry.motorCurrent, ...
                 telemetry.motorRpm, telemetry.compRpm, telemetry.tickMs, ...
                 'VariableNames', {'TimeMin', 'TempEvapInC', 'TempBowlC', ...
                                   'TempEvapOutC', 'DeltaT_C', 'MotorCurrentA', ...
                                   'MotorRpm', 'CompressorRpm', 'TickMs'});
        writetable(T, char(info.dataCsv));
        save(char(info.dataMat), 'T', 'rawFrames', 'logLines', 'info');
        fprintf(' Saved parsed data table to %s\n', char(info.dataCsv));
        fprintf(' Saved MATLAB workspace snapshot to %s\n', char(info.dataMat));
    else
        fprintf(' No parsed samples available for CSV/MAT export.\n');
    end

    if ~isempty(rawFrames)
        fid = fopen(char(info.rawTxt), 'w');
        if fid > 0
            fprintf(fid, '%s\n', rawFrames);
            fclose(fid);
            fprintf(' Saved raw frames to %s\n', char(info.rawTxt));
        else
            warning('ChillPillLiveLogger:FileIO', 'Failed to write %s', info.rawTxt);
        end
    end

    if ~isempty(logLines)
        fid = fopen(char(info.logTxt), 'w');
        if fid > 0
            fprintf(fid, '%s\n', logLines);
            fclose(fid);
            fprintf(' Saved non-parsed logs to %s\n', char(info.logTxt));
        else
            warning('ChillPillLiveLogger:FileIO', 'Failed to write %s', info.logTxt);
        end
    end

    fprintf('\nLogging session finished.\n');
end

function safeSerialClose(s)
    if ~isempty(s) && isvalid(s)
        try %#ok<TRYNC>
            flush(s);
            clear s;
        catch
        end
    end
end

function cleanupFigure(fig)
    if ~isempty(fig) && isvalid(fig)
        try %#ok<TRYNC>
            close(fig);
        catch
        end
    end
end
