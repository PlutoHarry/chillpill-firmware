Modelling workflow using MATLAB exports
=======================================

The MATLAB logger (`Modelling/ChillPillLiveLogger.m`) drops every capture into
`Modelling/output/` as three artefacts: a CSV table, a MAT file (with the table,
raw frame strings, and metadata), and a plain-text dump of unparsed serial
lines. The CSV/MAT files share the same base name, e.g.:

    chillpill_data_2024_05_15_142233.csv
    chillpill_data_2024_05_15_142233.mat
    chillpill_raw_2024_05_15_142233.txt

Use the following process to turn those recordings into modelling data sets.

1. Load the session into MATLAB (replace `repoRoot` with the location of this
   repository on your machine):

       data = readtable(fullfile(repoRoot, "Modelling", "output", "chillpill_data_YYYY_MM_DD_HHMMSS.csv"));
       % or
       S = load(fullfile(repoRoot, "Modelling", "output", "chillpill_data_YYYY_MM_DD_HHMMSS.mat"));
       data = S.T;   % table with TimeMin, TempEvapInC, TempBowlC, etc.

   The `TimeMin` column is monotonic. Convert it to seconds if you prefer:

       data.TimeSec = data.TimeMin * 60;

2. Convert to a timetable for easy resampling and alignment with other logs:

       tt = table2timetable(data, 'RowTimes', seconds(data.TimeSec));
       tt = sortrows(tt);                     % guarantee chronological order
       tt = retime(tt, 'regular', 'linear', 'TimeStep', seconds(1));

   The resampled timetable provides uniformly spaced points required by most
   identification and optimisation routines.

3. Create modelling inputs/outputs. Typical combinations are:

   * Inputs: compressor RPM (`CompressorRpm`), auger RPM (`MotorRpm`), motor
     current (`MotorCurrentA`).
   * Outputs: evaporator inlet/outlet temperatures, bowl temperature, or the
     derived delta (`DeltaT_C`).

       u = tt{:, {'CompressorRpm', 'MotorRpm', 'MotorCurrentA'}};
       y = tt{:, {'TempEvapInC', 'TempEvapOutC', 'TempBowlC', 'DeltaT_C'}};

4. Fit first-pass dynamic models with the System Identification Toolbox:

       Ts = seconds(tt.Time(2) - tt.Time(1));
       dataId = iddata(y, u, Ts);
       % Example: second-order state-space model
       sys_ss = ssest(dataId, 2);
       compare(dataId, sys_ss);

   Use the residual plots (`resid`), fit percentages, and prediction errors to
   assess whether the selected order is adequate. Iterate on model structure
   (ARX, OE, NLARX) until the residuals resemble white noise.

5. Update the firmware tuning with the fitted model:

   * Translate compressor/auger dynamics into updated limits in
     `Core/Src/control_config.c`.
   * Adjust estimator gains or filters using the time constants extracted from
     the model (`sys_ss.A`/`sys_ss.C`).
   * Revisit PID tuning tables in `Core/Src/pid_controller.c` using pole/zero
     locations that achieve the desired bandwidth and damping.

6. Archive the modelling artefacts:

   * Store the MATLAB model (`sys_ss`, identification data) to the MAT file for
     future reference: `save(..., 'sys_ss', 'dataId', '-append');`
   * Export summary figures (e.g., from `compare` or `step`) to the `Modelling`
     folder so test engineers can reproduce the improvements.

By following this workflow every test run produces a reproducible data set that
feeds directly into control improvements. Use Git branches or the issue tracker
to link MAT files, analysis scripts, and firmware changes so that calibration
updates stay traceable.
