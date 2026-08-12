function T = exportDepartureTimeLatency(departureTimes, detailsCell, outputDir, replanningIntervalS)
%EXPORTDEPARTURETIMELATENCY Export stage timing and RescueA evidence.
%   T = exportDepartureTimeLatency(departureTimes, detailsCell, outputDir)
%   writes departure_time_latency_details.csv and
%   departure_time_hardware.csv. The optional replanningIntervalS is used
%   only to calculate a timing-margin field; leave it as NaN unless an
%   operational replanning interval has been defined in the manuscript.

    if nargin < 3 || isempty(outputDir), outputDir = pwd; end
    if nargin < 4 || isempty(replanningIntervalS)
        replanningIntervalS = NaN;
    end
    if ~exist(outputDir,'dir'), mkdir(outputDir); end

    departureTimes = departureTimes(:);
    n = numel(departureTimes);
    if numel(detailsCell) ~= n
        error('departureTimes and detailsCell must have the same length.');
    end

    chosen_path_type = repmat({''},n,1);
    total_s = nan(n,1);
    initialization_s = nan(n,1);
    warm_start_s = nan(n,1);
    main_optimization_s = nan(n,1);
    topk_s = nan(n,1);
    topk_generation_s = nan(n,1);
    smoothing_s = nan(n,1);
    topk_evaluation_s = nan(n,1);
    topk_selection_s = nan(n,1);
    topk_overhead_s = nan(n,1);
    rescueA_s = nan(n,1);
    rescueB_s = nan(n,1);
    other_s = nan(n,1);

    topk_parent_count = nan(n,1);
    raw_generated = nan(n,1);
    smooth_generated = nan(n,1);
    mild_generated = nan(n,1);
    topk_candidates_evaluated = nan(n,1);

    rescueA_triggered = false(n,1);
    rescueA_executed = false(n,1);
    initial_conflict_segments = zeros(n,1);
    rescueA_diagnostic_evaluations = zeros(n,1);
    rescueA_candidates_generated = zeros(n,1);
    rescueA_candidates_evaluated = zeros(n,1);
    rescueA_final_evaluations = zeros(n,1);
    rescueA_successful_insertions = zeros(n,1);
    rescueA_adopted = false(n,1);
    rescueA_fraction_total_pct = nan(n,1);

    rescueB_triggered = false(n,1);
    rescueB_executed = false(n,1);
    rescueB_adopted = false(n,1);

    J_final = nan(n,1);
    feasible = false(n,1);
    replanning_interval_s = repmat(replanningIntervalS,n,1);
    within_replanning_interval = nan(n,1);

    for i = 1:n
        d = detailsCell{i};
        if ~isstruct(d), continue; end

        chosen_path_type{i} = fieldOr(d,'chosen_path_type','');
        J_final(i) = fieldOr(d,'J_final',NaN);
        feasible(i) = logical(fieldOr(d,'feasible',false));

        tm = fieldOr(d,'timing',struct());
        total_s(i) = fieldOr(tm,'total_s',NaN);
        initialization_s(i) = fieldOr(tm,'initialization_s',NaN);
        warm_start_s(i) = fieldOr(tm,'warm_start_s',NaN);
        main_optimization_s(i) = fieldOr(tm,'main_optimization_s', ...
            fieldOr(tm,'search_s',NaN));
        topk_s(i) = fieldOr(tm,'topk_s',NaN);
        topk_generation_s(i) = fieldOr(tm,'topk_generation_s',NaN);
        smoothing_s(i) = fieldOr(tm,'smoothing_s',NaN);
        topk_evaluation_s(i) = fieldOr(tm,'topk_evaluation_s',NaN);
        topk_selection_s(i) = fieldOr(tm,'topk_selection_s',NaN);
        topk_overhead_s(i) = fieldOr(tm,'topk_overhead_s',NaN);
        rescueA_s(i) = fieldOr(tm,'rescueA_s',NaN);
        rescueB_s(i) = fieldOr(tm,'rescueB_s',NaN);
        other_s(i) = fieldOr(tm,'other_s',NaN);

        cs = fieldOr(d,'candidate_stats',struct());
        topk_parent_count(i) = fieldOr(cs,'topk_parent_count',NaN);
        raw_generated(i) = fieldOr(cs,'raw_generated',NaN);
        smooth_generated(i) = fieldOr(cs,'smooth_generated',NaN);
        mild_generated(i) = fieldOr(cs,'mild_generated',NaN);
        topk_candidates_evaluated(i) = ...
            fieldOr(cs,'topk_candidates_evaluated',NaN);

        ra = fieldOr(d,'rescueA_stats',struct());
        rescueA_triggered(i) = logical(fieldOr(ra,'triggered',false));
        rescueA_executed(i) = logical(fieldOr(ra,'executed',false));
        initial_conflict_segments(i) = ...
            fieldOr(ra,'initial_conflict_segments',0);
        rescueA_diagnostic_evaluations(i) = ...
            fieldOr(ra,'diagnostic_evaluations',0);
        rescueA_candidates_generated(i) = ...
            fieldOr(ra,'candidates_generated',0);
        rescueA_candidates_evaluated(i) = ...
            fieldOr(ra,'candidates_evaluated',0);
        rescueA_final_evaluations(i) = ...
            fieldOr(ra,'final_evaluations',0);
        rescueA_successful_insertions(i) = ...
            fieldOr(ra,'successful_insertions',0);
        rescueA_adopted(i) = logical(fieldOr(ra,'adopted',false));

        rb = fieldOr(d,'rescueB_stats',struct());
        rescueB_triggered(i) = logical(fieldOr(rb,'triggered',false));
        rescueB_executed(i) = logical(fieldOr(rb,'executed',false));
        rescueB_adopted(i) = logical(fieldOr(rb,'adopted',false));

        if isfinite(total_s(i)) && total_s(i) > 0
            rescueA_fraction_total_pct(i) = 100*rescueA_s(i)/total_s(i);
        end
        if isfinite(replanningIntervalS)
            within_replanning_interval(i) = ...
                double(total_s(i) <= replanningIntervalS);
        end
    end

    departure_time_s = departureTimes;
    T = table(departure_time_s,chosen_path_type,J_final,feasible, ...
        total_s,initialization_s,warm_start_s,main_optimization_s, ...
        topk_s,topk_generation_s,smoothing_s,topk_evaluation_s, ...
        topk_selection_s,topk_overhead_s,rescueA_s,rescueB_s,other_s, ...
        topk_parent_count,raw_generated,smooth_generated,mild_generated, ...
        topk_candidates_evaluated,rescueA_triggered,rescueA_executed, ...
        initial_conflict_segments,rescueA_diagnostic_evaluations, ...
        rescueA_candidates_generated,rescueA_candidates_evaluated, ...
        rescueA_final_evaluations,rescueA_successful_insertions, ...
        rescueA_adopted,rescueA_fraction_total_pct,rescueB_triggered, ...
        rescueB_executed,rescueB_adopted,replanning_interval_s, ...
        within_replanning_interval);

    writetable(T,fullfile(outputDir,'departure_time_latency_details.csv'));
    writeHardwareTable(outputDir);
    writeT240Report(T,outputDir);
    fprintf('  Departure-time latency data saved: %s\n', ...
        fullfile(outputDir,'departure_time_latency_details.csv'));
end

function writeT240Report(T,outputDir)
    idx = find(T.departure_time_s == 240,1);
    if isempty(idx), return; end

    fid = fopen(fullfile(outputDir,'departure_time_t240_rescueA_report.txt'),'w');
    if fid < 0, return; end
    cleaner = onCleanup(@() fclose(fid));

    fprintf(fid,'DEPARTURE-TIME CASE t = 240 s\n');
    fprintf(fid,'Chosen output mode: %s\n',T.chosen_path_type{idx});
    fprintf(fid,'Initial dynamic-conflict segments: %d\n', ...
        T.initial_conflict_segments(idx));
    fprintf(fid,'RescueA candidates generated: %d\n', ...
        T.rescueA_candidates_generated(idx));
    fprintf(fid,'RescueA candidates evaluated: %d\n', ...
        T.rescueA_candidates_evaluated(idx));
    fprintf(fid,'RescueA successful insertions: %d\n', ...
        T.rescueA_successful_insertions(idx));
    fprintf(fid,'RescueA adopted: %d\n',T.rescueA_adopted(idx));
    fprintf(fid,'RescueA additional computation: %.6f s\n',T.rescueA_s(idx));
    fprintf(fid,'Total planning time: %.6f s\n',T.total_s(idx));
    fprintf(fid,'RescueA fraction of total time: %.3f%%\n', ...
        T.rescueA_fraction_total_pct(idx));

    if isfinite(T.replanning_interval_s(idx))
        fprintf(fid,'Defined replanning interval: %.6f s\n', ...
            T.replanning_interval_s(idx));
        fprintf(fid,'Completed within interval: %d\n', ...
            T.within_replanning_interval(idx));
    else
        fprintf(fid,['Operational compatibility: not evaluated because no ', ...
            'replanning interval was supplied.\n']);
    end
end

function v = fieldOr(s,name,default)
    if isstruct(s) && isfield(s,name) && ~isempty(s.(name))
        v = s.(name);
    else
        v = default;
    end
end

function writeHardwareTable(outputDir)
    cpu = strtrim(getenv('PROCESSOR_IDENTIFIER'));
    logicalProcessors = strtrim(getenv('NUMBER_OF_PROCESSORS'));
    ramGB = NaN;
    try
        [~,sys] = memory;
        ramGB = sys.PhysicalMemory.Total/2^30;
    catch
    end
    try
        os = system_dependent('getos');
    catch
        os = computer;
    end
    try
        release = version('-release');
    catch
        release = 'unknown';
    end

    item = {'run_datetime';'matlab_version';'matlab_release'; ...
        'architecture';'operating_system';'cpu_model'; ...
        'logical_processors';'physical_memory_gb';'parallel_pool_active'; ...
        'timer_scope'};
    value = {datestr(now,31);version;release;computer('arch');os;cpu; ...
        logicalProcessors;num2str(ramGB,'%.2f'); ...
        mat2str(poolIsActive()); ...
        ['complete runRA_ALA call; stage timers are mutually exclusive ', ...
         'except warm_start_s, which is a subcomponent of initialization_s']};
    H = table(item,value);
    writetable(H,fullfile(outputDir,'departure_time_hardware.csv'));
end

function tf = poolIsActive()
    tf = false;
    if exist('gcp','file') == 2
        try
            tf = ~isempty(gcp('nocreate'));
        catch
        end
    end
end