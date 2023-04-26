%% Get every data
test_source = phased_20_230423_134750;
slave_source = phased_21_230423_134749;

% Need this clock for using repeat 20
% Maybe you should wrap your clocklist if you are using FTMClock
test_clock = test_source.MVMExtra.MuClock;
slave_clock = slave_source.MVMExtra.MuClock;

%% Data source
batches = slave_source.splitBundleByField('TxDPASRequest.Stage', 'bundle');

if median(batches(1).TxDPASRequest.Stage) ~= 0
    error('The first batch must be DPASRequest *Training* stage.');
end

%% Configurations

cfoTrackingByTxTiming = true;
% cfoTrackingByTxTiming = false;
useFTMClock = true;

%% Training & Tracking

skipTraining = false;
lastEndingTime = 0;
if ~isempty(batches(1).MVMExtra)
    lastEndingTime = batches(1).MVMExtra.FTMClock(1) / 320e6;
end
startForNextScheduledTime = 0;
SFOResults = [];
ResultsCFO = [];
ResultsSFO = [];
startTime = tic;
wrappingLimit = [];

for groupI = 1 : size(batches, 1)
    currentBatch = batches(groupI);
    currentBatch.doPhaseRegression;
    TrainingStage = median(currentBatch.TxDPASRequest.Stage) == 0;
    BatchLength = mean(currentBatch.TxDPASRequest.BatchLength);
    Interval = mean(currentBatch.TxDPASRequest.Interval);
    Step = mean(currentBatch.TxDPASRequest.Step);
    Sequence = currentBatch.TxDPASRequest.Sequence;
    CarrierFreq = mean(currentBatch.TxDPASRequest.CarrierFrequency);
    SamplingRate = mean(currentBatch.TxDPASRequest.SamplingRate);
    TxBurstDelayNs = mean(currentBatch.TxDPASRequest.TxDelayNs);
    [~, sortIndex] = sort(Sequence);
    PhaseAtDC = currentBatch.PhaseAtDC(sortIndex, :);
    slopes = currentBatch.PhaseSlope(sortIndex, :);
    wrappingLimit = decidePhaseSlopeWrappingLimit(slopes);

    TimeStampTime = double(currentBatch.Basic.Timestamp(sortIndex)) / 1e6;
    ScheduledRxTime_us = ComputeScheduledTime(Interval, Step, Sequence, TxBurstDelayNs / 1e3);
    ScheduledRxTime = ScheduledRxTime_us / 1e6;
    ScheduledRxTime = ScheduledRxTime + startForNextScheduledTime;
    startForNextScheduledTime = startForNextScheduledTime + ComputeScheduledTime(Interval, Step, BatchLength,  TxBurstDelayNs / 1e3) / 1e6;
    if ~isempty(currentBatch.MVMExtra)
        FTMTime = currentBatch.MVMExtra.FTMClock(sortIndex) / 320e6;
        FTMTime = UniformUnwrap(FTMTime, 2^32 / 320e6);
    end
    
    if cfoTrackingByTxTiming
        timeSource = ScheduledRxTime;
        ClockWrappingLimit = inf;
    elseif useFTMClock
        timeSource = FTMTime;
        ClockWrappingLimit = 2^32 / 320e6;
    else
        timeSource = TimeStampTime;
        ClockWrappingLimit = 2^32 / 1e6;
    end
    
    if TrainingStage && ~skipTraining

        csi = currentBatch.CSI{1}(:, currentBatch.DCIndex);
        [cfo_by_ls, cfo_linear_residual] = CFOEstimationViaLinearFitting(Sequence, timeSource, csi);
        cfo_by_plomb = PlombBasedCFOEstimation(timeSource, angle(csi));
        initialCFO = cfo_by_plomb;

        [sfo_by_ls_1st, sfo_by_ls_2nd, sfo_residual_1st, sfo_residual_2nd] = SFOEstimationViaLinearFitting(Sequence, timeSource, slopes(:,1), wrappingLimit, false);
        sfo_plomb_final = PlombBasedSFOEstimation(timeSource, slopes(:,1), wrappingLimit, SamplingRate);
        initialSFO = sfo_plomb_final;

        initialCFOChangingRate = 0;
        trackingParam = KalmanTrackerParameters.getParametersKalman3D;
        trackingParam.InitialState = [initialCFOChangingRate, initialCFO, PhaseAtDC(1,1)]';
        trackingParam.ResetTracker = true;
        trackingParam.CarrierFrequency = CarrierFreq;
        trackingParam.LastRxTime = lastEndingTime;
        trackingParam.ClockWrappingLimit = ClockWrappingLimit;

        slopeTrackingParam = KalmanSlopeTrackerParameters.getParameters;
        slopeTrackingParam.InitialState = [0, initialSFO, slopes(1,1)]';
        slopeTrackingParam.ResetTracker = true;
        slopeTrackingParam.LastRxTime = lastEndingTime;
        slopeTrackingParam.SlopeWrappingLimit = wrappingLimit;
        slopeTrackingParam.ClockWrappingLimit = ClockWrappingLimit;

        skipTraining = true;
    end
% 
%     [cfo_by_ls, cfo_linear_residual] = CFOEstimationViaLinearFitting(Sequence, timeSource, csi);
%     [sfo_by_ls_1st, sfo_by_ls_2nd, sfo_residual_1st, sfo_residual_2nd] = SFOEstimationViaLinearFitting(Sequence, timeSource, slopes(:,1), wrappingLimit, false);

    groupResultsCFO = [];
    groupResultsSFO = [];
    for i = 1 : numel(timeSource)
        results = KalmanPhaseTracker4(timeSource(i), PhaseAtDC(i, :), trackingParam);
        groupResultsCFO = [groupResultsCFO results];
        trackingParam.ResetTracker = false;

        slopeResult = KalmanPhaseSlopeTracker(timeSource(i), slopes(i,:), slopeTrackingParam);
        groupResultsSFO = [groupResultsSFO slopeResult];
        slopeTrackingParam.ResetTracker = false;
    end
    ResultsCFO = [ResultsCFO groupResultsCFO];
    ResultsSFO = [ResultsSFO groupResultsSFO];


    if ~TrainingStage
        residualStd = std([ResultsCFO(end - size(ResultsCFO, 2) + 1 : end).ResidualPredicted]);
        if residualStd > 0.5
            skipTraining = false;
            lastEndingTime = ResultsCFO(end).RxTime;
        end
    end
end

%% Calibration

intercepts = -[ResultsCFO.WrappedCorrectedPhase]';
slopes = -[ResultsSFO.WrappedCorrectedSlope]';
corrected = slave_source.adjustPhase(slopes, intercepts);
slave_phase = cat(1, ResultsCFO.UnwrappedCorrectedPhase);
test_phase_pre = interp1(slave_clock, slave_phase, test_clock, "linear");


%% This is just plotting them, bro.
% figure;
% 
% subplot(1, 3, 1);
% plot(test_cfo);
% title("test-cfo");
% 
% subplot(1, 3, 2);
% plot(slave_cfo);
% title("slave-cfo");
% 
% % subplot(1, 3, 3)
% x1 = test_source.PhaseAtDC(:,1);
% % y = [ResultsCFO.CFO] / 1e3;
% y1 = test_cfo;
% z1 = x1 - y1;
% plot(wrapToPi(z));
% title("phase");

% All of above was interp only one nic for this phase arrays, and it can be
% used for more times.

%% Get every data
% test_source = phased_80_230414_155947;
% slave_source = phased_81_230414_155921;
test_ftm_clock = test_source.MVMExtra.MuClock;
slave_ftm_clock = slave_source.MVMExtra.MuClock;

%% Data source
batches = test_source.splitBundleByField('TxDPASRequest.Stage', 'bundle');

if median(batches(1).TxDPASRequest.Stage) ~= 0
    error('The first batch must be DPASRequest *Training* stage.');
end

%% Configurations

cfoTrackingByTxTiming = true;
% cfoTrackingByTxTiming = false;
useFTMClock = true;

%% Training & Tracking

skipTraining = false;
lastEndingTime = 0;
if ~isempty(batches(1).MVMExtra)
    lastEndingTime = batches(1).MVMExtra.FTMClock(1) / 320e6;
end
startForNextScheduledTime = 0;
SFOResults = [];
ResultsCFO = [];
ResultsSFO = [];
startTime = tic;
wrappingLimit = [];

for groupI = 1 : size(batches, 1)
    currentBatch = batches(groupI);
    currentBatch.doPhaseRegression;
    TrainingStage = median(currentBatch.TxDPASRequest.Stage) == 0;
    BatchLength = mean(currentBatch.TxDPASRequest.BatchLength);
    Interval = mean(currentBatch.TxDPASRequest.Interval);
    Step = mean(currentBatch.TxDPASRequest.Step);
    Sequence = currentBatch.TxDPASRequest.Sequence;
    CarrierFreq = mean(currentBatch.TxDPASRequest.CarrierFrequency);
    SamplingRate = mean(currentBatch.TxDPASRequest.SamplingRate);
    TxBurstDelayNs = mean(currentBatch.TxDPASRequest.TxDelayNs);
    [~, sortIndex] = sort(Sequence);
    PhaseAtDC = currentBatch.PhaseAtDC(sortIndex, :);
    slopes = currentBatch.PhaseSlope(sortIndex, :);
    wrappingLimit = decidePhaseSlopeWrappingLimit(slopes);

    TimeStampTime = double(currentBatch.Basic.Timestamp(sortIndex)) / 1e6;
    ScheduledRxTime_us = ComputeScheduledTime(Interval, Step, Sequence, TxBurstDelayNs / 1e3);
    ScheduledRxTime = ScheduledRxTime_us / 1e6;
    ScheduledRxTime = ScheduledRxTime + startForNextScheduledTime;
    startForNextScheduledTime = startForNextScheduledTime + ComputeScheduledTime(Interval, Step, BatchLength,  TxBurstDelayNs / 1e3) / 1e6;
    if ~isempty(currentBatch.MVMExtra)
        FTMTime = currentBatch.MVMExtra.FTMClock(sortIndex) / 320e6;
        FTMTime = UniformUnwrap(FTMTime, 2^32 / 320e6);
    end
    
    if cfoTrackingByTxTiming
        timeSource = ScheduledRxTime;
        ClockWrappingLimit = inf;
    elseif useFTMClock
        timeSource = FTMTime;
        ClockWrappingLimit = 2^32 / 320e6;
    else
        timeSource = TimeStampTime;
        ClockWrappingLimit = 2^32 / 1e6;
    end
    
    if TrainingStage && ~skipTraining

        csi = currentBatch.CSI{1}(:, currentBatch.DCIndex);
        [cfo_by_ls, cfo_linear_residual] = CFOEstimationViaLinearFitting(Sequence, timeSource, csi);
        cfo_by_plomb = PlombBasedCFOEstimation(timeSource, angle(csi));
        initialCFO = cfo_by_plomb;

        [sfo_by_ls_1st, sfo_by_ls_2nd, sfo_residual_1st, sfo_residual_2nd] = SFOEstimationViaLinearFitting(Sequence, timeSource, slopes(:,1), wrappingLimit, false);
        sfo_plomb_final = PlombBasedSFOEstimation(timeSource, slopes(:,1), wrappingLimit, SamplingRate);
        initialSFO = sfo_plomb_final;

        initialCFOChangingRate = 0;
        trackingParam = KalmanTrackerParameters.getParametersKalman3D;
        trackingParam.InitialState = [initialCFOChangingRate, initialCFO, PhaseAtDC(1,1)]';
        trackingParam.ResetTracker = true;
        trackingParam.CarrierFrequency = CarrierFreq;
        trackingParam.LastRxTime = lastEndingTime;
        trackingParam.ClockWrappingLimit = ClockWrappingLimit;

        slopeTrackingParam = KalmanSlopeTrackerParameters.getParameters;
        slopeTrackingParam.InitialState = [0, initialSFO, slopes(1,1)]';
        slopeTrackingParam.ResetTracker = true;
        slopeTrackingParam.LastRxTime = lastEndingTime;
        slopeTrackingParam.SlopeWrappingLimit = wrappingLimit;
        slopeTrackingParam.ClockWrappingLimit = ClockWrappingLimit;

        skipTraining = true;
    end
% 
%     [cfo_by_ls, cfo_linear_residual] = CFOEstimationViaLinearFitting(Sequence, timeSource, csi);
%     [sfo_by_ls_1st, sfo_by_ls_2nd, sfo_residual_1st, sfo_residual_2nd] = SFOEstimationViaLinearFitting(Sequence, timeSource, slopes(:,1), wrappingLimit, false);

    groupResultsCFO = [];
    groupResultsSFO = [];
    for i = 1 : numel(timeSource)
        results = KalmanPhaseTracker4(timeSource(i), PhaseAtDC(i, :), trackingParam);
        groupResultsCFO = [groupResultsCFO results];
        trackingParam.ResetTracker = false;

        slopeResult = KalmanPhaseSlopeTracker(timeSource(i), slopes(i,:), slopeTrackingParam);
        groupResultsSFO = [groupResultsSFO slopeResult];
        slopeTrackingParam.ResetTracker = false;
    end
    ResultsCFO = [ResultsCFO groupResultsCFO];
    ResultsSFO = [ResultsSFO groupResultsSFO];


    if ~TrainingStage
        residualStd = std([ResultsCFO(end - size(ResultsCFO, 2) + 1 : end).ResidualPredicted]);
        if residualStd > 0.5
            skipTraining = false;
            lastEndingTime = ResultsCFO(end).RxTime;
        end
    end
end

%% Calibration

intercepts = -[ResultsCFO.WrappedCorrectedPhase]';
% slopes = -[ResultsSFO.WrappedCorrectedSlope]';
% corrected = slave_source.adjustPhase(slopes, intercepts);
test_phase = cat(1, ResultsCFO.UnwrappedCorrectedPhase);
% slave_cfo = cat(1, ResultsCFO.UnwrappedCorrectedPhase);
% test_cfo = interp1(slave_ftm_clock, slave_cfo, test_ftm_clock, "linear");


%% This is just plotting them, bro.
figure;

% subplot(1, 3, 1);
% plot(test_cfo);
% title("test-cfo");
% 
% subplot(1, 3, 2);
% plot(slave_cfo);
% title("slave-cfo");

subplot(1, 2, 1)
plot(test_phase_pre);
title("phase_pre");

subplot(1, 2, 2)
plot(test_phase);
title("phase_true");
% x = test_source.PhaseAtDC(:,1);
% % y = [ResultsCFO.CFO] / 1e3;
% y = test_cfo;
% z = x - y;
% plot(wrapToPi(z));
% title("6_phase");


% All of above was interp only one nic for this phase arrays, and it can be
% used for more times.
















