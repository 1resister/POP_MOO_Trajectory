function [solutions,data,selection] = build_time_vibration_pareto(pairs,sweepData)
%BUILD_TIME_VIBRATION_PARETO Build the machining-time/vibration Pareto set.
% Every candidate is the minimum-vibration solution at one fixed integer N.

if isempty(pairs) || isempty(sweepData)
    error('POP_MOO:EmptyTimePareto', ...
        'The time-vibration Pareto front requires a completed time sweep.');
end
if numel(pairs)~=height(sweepData)
    error('POP_MOO:TimeParetoSizeMismatch', ...
        'The time-sweep pairs and summary table must have the same length.');
end

n=numel(pairs);
solutions=cell(n,1);
for index=1:n
    if isempty(pairs{index}) || ~isfield(pairs{index},'suppression')
        error('POP_MOO:InvalidTimeParetoPair', ...
            'Time-sweep pair %d has no suppression solution.',index);
    end
    solutions{index}=pairs{index}.suppression;
end

data=sweepData;
data.Jvibration=data.SuppressedJvibration;
data.Feasible=data.NoSuppressionFeasible & data.SuppressionFeasible;
data.ParetoOptimal=false(n,1);
valid=find(data.Feasible & isfinite(data.Time_s) & isfinite(data.Jvibration));
if isempty(valid)
    error('POP_MOO:NoTimeParetoSolution', ...
        'No feasible time-vibration candidates are available.');
end

for index=valid(:).'
    noWorse=data.Time_s(valid)<=data.Time_s(index)+1e-12 & ...
        data.Jvibration(valid)<=data.Jvibration(index)+1e-12;
    strictlyBetter=data.Time_s(valid)<data.Time_s(index)-1e-12 | ...
        data.Jvibration(valid)<data.Jvibration(index)-1e-12;
    data.ParetoOptimal(index)=~any(noWorse & strictlyBetter);
end

front=find(data.ParetoOptimal);
[~,localMinimumTime]=min(data.Time_s(front));
minimumTimeIndex=front(localMinimumTime);
[~,localMinimumVibration]=min(data.Jvibration(front));
minimumVibrationIndex=front(localMinimumVibration);

timeValues=data.Time_s(front);
vibrationValues=data.Jvibration(front);
timeRange=max(timeValues)-min(timeValues);
vibrationRange=max(vibrationValues)-min(vibrationValues);
normalizedTime=(timeValues-min(timeValues))/max(timeRange,eps);
normalizedVibration=(vibrationValues-min(vibrationValues))/max(vibrationRange,eps);
utopiaDistance=sqrt(normalizedTime.^2+normalizedVibration.^2);

interior=front(front~=minimumTimeIndex & front~=minimumVibrationIndex);
if isempty(interior)
    candidates=front;
else
    candidates=interior;
end
[~,localKnee]=min(utopiaDistance(ismember(front,candidates)));
kneeIndex=candidates(localKnee);

data.NormalizedTime=nan(n,1);
data.NormalizedVibration=nan(n,1);
data.UtopiaDistance=nan(n,1);
data.NormalizedTime(front)=normalizedTime;
data.NormalizedVibration(front)=normalizedVibration;
data.UtopiaDistance(front)=utopiaDistance;
data.IsMinimumTime=false(n,1);
data.IsKnee=false(n,1);
data.IsMinimumVibration=false(n,1);
data.IsMinimumTime(minimumTimeIndex)=true;
data.IsKnee(kneeIndex)=true;
data.IsMinimumVibration(minimumVibrationIndex)=true;

selection=struct('minimum_time_index',minimumTimeIndex, ...
    'knee_index',kneeIndex, ...
    'minimum_vibration_index',minimumVibrationIndex, ...
    'front_indices',front, ...
    'knee_normalized_utopia_distance',data.UtopiaDistance(kneeIndex));
end
