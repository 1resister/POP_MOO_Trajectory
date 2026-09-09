function data = calculate_pareto_band_metrics(paretoSolutions,paretoData,cfg)
%CALCULATE_PARETO_BAND_METRICS Resonance-band energy for every Pareto point.
n=numel(paretoSolutions);
bandX=nan(n,1); bandY=nan(n,1); frequencyHz=nan(n,1);
for k=1:n
    if isempty(paretoSolutions{k}) || ~paretoSolutions{k}.feasible
        continue
    end
    metrics=calculate_frequency_metrics(paretoSolutions{k},cfg);
    frequencyHz(k)=metrics.mode(1).frequency;
    bandX(k)=metrics.mode(1).band_energy.modal_x;
    bandY(k)=metrics.mode(1).band_energy.modal_y;
end
bandTotal=bandX+bandY;
[~,referenceIndex]=min(abs(paretoData.LambdaVibration));
referenceEnergy=bandTotal(referenceIndex);
reduction=100*(referenceEnergy-bandTotal)/max(referenceEnergy,eps);
data=table(paretoData.LambdaVibration,paretoData.Jvibration, ...
    paretoData.Jstraightness,frequencyHz,bandX,bandY,bandTotal,reduction, ...
    paretoData.Feasible, ...
    'VariableNames',{'LambdaVibration','Jvibration','Jstraightness', ...
    'ResonanceFrequencyHz','BandEnergyX','BandEnergyY','BandEnergyTotal', ...
    'ReductionVsNoSuppression_percent','Feasible'});
end
