function data = calculate_pareto_band_metrics(paretoSolutions,paretoData,cfg)
%CALCULATE_PARETO_BAND_METRICS Paired band energy along the time Pareto set.
n=numel(paretoSolutions);
bandX=nan(n,1);
bandY=nan(n,1);
frequencyHz=nan(n,1);
for index=1:n
    if isempty(paretoSolutions{index}) || ~paretoSolutions{index}.feasible
        continue
    end
    metrics=calculate_frequency_metrics(paretoSolutions{index},cfg);
    frequencyHz(index)=metrics.mode(1).frequency;
    bandX(index)=metrics.mode(1).band_energy.modal_x;
    bandY(index)=metrics.mode(1).band_energy.modal_y;
end
bandTotal=bandX+bandY;
referenceEnergy=paretoData.NoSuppressionBandEnergy;
reduction=100*(referenceEnergy-bandTotal)./max(referenceEnergy,eps);
data=table(paretoData.N,paretoData.Time_s,paretoData.Jvibration, ...
    frequencyHz,bandX,bandY,bandTotal,referenceEnergy,reduction, ...
    paretoData.ParetoOptimal,paretoData.Feasible, ...
    'VariableNames',{'N','Time_s','Jvibration','ResonanceFrequencyHz', ...
    'BandEnergyX','BandEnergyY','BandEnergyTotal', ...
    'PairedNoSuppressionBandEnergy','ReductionVsPairedNoSuppression_percent', ...
    'ParetoOptimal','Feasible'});
end
