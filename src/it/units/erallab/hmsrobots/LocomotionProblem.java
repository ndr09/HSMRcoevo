package it.units.erallab.hmsrobots;

import it.units.erallab.hmsrobots.core.objects.Robot;
import it.units.erallab.hmsrobots.tasks.Locomotion;
import it.units.malelab.jgea.core.Problem;
import org.dyn4j.dynamics.Settings;

import java.util.List;
import java.util.function.Function;

public class LocomotionProblem  implements Problem<Robot, Double> {

    private final double maxFinalT;
    private final double minDT;
    private final double maxDT;
    private final double[][][] groundProfiles;
    private final List<Locomotion.Metric> metrics;

    public LocomotionProblem(double maxFinalT, double minDT, double maxDT, double[][][] groundProfiles, List<Locomotion.Metric> metrics) {
        this.maxFinalT = maxFinalT;
        this.minDT = minDT;
        this.maxDT = maxDT;
        this.groundProfiles = groundProfiles;
        this.metrics = metrics;
    }

    public Function<Robot,  Double> getFitnessFunction() {
        return (Function<Robot,  Double>) getFitnessFunction(metrics);
    }

    public Function<Robot,  List<Double>> getValidationFunction(List<Locomotion.Metric> localMetrics, int i) {
        return (Robot robot) -> {

            double dT = minDT;
            double finalT = maxFinalT;
            finalT = maxFinalT * (1d);

            Settings settings = new Settings();
            settings.setStepFrequency(dT);

            Locomotion locomotion = new Locomotion(finalT, groundProfiles[i], localMetrics, settings, null);
            List<Double> metricValues = locomotion.apply(robot, null);
            for (int j = 0; j < metricValues.size(); j++) {
                metricValues.set(j, metricValues.get(j) * (localMetrics.get(j).isToMinimize() ? 1d : (-1d)));

            }
            return metricValues;
        };
    }
    public Function<Robot,  Double> getFitnessFunction(List<Locomotion.Metric> localMetrics) {
        Function<Robot, Double> l = (Robot robot) -> {

            double dT = minDT;
            double finalT = maxFinalT;
            finalT = maxFinalT * (1d);

            Settings settings = new Settings();
            settings.setStepFrequency(dT);

            Locomotion locomotion = new Locomotion(finalT, groundProfiles[0], localMetrics, settings,null);
            List<Double> metricValues = locomotion.apply(robot, null);

            for (int j = 0; j < metricValues.size(); j++) {

                metricValues.set(j, metricValues.get(j) * (localMetrics.get(j).isToMinimize() ? 1d : (-1d)));

            }
            return metricValues.get(0);
        };

        return (Robot robot) -> {
            return l.apply(robot);
        };
    }

}
