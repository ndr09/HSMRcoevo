package it.units.erallab.hmsrobots;


import com.google.common.collect.Lists;
import com.google.common.collect.Range;
import it.units.erallab.hmsrobots.core.controllers.CentralizedMLP;
import it.units.erallab.hmsrobots.core.controllers.Controller;
import it.units.erallab.hmsrobots.core.objects.ControllableVoxel;
import it.units.erallab.hmsrobots.core.objects.Robot;
import it.units.erallab.hmsrobots.core.objects.SensingVoxel;
import it.units.erallab.hmsrobots.core.objects.Voxel;
import it.units.erallab.hmsrobots.core.sensors.*;
import it.units.erallab.hmsrobots.tasks.Locomotion;
import it.units.erallab.hmsrobots.util.Grid;
import it.units.erallab.hmsrobots.util.Util;
import it.units.malelab.jgea.Worker;
import it.units.malelab.jgea.core.Individual;
import it.units.malelab.jgea.core.evolver.CMAESEvolver;
import it.units.malelab.jgea.core.evolver.Evolver;
import it.units.malelab.jgea.core.evolver.StandardEvolver;
import it.units.malelab.jgea.core.evolver.stopcondition.Iterations;
import it.units.malelab.jgea.core.listener.Listener;
import it.units.malelab.jgea.core.listener.MultiFileListenerFactory;
import it.units.malelab.jgea.core.listener.collector.*;
import it.units.malelab.jgea.core.operator.Crossover;
import it.units.malelab.jgea.core.operator.GeneticOperator;
import it.units.malelab.jgea.core.operator.Mutation;
import it.units.malelab.jgea.core.order.PartialComparator;
import it.units.malelab.jgea.core.selector.Tournament;
import it.units.malelab.jgea.core.selector.Worst;
import it.units.malelab.jgea.representation.sequence.Sequence;
import it.units.malelab.jgea.representation.sequence.numeric.GaussianMutation;
import it.units.malelab.jgea.representation.sequence.numeric.GeometricCrossover;
import it.units.malelab.jgea.representation.sequence.numeric.UniformDoubleSequenceFactory;
import org.apache.commons.lang3.SerializationUtils;

import java.io.FileNotFoundException;
import java.io.Serializable;
import java.util.*;
import java.util.concurrent.ExecutionException;
import java.util.function.Function;
import java.util.logging.Level;
import java.util.stream.Collectors;

import static it.units.malelab.jgea.core.util.Args.*;


public class Main extends Worker {

    public Main(String[] args) throws FileNotFoundException {
        super(args);
    }

    public static void main(String[] args) throws FileNotFoundException {
        new Main(args);
    }

    @Override
    public void run() {

        // read parameters
        int[] runs = ri(a("run", "0"));
        List<String> shapeNames = l(a("shape", "biped")); // worm,biped
        List<String> terrainNames = l(a("terrain", "hardcore")); // flat,hardcore
        List<String> evolverNames = l(a("evolver", "cma-es")); //standard-1|2op,cma-es
        List<String> controllerNames = l(a("controller", "centralizedMLP-0-sighted.0"));
        double finalT = d(a("finalT", "120"));
        double minDT = d(a("minDT", "0.0333"));
        double maxDT = d(a("maxDT", "0.0333"));
        List<Double> drivingFrequencies = d(l(a("drivingF", "-1")));
        List<Double> mutationSigmas = d(l(a("mutationSigma", "0.15")));
        int nPop = i(a("npop", "100"));
        int iterations = i(a("iterations", "100"));
        int cacheSize = i(a("cacheSize", "10000"));
        boolean statsToStandardOutput = b(a("stout", "true"));

        final double AREA_RATIO_MAX_DELTA = d(a("areaDeltaRatio", "0.4"));
        List<Locomotion.Metric> metrics = Lists.newArrayList(
                Locomotion.Metric.TRAVEL_X_VELOCITY


        );

        final double SIDE_LENGTH = 3d;
        final double MASS_SIDE_LENGTH_RATIO = .30d;
        final double SPRING_F = 8d;
        final double SPRING_D = 0.3d;
        final double MASS_LINEAR_DAMPING = 1d;
        final double MASS_ANGULAR_DAMPING = 1d;
        final double FRICTION = 100d;
        final double RESTITUTION = 0.1d;
        final double MASS = 1d;
        final boolean LIMIT_CONTRACTION_FLAG = true;
        final boolean MASS_COLLISION_FLAG = false;
        final EnumSet<Voxel.SpringScaffolding> SPRING_SCAFFOLDINGS = EnumSet.allOf(Voxel.SpringScaffolding.class);


        // prepare shapes
        Map<String, Grid<Boolean>> namedShapes = new LinkedHashMap<>();
        namedShapes.put("biped", createShape(new int[]{7, 4}, new int[]{2, 0, 5, 2}));
        // prepare sensor configurations (will be later fixed by removing sensors where no voxels are)
        // centralized mlp
        Map<String, Function<Grid<Boolean>, Grid<SensingVoxel>>> namedSensorConfiguration = new LinkedHashMap<>();
        namedSensorConfiguration.put("blind.0", (Function<Grid<Boolean>, Grid<SensingVoxel>>) (final Grid<Boolean> shape) -> Grid.create(shape.getW(), shape.getH(), (Integer x, Integer y) -> {
            List<Sensor> sensors = new ArrayList<>();
            if (y == 0) {
                sensors.add(new Touch());
            }
            if (y == shape.getH() - 1) {
                sensors.add(new Velocity(true, 3d, Velocity.Axis.X, Velocity.Axis.Y));
            }
            sensors.add(new AreaRatio());
                return new SensingVoxel(SIDE_LENGTH, MASS_SIDE_LENGTH_RATIO, SPRING_F, SPRING_D, MASS_LINEAR_DAMPING, MASS_ANGULAR_DAMPING, FRICTION, RESTITUTION, MASS, LIMIT_CONTRACTION_FLAG, MASS_COLLISION_FLAG, AREA_RATIO_MAX_DELTA, SPRING_SCAFFOLDINGS, 0, ControllableVoxel.ForceMethod.DISTANCE, sensors);
        }));
        namedSensorConfiguration.put("sighted.0", (Function<Grid<Boolean>, Grid<SensingVoxel>>) (final Grid<Boolean> shape) -> Grid.create(shape.getW(), shape.getH(), (Integer x, Integer y) -> {
            List<Sensor> sensors = new ArrayList<>();
            if (y == 0) {
                sensors.add(new Average(new Touch(), 0.25d));
            }
            if (y == shape.getH() - 1) {
                sensors.add(new Velocity(true, 3d, Velocity.Axis.X, Velocity.Axis.Y));
            }
            if (x == shape.getW() - 1) {
                double rayLength = shape.getW() * Voxel.SIDE_LENGTH;
                LinkedHashMap<Lidar.Side, Integer> raysPerSide = new LinkedHashMap<>() {{
                    put(Lidar.Side.E, 3);
                }};
                double[] rayDir = new double[3];
                rayDir[0] = 0d;
                rayDir[1] = 15d;
                rayDir[2] = 30d;
                sensors.add(new Lidar(rayLength, raysPerSide, rayDir));
            }
            sensors.add(new Average(new AreaRatio(), 0.5d));
            return new SensingVoxel(sensors);
        }));


        // prepare things
        MultiFileListenerFactory statsListenerFactory = new MultiFileListenerFactory(a("dir", "."), a("fileStats", "test"));
        MultiFileListenerFactory serializedBestListenerFactory = new MultiFileListenerFactory(a("dir", "."), a("fileSerialized", "stest"));
        // iterate
        for (int run : runs) {
            for (String shapeName : shapeNames) {
                for (String terrainName : terrainNames) {
                    for (String evolverName : evolverNames) {
                        for (String controllerName : controllerNames) {
                            for (double mutationSigma : mutationSigmas) {
                                for (double drivingFrequency : drivingFrequencies) {
                                    // prepare robot related things
                                    Grid<Boolean> shape = namedShapes.get(shapeName);
                                    // build problem
                                    LocomotionProblem problem = new LocomotionProblem(
                                            finalT, minDT, maxDT,
                                            new double[][][]{Locomotion.createTerrain(terrainName, 1),
                                                    Locomotion.createTerrain(terrainName, 2),
                                                    Locomotion.stairsTerrainUP(2000, 100, 4)},
                                            metrics
                                    );
                                    // prepare factory and mapper
                                    UniformDoubleSequenceFactory factory = null;
                                    Function<Sequence<Double>, Robot> mapper = null;
                                    String sensorConfigurationName = controllerName.split("-")[2];
                                    double innerLayerFactor = Double.parseDouble(controllerName.split("-")[1]);
                                    Grid<SensingVoxel> sensingVoxels = namedSensorConfiguration.get(sensorConfigurationName).apply(shape);
                                    for (Grid.Entry<Boolean> shapeEntry : shape) {
                                        if (!shapeEntry.getValue()) {
                                            sensingVoxels.set(shapeEntry.getX(), shapeEntry.getY(), null);
                                        }
                                    }
                                    int nOfInputs = (int) sensingVoxels.values().stream()
                                            .filter(Objects::nonNull)
                                            .mapToInt(v -> v.getSensors().stream()
                                                    .mapToInt(s -> s.domains().length)
                                                    .sum())
                                            .sum();
                                    int[] innerNeurons;
                                    if (innerLayerFactor == 0d) {
                                        innerNeurons = new int[0];
                                    } else {
                                        innerNeurons = new int[]{(int) Math.round((double) nOfInputs * innerLayerFactor)};
                                    }
                                    Controller controller = new CentralizedMLP(
                                            sensingVoxels,
                                            innerNeurons,
                                            t -> Math.sin(2d * Math.PI * drivingFrequency * t)
                                    );
                                    double[] weights = ((CentralizedMLP) controller).getParams();
                                    int params = weights.length;
                                    factory = new UniformDoubleSequenceFactory(-1d, 1d, params);
                                    mapper = getCentralizedMLPMapper(sensingVoxels, drivingFrequency, innerNeurons);

                                    // prepare evolver
                                    Evolver<Sequence<Double>, Robot, Double> evolver = null;
                                    if (evolverName.startsWith("standard")) {
                                        Crossover<Sequence<Double>> crossover = new GeometricCrossover(Range.closedOpen(-1d, 2d));
                                        Mutation<Sequence<Double>> mutation = new GaussianMutation(mutationSigma);
                                        Map<GeneticOperator<Sequence<Double>>, Double> operators = new LinkedHashMap<>();
                                        if (evolverName.split("-")[1].equals("1op")) {
                                            operators.put(crossover.andThen(mutation), 1d);
                                        } else if (evolverName.split("-")[1].equals("2op")) {
                                            operators.put(crossover, 0.8d);
                                            operators.put(mutation, 0.2d);
                                        }
                                        evolver = new StandardEvolver<>(
                                                mapper,
                                                factory,
                                                PartialComparator.from(Double.class).on(Individual::getFitness),
                                                nPop,
                                                operators,
                                                new Tournament(Math.max(Math.round(nPop / 30), 2)),
                                                new Worst(),
                                                nPop,
                                                true);
                                    } else if (evolverName.startsWith("cma-es")) {
                                        int size = params;
                                        double initMin = -1d;
                                        double initMax = 1d;
                                        evolver = new CMAESEvolver<>(
                                                mapper,
                                                factory,
                                                PartialComparator.from(Double.class).on(Individual::getFitness),
                                                size,
                                                initMin,
                                                initMax

                                        );
                                    }
                                    // prepare keys
                                    Map<String, String> keys = new LinkedHashMap<>();
                                    keys.put("evolver", evolverName);
                                    keys.put("controller", controllerName);
                                    keys.put("run", Integer.toString(run));
                                    keys.put("n.pop", Integer.toString(nPop));
                                    keys.put("driving.frequency", Double.toString(drivingFrequency));
                                    keys.put("mutation.sigma", Double.toString(mutationSigma));
                                    keys.put("shape", shapeName);
                                    keys.put("terrain", terrainName);
                                    keys.put("metrics", metrics.stream().map((m) -> m.toString().toLowerCase().replace("_", ".")).collect(Collectors.joining("/")));
                                    L.info(String.format("Keys: %s", keys));
                                    // prepare collectors
                                    List<DataCollector> statsCollectors = Lists.newArrayList(
                                            new Static(keys),
                                            new Basic(),
                                            new Population(),
                                            new Diversity(),
                                            new FunctionOfOneBest<>(i -> List.of(new Item(
                                                    "fitness",
                                                    problem.getFitnessFunction(),
                                                    "%5.3f"
                                            ))),
                                            new FunctionOfOneBest<>(i -> List.of(new Item(
                                                    "main.fitness",
                                                    problem.getValidationFunction(Arrays.asList(Locomotion.Metric.values()), 0).apply(SerializationUtils.clone((Robot) i.getSolution())),
                                                    "%5.3f"
                                            ))),
                                            new FunctionOfOneBest<>(i -> List.of(new Item(
                                                    "validation.fitness",
                                                    problem.getValidationFunction(Arrays.asList(Locomotion.Metric.values()), 1).apply(SerializationUtils.clone((Robot) i.getSolution())),
                                                    "%5.3f"
                                            ))),
                                            new FunctionOfOneBest<>(i -> List.of(new Item(
                                                    "flat.fitness",
                                                    problem.getValidationFunction(Arrays.asList(Locomotion.Metric.values()), 2).apply(SerializationUtils.clone((Robot) i.getSolution())),
                                                    "%5.3f"
                                            )))
                                    );

                                    List<DataCollector> serializedCollectors = Lists.newArrayList(
                                            new Static(keys),
                                            new Basic(),
                                            new BestInfo(problem.getFitnessFunction(Arrays.asList(Locomotion.Metric.values())).toString(), "%+5.3f"),
                                            new FunctionOfOneBest<>((individual) -> Collections.singletonList(new Item("description", Util.lazilySerialize((Serializable) ((Individual) individual).getSolution()), "%s")))
                                            //new FunctionOfOneBest((Individual individual) -> Collections.singletonList(new Item("description", Util.lazilySerialize((Serializable) individual.getSolution()), "%s")))
                                    );
                                    // run evolver
                                    Random r = new Random(run);
                                    Listener listener = statsListenerFactory.build(
                                            statsCollectors.toArray(new DataCollector[statsCollectors.size()])
                                    ).then(serializedBestListenerFactory.build(
                                            serializedCollectors.toArray(new DataCollector[serializedCollectors.size()])
                                    ));
                                    if (statsToStandardOutput) {
                                        listener = listener.then(listener(statsCollectors.toArray(new DataCollector[statsCollectors.size()])));
                                    }
                                    try {
                                        //Function<Robot, Double> fit = (pro) -> (l.get(0));
                                        evolver.solve(problem.getFitnessFunction(), new Iterations(iterations), r, executorService, Listener.onExecutor(listener, executorService));
                                    } catch (InterruptedException | ExecutionException ex) {
                                        L.log(Level.SEVERE, String.format("Cannot solve problem: %s", ex), ex);
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    private Function<Sequence<Double>, Robot> getCentralizedMLPMapper(Grid<SensingVoxel> sensingVoxels, final double frequency, final int[] innerNeurons) {
        return (Sequence<Double> values) -> {
            double[] weights = new double[values.size()];
            for (int i = 0; i < values.size(); i++) {
                weights[i] = values.get(i);
            }
            Controller controller = new CentralizedMLP(
                    sensingVoxels,
                    innerNeurons,
                    weights,
                    t -> Math.sin(2d * Math.PI * frequency * t)
            );
            return new Robot(controller, SerializationUtils.clone(sensingVoxels));
        };
    }

    private Grid<Boolean> createShape(int[] enclosing, int[]... holes) {
        Grid<Boolean> shape = Grid.create(enclosing[0], enclosing[1], true);
        for (int[] hole : holes) {
            for (int x = hole[0]; x < hole[2]; x++) {
                for (int y = hole[1]; y < hole[3]; y++) {
                    shape.set(x, y, false);
                }
            }
        }
        return shape;
    }

}
