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
import it.units.malelab.jgea.core.evolver.stopcondition.Births;
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
import org.jcodec.common.ArrayUtil;

import javax.swing.table.TableStringConverter;
import java.io.FileNotFoundException;
import java.io.Serializable;
import java.util.*;
import java.util.concurrent.ExecutionException;
import java.util.function.Function;
import java.util.logging.Level;
import java.util.stream.Collectors;

import static it.units.malelab.jgea.core.util.Args.*;
import static java.lang.StrictMath.abs;


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
        List<String> mapperNames = l(a("mapper", "standard"));
        double finalT = d(a("finalT", "120"));
        double minDT = d(a("minDT", "0.0333"));
        double maxDT = d(a("maxDT", "0.0333"));
        List<Double> drivingFrequencies = d(l(a("drivingF", "-1")));
        List<Double> mutationSigmas = d(l(a("mutationSigma", "0.15")));
        int nPop = i(a("npop", "100"));
        int iterations = i(a("iterations", "100"));
        int cacheSize = i(a("cacheSize", "10000"));
        boolean statsToStandardOutput = b(a("stout", "true"));

        final double AREA_RATIO_MAX_DELTA = d(a("areaDeltaRatio", "0.7"));
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
        final double thresholdWorm = 1.9629758571599043; //2.87225975458687; biped
        final double thresholdBiped = 2.87225975458687;
        // prepare shapes
        Map<String, Grid<Boolean>> namedShapes = new LinkedHashMap<>();
        namedShapes.put("borg", createShape(new int[]{5, 5}));
        namedShapes.put("biped", createShape(new int[]{7, 4}, new int[]{2, 0, 5, 2}));
        namedShapes.put("worm", createShape(new int[]{7, 1}));
        namedShapes.put("worm1", createShape(new int[]{1, 1}));
        namedShapes.put("cross", createShapePoint(new int[]{5, 5}, new int[]{0, 0}, new int[]{1, 0}, new int[]{0, 1}, new int[]{3, 0}, new int[]{4, 0},
                new int[]{4, 1}, new int[]{0, 3}, new int[]{0, 4}, new int[]{1, 4}, new int[]{4, 3}, new int[]{3, 4}, new int[]{4, 4}));
        // prepare sensor configurations (will be later fixed by removing sensors where no voxels are)
        // centralized mlp
        Map<String, Function<Grid<Boolean>, Grid<SensingVoxel>>> namedSensorConfiguration = new LinkedHashMap<>();
        namedSensorConfiguration.put("low.0", (Function<Grid<Boolean>, Grid<SensingVoxel>>) (final Grid<Boolean> shape) -> Grid.create(shape.getW(), shape.getH(), (Integer x, Integer y) -> {
            List<Sensor> sensors = new ArrayList<>();
            if (y == 3 || y == 4) {
                sensors.add(new AreaRatio());

            }
            return new SensingVoxel(SIDE_LENGTH, MASS_SIDE_LENGTH_RATIO, SPRING_F, SPRING_D, MASS_LINEAR_DAMPING, MASS_ANGULAR_DAMPING, FRICTION, RESTITUTION, MASS, LIMIT_CONTRACTION_FLAG, MASS_COLLISION_FLAG, AREA_RATIO_MAX_DELTA, SPRING_SCAFFOLDINGS, 0, ControllableVoxel.ForceMethod.DISTANCE, sensors);
        }));
        namedSensorConfiguration.put("Wlow.0", (Function<Grid<Boolean>, Grid<SensingVoxel>>) (final Grid<Boolean> shape) -> Grid.create(shape.getW(), shape.getH(), (Integer x, Integer y) -> {
            List<Sensor> sensors = new ArrayList<>();
            if (x == 2 || x == 3 || x == 4) {
                sensors.add(new AreaRatio());
            }
            return new SensingVoxel(SIDE_LENGTH, MASS_SIDE_LENGTH_RATIO, SPRING_F, SPRING_D, MASS_LINEAR_DAMPING, MASS_ANGULAR_DAMPING, FRICTION, RESTITUTION, MASS, LIMIT_CONTRACTION_FLAG, MASS_COLLISION_FLAG, AREA_RATIO_MAX_DELTA, SPRING_SCAFFOLDINGS, 0, ControllableVoxel.ForceMethod.DISTANCE, sensors);
        }));
        namedSensorConfiguration.put("Clow.0", (Function<Grid<Boolean>, Grid<SensingVoxel>>) (final Grid<Boolean> shape) -> Grid.create(shape.getW(), shape.getH(), (Integer x, Integer y) -> {
            List<Sensor> sensors = new ArrayList<>();
            if (x == 2) {
                sensors.add(new AreaRatio());
            }
            if (y == 2 && x != 2) {
                sensors.add(new AreaRatio());
            }
            return new SensingVoxel(SIDE_LENGTH, MASS_SIDE_LENGTH_RATIO, SPRING_F, SPRING_D, MASS_LINEAR_DAMPING, MASS_ANGULAR_DAMPING, FRICTION, RESTITUTION, MASS, LIMIT_CONTRACTION_FLAG, MASS_COLLISION_FLAG, AREA_RATIO_MAX_DELTA, SPRING_SCAFFOLDINGS, 0, ControllableVoxel.ForceMethod.DISTANCE, sensors);
        }));

        namedSensorConfiguration.put("blind.0", (Function<Grid<Boolean>, Grid<SensingVoxel>>) (final Grid<Boolean> shape) -> Grid.create(shape.getW(), shape.getH(), (Integer x, Integer y) -> {
            List<Sensor> sensors = new ArrayList<>();
            if (y == 0) {
                sensors.add(new Average(new Touch(), 0.25d));
            }
            if (y == shape.getH() - 1) {
                sensors.add(new Velocity(true, 3d, Velocity.Axis.X, Velocity.Axis.Y));
            }
            sensors.add(new AreaRatio());
            return new SensingVoxel(SIDE_LENGTH, MASS_SIDE_LENGTH_RATIO, SPRING_F, SPRING_D, MASS_LINEAR_DAMPING, MASS_ANGULAR_DAMPING, FRICTION, RESTITUTION, MASS, LIMIT_CONTRACTION_FLAG, MASS_COLLISION_FLAG, AREA_RATIO_MAX_DELTA, SPRING_SCAFFOLDINGS, 0, ControllableVoxel.ForceMethod.DISTANCE, sensors);
        }));
        namedSensorConfiguration.put("Wblind.0", (Function<Grid<Boolean>, Grid<SensingVoxel>>) (final Grid<Boolean> shape) -> Grid.create(shape.getW(), shape.getH(), (Integer x, Integer y) -> {
            List<Sensor> sensors = new ArrayList<>();

            sensors.add(new Average(new Touch(), 0.25d));
            sensors.add(new Velocity(true, 3d, Velocity.Axis.X, Velocity.Axis.Y));
            sensors.add(new AreaRatio());
            return new SensingVoxel(SIDE_LENGTH, MASS_SIDE_LENGTH_RATIO, SPRING_F, SPRING_D, MASS_LINEAR_DAMPING, MASS_ANGULAR_DAMPING, FRICTION, RESTITUTION, MASS, LIMIT_CONTRACTION_FLAG, MASS_COLLISION_FLAG, AREA_RATIO_MAX_DELTA, SPRING_SCAFFOLDINGS, 0, ControllableVoxel.ForceMethod.DISTANCE, sensors);
        }));
        namedSensorConfiguration.put("Cblind.0", (Function<Grid<Boolean>, Grid<SensingVoxel>>) (final Grid<Boolean> shape) -> Grid.create(shape.getW(), shape.getH(), (Integer x, Integer y) -> {
            List<Sensor> sensors = new ArrayList<>();

            if ((x == 0 && y == 2) || (x == 2 && y == 0) || (x == 2 && y == 4) || (x == 4 && y == 2)) {
                sensors.add(new Average(new Touch(), 0.25d));

            }

            if (x == 2) {
                sensors.add(new Velocity(true, 3d, Velocity.Axis.X, Velocity.Axis.Y));
            }
            if (y == 2 && x != 2) {
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
                rayDir[1] = Math.toRadians(-15d);
                rayDir[2] = Math.toRadians(-30d);
                sensors.add(new Lidar(rayLength, rayDir));
            }
            sensors.add(new AreaRatio());
            return new SensingVoxel(SIDE_LENGTH, MASS_SIDE_LENGTH_RATIO, SPRING_F, SPRING_D, MASS_LINEAR_DAMPING, MASS_ANGULAR_DAMPING, FRICTION, RESTITUTION, MASS, LIMIT_CONTRACTION_FLAG, MASS_COLLISION_FLAG, AREA_RATIO_MAX_DELTA, SPRING_SCAFFOLDINGS, 0, ControllableVoxel.ForceMethod.DISTANCE, sensors);
        }));
        namedSensorConfiguration.put("Wsighted.0", (Function<Grid<Boolean>, Grid<SensingVoxel>>) (final Grid<Boolean> shape) -> Grid.create(shape.getW(), shape.getH(), (Integer x, Integer y) -> {
            List<Sensor> sensors = new ArrayList<>();
            Average av = new Average(new Touch(), 0.25d);
            sensors.add(av);


            sensors.add(new Velocity(true, 3d, Velocity.Axis.X, Velocity.Axis.Y));
            sensors.add(new AreaRatio());

            if (x == shape.getW() - 1) {
                double rayLength = shape.getW() * Voxel.SIDE_LENGTH;
                LinkedHashMap<Lidar.Side, Integer> raysPerSide = new LinkedHashMap<>() {{
                    put(Lidar.Side.E, 3);
                }};
                double[] rayDir = new double[3];
                rayDir[0] = 0d;
                rayDir[1] = Math.toRadians(-15d);
                rayDir[2] = Math.toRadians(-30d);
                sensors.add(new Lidar(rayLength, rayDir));
            }

            if (x == 0) {
                double rayLength = shape.getW() * Voxel.SIDE_LENGTH;
                LinkedHashMap<Lidar.Side, Integer> raysPerSide = new LinkedHashMap<>() {{
                    put(Lidar.Side.W, 3);
                }};
                double[] rayDir = new double[3];
                rayDir[0] = Math.toRadians(180d);
                rayDir[1] = Math.toRadians(-150d);
                rayDir[2] = Math.toRadians(-165d);
                sensors.add(new Lidar(rayLength, rayDir));
            }


            return new SensingVoxel(SIDE_LENGTH, MASS_SIDE_LENGTH_RATIO, SPRING_F, SPRING_D, MASS_LINEAR_DAMPING, MASS_ANGULAR_DAMPING, FRICTION, RESTITUTION, MASS, LIMIT_CONTRACTION_FLAG, MASS_COLLISION_FLAG, AREA_RATIO_MAX_DELTA, SPRING_SCAFFOLDINGS, 0, ControllableVoxel.ForceMethod.DISTANCE, sensors);
        }));

        namedSensorConfiguration.put("Csighted.0", (Function<Grid<Boolean>, Grid<SensingVoxel>>) (final Grid<Boolean> shape) -> Grid.create(shape.getW(), shape.getH(), (Integer x, Integer y) -> {
            List<Sensor> sensors = new ArrayList<>();

            sensors.add(new AreaRatio());

            if (x == 2 && y == 0) {
                double rayLength = shape.getW() * Voxel.SIDE_LENGTH;
                LinkedHashMap<Lidar.Side, Integer> raysPerSide = new LinkedHashMap<>() {{
                    put(Lidar.Side.E, 3);
                }};
                double[] rayDir = new double[3];
                rayDir[0] = Math.toRadians(-90d);
                rayDir[1] = Math.toRadians(-105);
                rayDir[2] = Math.toRadians(-120d);
                sensors.add(new Lidar(rayLength, rayDir));
                sensors.add(new Average(new Touch(), 0.25d));

            }
            if (x == 0 && y == 2) {
                double rayLength = shape.getW() * Voxel.SIDE_LENGTH;
                LinkedHashMap<Lidar.Side, Integer> raysPerSide = new LinkedHashMap<>() {{
                    put(Lidar.Side.E, 3);
                }};
                double[] rayDir = new double[3];
                rayDir[0] = Math.toRadians(180d);
                rayDir[1] = Math.toRadians(165d);
                rayDir[2] = Math.toRadians(150d);
                sensors.add(new Lidar(rayLength, rayDir));
                sensors.add(new Average(new Touch(), 0.25d));

            }
            if (x == 2 && y == 4) {
                double rayLength = shape.getW() * Voxel.SIDE_LENGTH;
                LinkedHashMap<Lidar.Side, Integer> raysPerSide = new LinkedHashMap<>() {{
                    put(Lidar.Side.E, 3);
                }};
                double[] rayDir = new double[3];
                rayDir[0] = Math.toRadians(90d);
                rayDir[1] = Math.toRadians(75);
                rayDir[2] = Math.toRadians(60d);
                sensors.add(new Lidar(rayLength, rayDir));
                sensors.add(new Average(new Touch(), 0.25d));

            }
            if (x == 4 && y == 2) {
                double rayLength = shape.getW() * Voxel.SIDE_LENGTH;
                LinkedHashMap<Lidar.Side, Integer> raysPerSide = new LinkedHashMap<>() {{
                    put(Lidar.Side.E, 3);
                }};
                double[] rayDir = new double[3];
                rayDir[0] = Math.toRadians(0d);
                rayDir[1] = Math.toRadians(-15d);
                rayDir[2] = Math.toRadians(-30d);
                sensors.add(new Lidar(rayLength, rayDir));
                sensors.add(new Average(new Touch(), 0.25d));

            }

            if (x == 2) {
                sensors.add(new Velocity(true, 3d, Velocity.Axis.X, Velocity.Axis.Y));
            }

            if (y == 2 && x != 2) {
                sensors.add(new Velocity(true, 3d, Velocity.Axis.X, Velocity.Axis.Y));
            }
            return new SensingVoxel(SIDE_LENGTH, MASS_SIDE_LENGTH_RATIO, SPRING_F, SPRING_D, MASS_LINEAR_DAMPING, MASS_ANGULAR_DAMPING, FRICTION, RESTITUTION, MASS, LIMIT_CONTRACTION_FLAG, MASS_COLLISION_FLAG, AREA_RATIO_MAX_DELTA, SPRING_SCAFFOLDINGS, 0, ControllableVoxel.ForceMethod.DISTANCE, sensors);
        }));

        namedSensorConfiguration.put("rotation.0", (Function<Grid<Boolean>, Grid<SensingVoxel>>) (final Grid<Boolean> shape) -> Grid.create(shape.getW(), shape.getH(), (Integer x, Integer y) -> {
            List<Sensor> sensors = new ArrayList<>();
            if (y == 0 || y == shape.getH() - 1 || x == 0 || x == shape.getW() - 1) {
                sensors.add(new Average(new Touch(), 0.25d));
            }
            if (y == 2 && x == 2) {
                sensors.add(new Angle());
            }
            sensors.add(new AreaRatio());
            return new SensingVoxel(SIDE_LENGTH, MASS_SIDE_LENGTH_RATIO, SPRING_F, SPRING_D, MASS_LINEAR_DAMPING, MASS_ANGULAR_DAMPING, FRICTION, RESTITUTION, MASS, LIMIT_CONTRACTION_FLAG, MASS_COLLISION_FLAG, AREA_RATIO_MAX_DELTA, SPRING_SCAFFOLDINGS, 0, ControllableVoxel.ForceMethod.DISTANCE, sensors);
        }));

        namedSensorConfiguration.put("all.0", (Function<Grid<Boolean>, Grid<SensingVoxel>>) (final Grid<Boolean> shape) -> Grid.create(shape.getW(), shape.getH(), (Integer x, Integer y) -> {
            List<Sensor> sensors = new ArrayList<>();

            sensors.add(new Average(new Touch(), 0.25d));

            Velocity v = new Velocity(true, 3d, Velocity.Axis.X, Velocity.Axis.Y);
            sensors.add(v);


            double rayLength = shape.getW() * Voxel.SIDE_LENGTH;
            LinkedHashMap<Lidar.Side, Integer> raysPerSide = new LinkedHashMap<>() {{
                put(Lidar.Side.E, 3);
            }};
            double[] rayDir = new double[3];
            rayDir[0] = Math.toRadians(0d);
            rayDir[1] = Math.toRadians(-15d);
            rayDir[2] = Math.toRadians(-30d);
            Lidar l = new Lidar(rayLength, rayDir);

            sensors.add(l);
            AreaRatio ar = new AreaRatio();
            sensors.add(ar);
            return new SensingVoxel(SIDE_LENGTH, MASS_SIDE_LENGTH_RATIO, SPRING_F, SPRING_D, MASS_LINEAR_DAMPING, MASS_ANGULAR_DAMPING, FRICTION, RESTITUTION, MASS, LIMIT_CONTRACTION_FLAG, MASS_COLLISION_FLAG, AREA_RATIO_MAX_DELTA, SPRING_SCAFFOLDINGS, 0, ControllableVoxel.ForceMethod.DISTANCE, sensors);
        }));
        namedSensorConfiguration.put("empty.0", (Function<Grid<Boolean>, Grid<SensingVoxel>>) (final Grid<Boolean> shape) -> Grid.create(shape.getW(), shape.getH(), (Integer x, Integer y) -> {
            List<Sensor> sensors = new ArrayList<>();
            return new SensingVoxel(SIDE_LENGTH, MASS_SIDE_LENGTH_RATIO, SPRING_F, SPRING_D, MASS_LINEAR_DAMPING, MASS_ANGULAR_DAMPING, FRICTION, RESTITUTION, MASS, LIMIT_CONTRACTION_FLAG, MASS_COLLISION_FLAG, AREA_RATIO_MAX_DELTA, SPRING_SCAFFOLDINGS, 0, ControllableVoxel.ForceMethod.DISTANCE, sensors);
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
                                    for (String mapName : mapperNames) {
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
                                        int nOfOutputs = (int) sensingVoxels.values().stream()
                                                .filter(Objects::nonNull).count();
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

                                        int VN = (int) sensingVoxels.values().stream().filter(Objects::nonNull).count();
                                        int VWL = (7 * VN + 9);
                                        int geneLenghts = VN * VWL + 2 * VN;
                                        //System.out.println("gene len"+geneLenghts);
                                        factory = new UniformDoubleSequenceFactory(-1d, 1d, geneLenghts);
                                        Random random = new Random(run);
                                        try {
                                            switch (mapName) {
                                                case "standard":
                                                    mapper = getCentralizedMLPMapper(sensingVoxels, drivingFrequency, innerNeurons);
                                                    break;
                                                case "threshold":
                                                    if(shapeName.equals("worm")) {
                                                        mapper = getCentralizedMLPMapperDynamicSensorCMAESFriendly(sensingVoxels, thresholdWorm, drivingFrequency, innerNeurons);
                                                    }else{
                                                        mapper = getCentralizedMLPMapperDynamicSensorCMAESFriendly(sensingVoxels, thresholdBiped, drivingFrequency, innerNeurons);

                                                    }
                                                    break;
                                                case "v":
                                                    if(shapeName.equals("worm")) {
                                                        mapper = getCentralizedMLPMapperDynamicSensorGI(sensingVoxels, 28, drivingFrequency, innerNeurons);
                                                    }else{
                                                        mapper = getCentralizedMLPMapperDynamicSensorGI(sensingVoxels, 40, drivingFrequency, innerNeurons);
                                                    }
                                                    break;
                                                default:
                                                    throw new IllegalArgumentException(mapName);

                                            }
                                        }catch (IllegalArgumentException ex){
                                            L.log(Level.SEVERE, String.format("Wrong mapper name: %s use standard, threshold or limited", ex), ex);
                                        }
                                        //mapper = getCentralizedMLPMapper(sensingVoxels, drivingFrequency, innerNeurons);
                                        //mapper = getCentralizedMLPStructureMapper(namedSensorConfiguration.get(sensorConfigurationName).apply(shape), drivingFrequency, innerNeurons, AREA_RATIO_MAX_DELTA, random);
                                        //mapper = getCentralizedMLPMapperDynamicSensorCMAESFriendly(sensingVoxels, threshold, drivingFrequency, innerNeurons);
                                        //mapper = getCentralizedMLPMapperDynamicSensorGI(sensingVoxels, 40, drivingFrequency, innerNeurons);
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
                                                    initMax,
                                                    VN

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
                                        double threshold = shapeName.equals("worm") ? thresholdWorm: thresholdBiped;
                                        // prepare collectors
                                        List<DataCollector> statsCollectors = Lists.newArrayList(
                                                new Static(keys),
                                                new Basic(),
                                                new Population(),
                                                new Diversity(),
                                                new FunctionOfOneBest<>(i -> List.of(new Item(
                                                        "fitness",
                                                        problem.getFitnessFunction().apply(SerializationUtils.clone((Robot) i.getSolution())),
                                                        "%5.3f"
                                                ))),
                                                new FunctionOfFiltered<>(i -> List.of(new Item(
                                                        "sensors.fitness",
                                                        i.stream().map(r -> problem.getFitnessFunction().apply(SerializationUtils.clone((Robot) r.getSolution()))).collect(Collectors.toCollection(ArrayList::new)),//.sum()/Double.parseDouble(Long.toString(i.stream().count())),
                                                        "%5.3f")),
                                                        l -> l.stream().filter(i -> (countOnDomain(((CentralizedMLP) (((Robot) i.getSolution()).getController())).getParams(), VN, threshold) > 0)).collect(Collectors.toCollection(ArrayList::new))),

                                                new FunctionOfFiltered<>(i -> List.of(new Item(
                                                        "noSensors.fitness",
                                                        i.stream().map(r -> problem.getFitnessFunction().apply(SerializationUtils.clone((Robot) r.getSolution()))).collect(Collectors.toCollection(ArrayList::new)),//.sum()/Double.parseDouble(Long.toString(i.stream().count())),
                                                        "%5.3f")),
                                                        l -> l.stream().filter(i -> (countOnDomain(((CentralizedMLP) (((Robot) i.getSolution()).getController())).getParams(), VN, threshold) == 0)).collect(Collectors.toCollection(ArrayList::new))),
                                                new FunctionOfOneBest<>(i -> List.of(new Item(
                                                        "fitness",
                                                        problem.getFitnessFunction().apply(SerializationUtils.clone((Robot) i.getSolution())),
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
                                        ArrayList<Item> tmp = new ArrayList<>();
                                        tmp.add(new Item("noSensor", "ttttt", "%s"));

                                        List<DataCollector> serializedCollectors = Lists.newArrayList(
                                                new Static(keys),
                                                new Basic(),
                                                //new BestInfo(problem.getFitnessFunction(Arrays.asList(Locomotion.Metric.values())).toString(), "%+5.3f"),
                                                new FunctionOfOneBest<>((individual) -> Collections.singletonList(new Item("description", Util.lazilySerialize(((Serializable) ((Individual) individual).getSolution())), "%s"))),
                                                new FunctionOfOneBestFiltered<>((individual) -> Collections.singletonList(new Item("noSensor", Util.lazilySerialize(((Serializable) ((Individual) individual).getSolution())), "%s")),
                                                        l -> l.stream().filter(i -> (countOnDomain(((CentralizedMLP) (((Robot) i.getSolution()).getController())).getParams(), VN, threshold) == 0)).collect(Collectors.toCollection(ArrayList::new)), "noSensor", "%s", tmp)

                                                //new FunctionOfOneBest((Individual individual) -> Collections.singletonList(new Item("description", Util.lazilySerialize((Serializable) individual.getSolution()), "%s")))
                                        );
                                        // run evolver

                                        Listener listener = statsListenerFactory.build(
                                                statsCollectors.toArray(new DataCollector[statsCollectors.size()])
                                        ).then(serializedBestListenerFactory.build(
                                                serializedCollectors.toArray(new DataCollector[serializedCollectors.size()])
                                        ));
                                    /*Listener listener = (serializedBestListenerFactory.build(
                                            serializedCollectors.toArray(new DataCollector[serializedCollectors.size()])
                                    )).then(statsListenerFactory.build(
                                            statsCollectors.toArray(new DataCollector[statsCollectors.size()])
                                    ));*/
                                        if (statsToStandardOutput) {
                                            listener = listener.then(listener(statsCollectors.toArray(new DataCollector[statsCollectors.size()])));
                                        }
                                        try {
                                            //Function<Robot, Double> fit = (pro) -> (l.get(0));
                                            evolver.solve(problem.getFitnessFunction(), new Births(25000), random, executorService, Listener.onExecutor(listener, executorService));
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
    }

    private Function<Sequence<Double>, Robot> getCentralizedMLPMapper(Grid<SensingVoxel> sensingVoxels, final double frequency, final int[] innerNeurons) {
        return (Sequence<Double> values) -> {
            //System.out.println(values.size());
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

    private Function<Sequence<Double>, Robot> getCentralizedMLPMapperDynamicSensorGI(Grid<SensingVoxel> sensingVoxels, final int maxActiveSensor, final double frequency, final int[] innerNeurons) {
        return (Sequence<Double> values) -> {
            String s = new String();
            for (Double d : values) {
                s += d + ",";
            }
            //System.out.println(s);
            double[] weights = new double[values.size()];
            TreeMap<Double, Integer> selector = new TreeMap<>();
            int VN = (int) sensingVoxels.values().stream().filter(Objects::nonNull).count();

            for (int i = 1; i < (int) (values.size() / VN) - 1; i++) {
                double max = 0d;
                for (int j = 0; j < VN; j++) {
                    if (i != 0) { //first input is the driving function
                        if (abs(values.get(i * VN + j)) > max) {
                            max = values.get(i * VN + j);
                        }
                    }
                }
                selector.put(max, i);
            }
            int valuesToRemove = (values.size() / VN) - 2 - maxActiveSensor;
            for (int i = 0; i < valuesToRemove; i++) {
                selector.remove(selector.lastEntry().getKey());
            }
            for (int i = 0; i < (int) (values.size() / VN); i++) {
                if (selector.containsValue(i) || i == 0 || i == (values.size() / VN) - 1) {
                    for (int j = 0; j < VN; j++) {
                        weights[i * VN + j] = values.get(i * VN + j);
                    }
                } else {
                    for (int j = 0; j < VN; j++) {
                        weights[i * VN + j] = 0d;
                    }
                }
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

    private Function<Sequence<Double>, Robot> getCentralizedMLPMapperDynamicSensorCMAESFriendly(Grid<SensingVoxel> sensingVoxels, final double threshold, final double frequency, final int[] innerNeurons) {
        return (Sequence<Double> values) -> {
            double[] weights = new double[values.size()];
            int VN = (int) sensingVoxels.values().stream().filter(Objects::nonNull).count();

            for (int i = 0; i < (int) (values.size() / VN); i++) {
                boolean isOn = false;
                for (int j = 0; j < VN; j++) {
                    if (i != 0) { //first input is the driving function
                        if (abs(values.get(i * VN + j)) > threshold) {
                            isOn = true;
                        }
                    }
                }

                for (int j = 0; j < VN; j++) {
                    if (isOn || i == 0) {
                        weights[i * VN + j] = values.get(i * VN + j);
                    } else { //turning off the i-th  domain
                        weights[i * VN + j] = 0d;
                    }
                }


            }

            Controller controller = new CentralizedMLP(
                    sensingVoxels,
                    innerNeurons,
                    weights,
                    t -> Math.sin(2d * Math.PI * frequency * t)
            );
            /*
            ((CentralizedSensing)controller).setFunction(new MultiLayerPerceptron(
                    MultiLayerPerceptron.ActivationFunction.TANH,
                    ((CentralizedSensing)controller).nOfInputs(),
                    innerNeurons,
                    ((CentralizedSensing)controller).nOfOutputs()
                    )
            );*/
            return new Robot(controller, SerializationUtils.clone(sensingVoxels));
        };
    }


    private Function<Sequence<Double>, Robot> getCentralizedMLPStructureMapper(Grid<SensingVoxel> sensingVoxels, final double frequency, final int[] innerNeurons, Double areaRatio, Random r) {
        return (Sequence<Double> values) -> {
            int k = 0;
            //System.out.println("aaaa");
            //System.out.println(values.stream().mapToDouble(i->i).sum()/values.size());
            int VN = (int) sensingVoxels.values().stream().filter(Objects::nonNull).count();
            Grid<SensingVoxel> vxls = new Grid<>(sensingVoxels.getW(), sensingVoxels.getH(), null);
            copyStructure(sensingVoxels, vxls);
            int VWL = (7 * VN + 9);
            //System.out.println("here "+ VN + "   "+values.size());
            ArrayList<Double> weights = new ArrayList<>();
            String test = "";
            for (int i = 0; i < VN; i++) {
                List<Sensor> sensors = new ArrayList<>();
                //System.out.println("--------- "+i);
                for (int C = 0; C < 4; C++) {
                    //System.out.println("here "+weights.size());
                    //for (int j = 0; j < VN + 1; j++) {
                    //i*(7*VN+10) start of i-th voxel
                    //
                    switch (C) {
                        case 0:
                            //System.out.println(values.get(0));
                            if (r.nextDouble() < RELU(values.get(i * VWL))) {
                                //copy 1 domain
                                //System.out.println("avg dom");
                                for (int l = i * VWL + 1; l < i * VWL + (VN + 1); l++) {
                                    weights.add(values.get(l));
                                }
                                test += "T";
                                //System.out.println("avg dom");
                                // i voxel has touch sensor
                                Average a = new Average(new Touch(), 0.25d);
                                //System.out.println("av "+a.domains().length);
                                sensors.add(a);
                                k += 1;
                            }
                            break;
                        case 1:
                            //System.out.println("C 1 "+ i+" "+VWL+"  "+VN+ "  "+values.size());
                            if (r.nextDouble() < RELU(values.get(i * VWL + (VN + 1)))) {
                                // copy 2 domain
                                //System.out.println("V dom");
                                for (int l = i * VWL + (VN + 2); l < i * VWL + (3 * VN + 2); l++) {
                                    //System.out.println(l);
                                    weights.add(values.get(l));
                                }
                                //System.out.println("V dom");
                                test += "V";
                                // i voxel has velocity sensor
                                Velocity v = new Velocity(true, 3d, Velocity.Axis.X, Velocity.Axis.Y);
                                //.println("ve "+v.domains().length);
                                sensors.add(v);
                                k += 2;
                            }
                            break;
                        case 2:
                            //System.out.println("C 2");
                            if (r.nextDouble() < RELU(values.get(i * VWL + (3 * VN + 2)))) {
                                // copy 1 domain
                                //System.out.println("A dom");
                                for (int l = i * VWL + (3 * VN + 3); l < i * VWL + (4 * VN + 3); l++) {
                                    weights.add(values.get(l));
                                }
                                //System.out.println("A dom");
                                // i voxel has area ratio sensor
                                test += "A";
                                //System.out.println("ar "+(new AreaRatio()).domains().length );
                                sensors.add(new AreaRatio());
                                k += 1;
                            }
                            break;
                        case 3:
                            //System.out.println("C 3");
                            ArrayList<Double> rayDir = new ArrayList<>();

                            if (r.nextDouble() < RELU(values.get(i * VWL + (4 * VN + 3)))) { //first ray of lidar
                                //System.out.println(values.get(i * VWL + (4 * VN + 3)));
                                rayDir.add(values.get(i * VWL + (4 * VN + 4)));
                                //System.out.println("1 dom");
                                for (int l = i * VWL + (4 * VN + 9); l < i * VWL + (5 * VN + 9); l++) {
                                    weights.add(values.get(l));
                                }
                                //System.out.println("1 dom");
                                //copy first domain
                            }
                            if (r.nextDouble() < RELU(values.get(i * VWL + (4 * VN + 5)))) { //second ray of lidar
                                //System.out.println(values.get(i * VWL + (4 * VN + 5)));
                                rayDir.add(values.get(i * VWL + (4 * VN + 6)));
                                //System.out.println("2 dom");
                                for (int l = i * VWL + (5 * VN + 9); l < i * VWL + (6 * VN + 9); l++) {
                                    weights.add(values.get(l));
                                }
                                //System.out.println("2 dom");
                                //copy second domain
                            }
                            if (r.nextDouble() < RELU(values.get(i * VWL + (4 * VN + 7)))) { //third ray of lidar
                                //System.out.println(values.get(i * VWL + (4 * VN + 7)));
                                rayDir.add(values.get(i * VWL + (4 * VN + 8)));
                                //System.out.println("3 dom");
                                for (int l = i * VWL + (6 * VN + 9); l < i * VWL + (7 * VN + 9); l++) {
                                    weights.add(values.get(l));
                                }
                                //System.out.println("3 dom");
                                //copy third domain
                            }
                            if (rayDir.size() > 0) {
                                double rayLength = sensingVoxels.getW() * Voxel.SIDE_LENGTH;
                                LinkedHashMap<Lidar.Side, Integer> raysPerSide = new LinkedHashMap<>() {{
                                    put(Lidar.Side.E, rayDir.size());
                                }};
                                double[] rayDirArray = rayDir.stream().mapToDouble(d -> 180 * d).toArray();
                                Lidar l = new Lidar(rayLength, rayDirArray);
                                sensors.add(l);
                                test += "L" + rayDir.size();
                                k += rayDir.size();
                            }
                    }
                    //}


                }
                //System.out.println("add");
                //TimeFunction domain is always present
                /*sensors.add(new TimeFunction(t -> Math.sin(2d * Math.PI * -1 * t), -1,1));
                for(int l = i * VWL + (7 * VN + 9); l < i * VWL + (8 * VN + 9); l++){
                    weights.add(values.get(l));
                }*/
                test += '-';
                vxls = addSensors(vxls, sensors, i, areaRatio);
            }
            //System.out.println("aaa");

            double[] filteredWeights = new double[weights.size() + 2 * VN];
            //System.out.println(filteredWeights.length);
            for (int i = 0; i < weights.size(); i++) {
                filteredWeights[i] = weights.get(i);
            }
            //System.out.println(test);
            //filteredWeights[filteredWeights.length - 2] = values.get(values.size() - 2);
            //filteredWeights[filteredWeights.length - 1] = values.get(values.size() - 1);
            for (int i = 0; i < 2 * VN; i++) {
                filteredWeights[i] = values.get((values.size() - 2 * VN) + i);
            }
            test = filteredWeights.length + "  " + k + "  " + test;

            Controller controller = new CentralizedMLP(
                    vxls,
                    innerNeurons,
                    filteredWeights,
                    t -> Math.sin(2d * Math.PI * frequency * t)
            );
            //Controller controller = new CentralizedSensing(vxls);
            /*MultiLayerPerceptron tt = new MultiLayerPerceptron(
                    MultiLayerPerceptron.ActivationFunction.TANH,
                    ((CentralizedSensing)controller).nOfInputs(),
                    innerNeurons,
                    ((CentralizedSensing)controller).nOfOutputs());
            System.out.println(test+ "     "+tt.getParams().length);*/
            /*((CentralizedSensing)controller).setFunction(new MultiLayerPerceptron(
                    MultiLayerPerceptron.ActivationFunction.TANH,
                    ((CentralizedSensing)controller).nOfInputs(),
                    innerNeurons,
                    ((CentralizedSensing)controller).nOfOutputs(),
                    filteredWeights
            ));*/

            return new Robot(controller, SerializationUtils.clone(vxls));
        };
    }

    private double RELU(double x) {
        return x > 0 ? x : 0;
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

    private Grid<Boolean> createShapePoint(int[] enclosing, int[]... holes) {
        Grid<Boolean> shape = Grid.create(enclosing[0], enclosing[1], true);
        for (int[] hole : holes) {
            shape.set(hole[0], hole[1], false);

        }
        return shape;
    }

    private void copyStructure(Grid<SensingVoxel> frm, Grid<SensingVoxel> to) {
        for (int i = 0; i < frm.getW(); i++) {
            for (int j = 0; j < frm.getW(); j++) {
                if (frm.get(i, j) != null) {
                    List<Sensor> sensors = new ArrayList<>();
                    to.set(i, j, new SensingVoxel(sensors));
                }

            }
        }


    }

    private Grid<SensingVoxel> addSensors(Grid<SensingVoxel> grid, List<Sensor> sensors, Integer pos, Double areaRatio) {
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
        int counter = 0;
        for (int i = 0; i < grid.getW(); i++) {
            for (int j = 0; j < grid.getW(); j++) {
                if (grid.get(i, j) != null) {
                    if (counter == pos) {

                        SensingVoxel vxl = new SensingVoxel(SIDE_LENGTH, MASS_SIDE_LENGTH_RATIO, SPRING_F, SPRING_D, MASS_LINEAR_DAMPING, MASS_ANGULAR_DAMPING, FRICTION, RESTITUTION, MASS, LIMIT_CONTRACTION_FLAG, MASS_COLLISION_FLAG, areaRatio, SPRING_SCAFFOLDINGS, 0, ControllableVoxel.ForceMethod.DISTANCE, sensors);

                        grid.set(i, j, vxl);
                        //System.out.println("sensor voxel set "+i+"  "+j+ "  "+sensors.size());
                    } else {
                        counter++;
                    }
                }
            }
        }
        return grid;

    }

    private int countOnDomain(double[] weights, int VN, double threshold) {
        int count = 0;
        for (int i = 1; i < (int) (weights.length / VN); i++) {
            boolean isOn = false;
            for (int j = 0; j < VN; j++) {
                if (abs(weights[i * VN + j]) >= threshold) {

                    isOn = true;
                }

            }
            if (isOn) {
                count += 1;
            }
        }
        return count;
    }
}
