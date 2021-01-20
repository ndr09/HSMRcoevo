/*
 * Copyright (C) 2019 Eric Medvet <eric.medvet@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
package it.units.erallab.hmsrobots;

import com.google.common.collect.Lists;
import it.units.erallab.hmsrobots.core.controllers.CentralizedMLP;
import it.units.erallab.hmsrobots.core.controllers.Controller;
import it.units.erallab.hmsrobots.core.objects.Robot;
import it.units.erallab.hmsrobots.core.objects.SensingVoxel;
import it.units.erallab.hmsrobots.core.sensors.*;
import it.units.erallab.hmsrobots.tasks.Locomotion;
import it.units.erallab.hmsrobots.tasks.Task;
import it.units.erallab.hmsrobots.util.Grid;
import it.units.erallab.hmsrobots.util.Util;
import it.units.erallab.hmsrobots.viewers.GridEpisodeRunner;
import it.units.erallab.hmsrobots.viewers.GridFileWriter;
import it.units.erallab.hmsrobots.viewers.GridOnlineViewer;
import it.units.erallab.hmsrobots.viewers.GridSnapshotListener;
import org.apache.commons.csv.CSVFormat;
import org.apache.commons.csv.CSVRecord;
import org.apache.commons.lang3.SerializationUtils;
import org.apache.commons.lang3.tuple.Pair;
import org.apache.commons.math3.random.EmpiricalDistribution;
import org.dyn4j.dynamics.Settings;

import java.io.*;
import java.util.*;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.function.Function;
import java.util.logging.Logger;

import static it.units.malelab.jgea.core.util.Args.*;

//import it.units.erallab.hmsrobots.util.MutualInformation;

/**
 * @author Eric Medvet <eric.medvet@gmail.com>
 * inputFile=serialized commonFilter=controller:centralizedMLP-0-sighted.0# xFilter=controller:centralizedMLP-0-sighted.0 yFilter=evolver:standard-2op,cma-es
 */
public class Video {

    private static final Logger L = Logger.getLogger(it.units.erallab.hmsrobots.Video.class.getName());
    private static double dT = 0d;
    private static String terrain = null;
    private static double finalT = 0d;

    public static void main(String[] args) throws IOException, ClassNotFoundException {
        //read main params
        String serializedColumnName = a(args, "data/all.txt", "best.description");
        String inputFile = a(args, "inputFile", "data/all.txt");
        String outputFile = a(args, "outputFile", "biped_sighted_1000_0_0.mp4");
        boolean online = b(a(args, "online", "false"));

        int w = i(a(args, "w", "640"));
        int h = i(a(args, "h", "480"));
        int frameRate = i(a(args, "frameRate", "30"));
        finalT = d(a(args, "finalT", "180"));
        dT = d(a(args, "dT", "0.0333"));
        terrain = a(args, "terrain", "hardcore");
        //read grid params
        //TODO: fix, doesn't work without filters
        String commonFilter = a(args, "commonFilter", "");
        String xFilters = a(args, "xFilter", "");
        String yFilters = a(args, "yFilter", "");
        String[] xFilterValues = xFilters.split(":")[1].split(",");
        String[] yFilterValues = yFilters.split(":")[1].split(",");
        String xFilterKey = xFilters.split(":")[0];
        String yFilterKey = yFilters.split(":")[0];
        Grid<Map<String, String>> filterGrid = Grid.create(xFilterValues.length, yFilterValues.length);
        for (int x = 0; x < xFilterValues.length; x++) {
            for (int y = 0; y < yFilterValues.length; y++) {
                String filter = commonFilter.replace("#", ";");
                filter = filter + ";" + xFilterKey + ":" + xFilterValues[x];
                filter = filter + ";" + yFilterKey + ":" + yFilterValues[y];
                filterGrid.set(x, y, filter(filter));
            }
        }
        filterGrid.values();
        
        //prepare problem
        //HashMap<Double, Integer> distribution = new HashMap<>();
        try {
            FileWriter myWriter = new FileWriter("worm_lim.txt");
            for (int i = 0; i < 10; i++) {
                inputFile = "data/sighted_1000/serialized."+i+".txt";
               // System.out.println("file "+i);
                fromCSV(
                        inputFile,
                        online,
                        outputFile,
                        w, h, frameRate,
                        serializedColumnName,
                        filterGrid,
                        s -> Util.lazilyDeserialize(s),
                        myWriter

                );
            }
            myWriter.close();
           // System.out.println("Successfully wrote to the file.");

        } catch (IOException e) {
            //System.out.println("An error occurred.");
            e.printStackTrace();
        }
        /*fromCSV(
                inputFile,
                online,
                outputFile,
                w, h, frameRate,
                serializedColumnName,
                filterGrid,
                s -> Util.lazilyDeserialize(s)
        );*/
        /*try {
            FileWriter myWriter = new FileWriter("data_3_sense_bv.txt");
            for (int i = 0; i < 10; i++) {
                inputFile = "data/3bl/serialized." + i + ".txt";
                System.out.println("file "+i);
                fromCSVSensor(
                        inputFile,
                        online,
                        outputFile,
                        w, h, frameRate,
                        serializedColumnName,
                        filterGrid,
                        s -> Util.lazilyDeserialize(s),
                        myWriter

                );
            }
            myWriter.close();
            System.out.println("Successfully wrote to the file.");

        } catch (IOException e) {
            System.out.println("An error occurred.");
            e.printStackTrace();
        }*/

    }

    private static <S> void fromCSVWegihts(String inputFile, boolean online, String outputFile, int w, int h, int frameRate,
                                           String serializedColumnName, Grid<Map<String, String>> filterGrid, Function<String, S> deserializer,
                                           FileWriter file
    ) throws IOException {//
        //read data
        Grid<Pair<String, S>> namedSolutionGrid = Grid.create(filterGrid);
        Reader in = new FileReader(inputFile);
        Boolean flag = true;
        for (CSVRecord record : CSVFormat.DEFAULT.withDelimiter(';').withFirstRecordAsHeader().parse(new FileReader(inputFile))) {
            String serialized = record.get(serializedColumnName);
            if (flag && serialized != null && record.get("controller").equals("centralizedMLP-0-Wsighted.0")) {//&& record.get("iterations").equals("100")
                System.out.println(record.get("controller"));
                flag = true;

                Robot robot = (Robot) deserializer.apply(serialized);
                for (double d : Arrays.stream(((CentralizedMLP) robot.getController()).getParams()).map(Math::abs).toArray()) {
                    file.write(d + ",");
                }
                file.write("\n");
            }
        }
    }
    private static <S> void fromCSVSensor(String inputFile, boolean online, String outputFile, int w, int h, int frameRate,
                                    String serializedColumnName, Grid<Map<String, String>> filterGrid, Function<String, S> deserializer,
                                          FileWriter file
    ) throws IOException {//
        //read data
        Grid<Pair<String, S>> namedSolutionGrid = Grid.create(filterGrid);
        Reader in = new FileReader(inputFile);
        Map<String, Integer> sensors = new HashMap<>();
        sensors.put("AreaRatio", -1);
        sensors.put("Velocity", -1);
        sensors.put("Lidar", -1);
        sensors.put("Touch", -1);
        Boolean flag = true;
        for (CSVRecord record : CSVFormat.DEFAULT.withDelimiter(';').withFirstRecordAsHeader().parse(new FileReader(inputFile))) {
            String serialized = record.get(serializedColumnName);
            Robot robot = (Robot) deserializer.apply(serialized);
            int c = 0;
            for (int i = 0; i < robot.getVoxels().getH(); i++) {
                for (int j = 0; j < robot.getVoxels().getW(); j++) {
                    if (robot.getVoxels().get(j, i) != null) {
                        List<Sensor> sensorsList = ((SensingVoxel) robot.getVoxels().get(j, i)).getSensors();
                        for (int l = 0; l < sensorsList.size(); l++) {
                            if (sensorsList.get(l) instanceof AreaRatio && sensors.get("AreaRatio") == -1) {
                                sensors.put("AreaRatio", (record.get("iterations")) == null ? -1 : Integer.parseInt(record.get("iterations")));
                            }
                            if (sensorsList.get(l) instanceof Velocity && sensors.get("Velocity") == -1) {
                                sensors.put("Velocity", (record.get("iterations")) == null ? -1 : Integer.parseInt(record.get("iterations")));
                            }
                            if (sensorsList.get(l) instanceof Lidar && sensors.get("Lidar") == -1) {
                                sensors.put("Lidar", (record.get("iterations")) == null ? -1 : Integer.parseInt(record.get("iterations")));
                            }
                            if (sensorsList.get(l) instanceof Average && sensors.get("Touch") == -1) {
                                sensors.put("Touch", (record.get("iterations")) == null ? -1 : Integer.parseInt(record.get("iterations")));
                            }
                        }
                        for (int l = 0; l < sensorsList.size(); l++) {
                            if (sensorsList.get(l) instanceof AreaRatio ) {
                                System.out.println("("+j+","+i+"): "+"A");
                                c++;
                            }
                            if (sensorsList.get(l) instanceof Velocity) {
                                System.out.println("("+j+","+i+"): "+"Vx");
                                c++;
                                System.out.println("("+j+","+i+"): "+"Vy");
                                c++;
                            }
                            if (sensorsList.get(l) instanceof Lidar) {
                                System.out.println("("+j+","+i+"): "+"L1");
                                c++;
                                System.out.println("("+j+","+i+"): "+"L2");
                                c++;
                                System.out.println("("+j+","+i+"): "+"L3");
                                c++;
                            }
                            if (sensorsList.get(l) instanceof Average ) {
                                System.out.println("("+j+","+i+"): "+"T");
                                c++;
                            }
                        }
                    }

                }
            }
            String sense = new String();
            for(String key : sensors.keySet()){
                sense += key+":"+sensors.get(key);
            }
            file.write(sense);
        }
    }
    private static <S> void fromCSV(String inputFile, boolean online, String outputFile, int w, int h, int frameRate,
                                    String serializedColumnName, Grid<Map<String, String>> filterGrid, Function<String, S> deserializer,
                                    FileWriter myWriter) throws IOException {//FileWriter file
        //read data
        Grid<Pair<String, S>> namedSolutionGrid = Grid.create(filterGrid);
        Reader in = new FileReader(inputFile);
        Map<String, Integer> sensors = new HashMap<>();
        sensors.put("AreaRatio", -1);
        sensors.put("Velocity", -1);
        sensors.put("Lidar", -1);
        sensors.put("Touch", -1);
        Boolean flag = true;
        for (CSVRecord record : CSVFormat.DEFAULT.withDelimiter(';').withFirstRecordAsHeader().parse(new FileReader(inputFile))) {
            String serialized = record.get(serializedColumnName);
            /*if (flag && serialized != null && record.get("controller").equals("centralizedMLP-0-sighted.0") ) {//&& record.get("iterations").equals("100")
                System.out.println(record.get("controller"));
                flag = true;

                Robot robot = (Robot) deserializer.apply(serialized);
                for (double d : Arrays.stream(((CentralizedMLP) robot.getController()).getParams()).map(Math::abs).toArray()) {
                    file.write(d + ",");
                }
                file.write("\n");
            }*/
            //}
            //}
                /*for (int i = 0; i < robot.getVoxels().getW(); i++) {
                    for (int j = 0; j < robot.getVoxels().getW(); j++) {
                        if (robot.getVoxels().get(i, j) != null) {
                            List<Sensor> sensorsList = ((SensingVoxel) robot.getVoxels().get(i, j)).getSensors();
                            for (int l = 0; l < sensorsList.size(); l++) {
                                if (sensorsList.get(l) instanceof AreaRatio && sensors.get("AreaRatio") == -1) {
                                    sensors.put("AreaRatio", (record.get("iterations")) == null? -1: Integer.parseInt(record.get("iterations")));
                                }
                                if (sensorsList.get(l) instanceof Velocity && sensors.get("Velocity") == -1) {
                                    sensors.put("Velocity", (record.get("iterations")) == null? -1: Integer.parseInt(record.get("iterations")));
                                }
                                if (sensorsList.get(l) instanceof Lidar && sensors.get("Lidar") == -1) {
                                    sensors.put("Lidar", (record.get("iterations")) == null? -1: Integer.parseInt(record.get("iterations")));
                                }
                                if (sensorsList.get(l) instanceof Average && sensors.get("Touch") == -1) {
                                    sensors.put("Touch", (record.get("iterations")) == null? -1: Integer.parseInt(record.get("iterations")));
                                }
                            }
                        }

                    }
                }*/
            for (Grid.Entry<Map<String, String>> gridEntry : filterGrid) {
                //check filter
                boolean met = true;
                for (Map.Entry<String, String> condition : gridEntry.getValue().entrySet()) {
                   // System.out.println(condition.getKey());
                    //System.out.println(condition.getValue());
                    //.out.println(record.get(condition.getKey()));
                    //System.out.println(condition.getValue().equals(record.get(condition.getKey())));
                    //System.out.println(record.get("shape"));
                    met = met && condition.getValue().equals(record.get(condition.getKey()));
                }
                //put or replace in grid

                if (met) {
                    //L.info(String.format("Found record meeting %s", gridEntry.getValue()));
                    namedSolutionGrid.set(
                            gridEntry.getX(),
                            gridEntry.getY(),
                            Pair.of(
                                    gridEntry.getValue().toString(),
                                    deserializer.apply(serialized)
                            ));
                }
            }
        }


        /*for (String key:sensors.keySet()){
            if (key=="Velocity") {
                System.out.println(key + "  " + sensors.get(key));
            }
        }*/
        for (Grid.Entry<Pair<String, S>> entry : namedSolutionGrid) {
            if (entry.getValue() == null) {
                throw new IllegalArgumentException(String.format(
                        "Cell in position (%d,%d) is null because of filter %s",
                        entry.getX(), entry.getY(),
                        filterGrid.get(entry.getX(), entry.getY())
                ));
            }
        }
        ScheduledExecutorService uiExecutor = Executors.newScheduledThreadPool(4);
        ExecutorService executor = online ? Executors.newCachedThreadPool() : Executors.newFixedThreadPool(Runtime.getRuntime().availableProcessors());
        GridSnapshotListener gridSnapshotListener = null;

        if (online) {
            gridSnapshotListener = new GridOnlineViewer(Grid.create(namedSolutionGrid, Pair::getLeft), uiExecutor);
            ((GridOnlineViewer) gridSnapshotListener).start(5);
        } else {
            gridSnapshotListener = new GridFileWriter(w, h, frameRate, new File(outputFile), Grid.create(namedSolutionGrid, Pair::getLeft), uiExecutor);
        }
        int[] leftPos = new int[]{0, 1, 4, 5, 8, 9, 15, 16};
        int[] rightPos = new int[]{2, 3, 6, 7, 13, 14, 20, 21};
        int[] centerPos = new int[]{10, 11, 12, 17, 18, 19};
        // works only for the first simulation


        Settings settings = new Settings();
        settings.setStepFrequency(dT);
        //System.out.println(namedSolutionGrid.get(0, 0).getValue());
        /*MutualInformation MI = new MutualInformation(((Robot) namedSolutionGrid.get(0, 0).getValue()).getVoxels(),
                leftPos, centerPos, rightPos, 1d, settings.getStepFrequency(), "sensorMiTS_H1");*/

        Locomotion locomotion = new Locomotion(
                180,
                //Locomotion.highSlopeTerrainUp(2000, 100, 0.5),
                Locomotion.createTerrain("hardcore", 14738),
                //Locomotion.stairsTerrainUP(2000d,100d, 4),
                Lists.newArrayList(Locomotion.Metric.values()),
                settings,
                null
        );

        for (Pair<String, S>  p : namedSolutionGrid.values()){

            double v = locomotion.apply((Robot<?>) p.getRight()).get(0);
            System.out.println(v);
        }

            executor.shutdownNow();
            uiExecutor.shutdownNow();

    }


    public static Map<String, String> filter(String string) {
        Map<String, String> map = new HashMap<>();
        for (String pair : string.split(";")) {
            if (!pair.trim().isEmpty()) {
                String[] pieces = pair.trim().split(":");
                if (pieces.length > 1) {
                    map.put(pieces[0].trim(), pieces[1].trim());
                }
            }
        }
        return map;
    }

    public static EmpiricalDistribution createDistribution(int bin) {
        if (bin <= 0) {
            bin = 1;
        }
        return new EmpiricalDistribution(bin);
    }

    public static double[] getHistogram(EmpiricalDistribution distribution) {
        double[] histogram = new double[distribution.getBinCount()];
        int k = 0;
        for (org.apache.commons.math3.stat.descriptive.SummaryStatistics stats : distribution.getBinStats()) {
            histogram[k++] = (double) stats.getN();
        }

        return histogram;
    }

}
