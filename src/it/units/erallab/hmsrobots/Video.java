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
import it.units.erallab.hmsrobots.core.objects.Robot;
import it.units.erallab.hmsrobots.tasks.Locomotion;
import it.units.erallab.hmsrobots.tasks.Task;
import it.units.erallab.hmsrobots.util.Grid;
import it.units.erallab.hmsrobots.util.MutualInformation;
import it.units.erallab.hmsrobots.util.Util;
import it.units.erallab.hmsrobots.viewers.GridEpisodeRunner;
import it.units.erallab.hmsrobots.viewers.GridFileWriter;
import it.units.erallab.hmsrobots.viewers.GridOnlineViewer;
import it.units.erallab.hmsrobots.viewers.GridSnapshotListener;
import org.apache.commons.csv.CSVFormat;
import org.apache.commons.csv.CSVRecord;
import org.apache.commons.lang3.tuple.Pair;
import org.dyn4j.dynamics.Settings;

import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.io.Reader;
import java.util.HashMap;
import java.util.Map;
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
        String serializedColumnName = a(args, "data/stest", "best.description");
        String inputFile = a(args, "inputFile", "data/stest");
        String outputFile = a(args, "outputFile", "video/stest.mp4");
        boolean online = b(a(args, "online", "false"));

        int w = i(a(args, "w", "800"));
        int h = i(a(args, "h", "600"));
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

        fromCSV(
                inputFile,
                online,
                outputFile,
                w, h, frameRate,
                serializedColumnName,
                filterGrid,
                s -> Util.lazilyDeserialize(s)
        );
    }

    private static <S> void fromCSV(String inputFile, boolean online, String outputFile, int w, int h, int frameRate, String serializedColumnName, Grid<Map<String, String>> filterGrid, Function<String, S> deserializer) throws IOException {
        //read data
        Grid<Pair<String, S>> namedSolutionGrid = Grid.create(filterGrid);
        Reader in = new FileReader(inputFile);
        for (CSVRecord record : CSVFormat.DEFAULT.withDelimiter(';').withFirstRecordAsHeader().parse(new FileReader(inputFile))) {
            String serialized = record.get(serializedColumnName);
            if (serialized != null) {
                for (Grid.Entry<Map<String, String>> gridEntry : filterGrid) {
                    //check filter
                    boolean met = true;
                    for (Map.Entry<String, String> condition : gridEntry.getValue().entrySet()) {
                        System.out.println(condition.getValue());
                        System.out.println(condition.getKey());
                        System.out.println(record.get("shape"));
                        met = met && condition.getValue().equals(record.get(condition.getKey()));
                    }
                    //put or replace in grid
                    if (met) {
                        L.info(String.format("Found record meeting %s", gridEntry.getValue()));
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
        }
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
        System.out.println(namedSolutionGrid.get(0, 0).getValue());
        MutualInformation MI = new MutualInformation(((Robot) namedSolutionGrid.get(0, 0).getValue()).getVoxels(),
                leftPos, centerPos, rightPos, 1d, settings.getStepFrequency(), "sensorMiTS_H1");

        Locomotion locomotion = new Locomotion(
                finalT,
                Locomotion.highSlopeTerrainUp(2000, 100, 0.5),
                //Locomotion.createTerrain("hardcore", 1),
                //Locomotion.stairsTerrainUP(2000d,100d, 4),
                Lists.newArrayList(Locomotion.Metric.values()),
                settings,
                null
                );


        GridEpisodeRunner<S> runner = new GridEpisodeRunner<>(
                namedSolutionGrid, (Task<S, ?>) locomotion,
                gridSnapshotListener,
                executor
        );
        runner.run();
        if (!online) {
            executor.shutdownNow();
            uiExecutor.shutdownNow();
        }
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

}
