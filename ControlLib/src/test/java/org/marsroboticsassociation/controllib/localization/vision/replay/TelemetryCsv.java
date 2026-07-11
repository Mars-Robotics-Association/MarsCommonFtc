package org.marsroboticsassociation.controllib.localization.vision.replay;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * A header-indexed reader for the bank-localizer telemetry CSVs. It keys every value by <i>column
 * name</i> (never position) and yields {@link Double#NaN} for a column absent in an older file, so
 * the replay harness tolerates schema drift across runs.
 *
 * <p>Reads both plain {@code .csv} and brotli-compressed {@code .csv.br} recordings transparently: a
 * {@code .br} path is streamed through the {@code brotli} CLI (override with {@code -Dbrotli.bin=}),
 * so a recording compressed for storage still replays. Plain files need no external tool.
 */
public final class TelemetryCsv {

    private final List<String> columns;
    private final List<double[]> rows;
    private final Map<String, Integer> index = new HashMap<>();

    private TelemetryCsv(List<String> columns, List<double[]> rows) {
        this.columns = columns;
        this.rows = rows;
        for (int i = 0; i < columns.size(); i++) {
            index.put(columns.get(i), i);
        }
    }

    public static TelemetryCsv read(Path path) throws IOException {
        try (BufferedReader r = open(path)) {
            String headerLine = r.readLine();
            if (headerLine == null) {
                throw new IOException("empty CSV: " + path);
            }
            List<String> cols = new ArrayList<>();
            for (String c : headerLine.split(",", -1)) {
                cols.add(c.trim());
            }
            List<double[]> data = new ArrayList<>();
            String line;
            while ((line = r.readLine()) != null) {
                if (line.isEmpty()) {
                    continue;
                }
                String[] parts = line.split(",", -1);
                double[] vals = new double[cols.size()];
                for (int i = 0; i < cols.size(); i++) {
                    vals[i] = (i < parts.length) ? parseDouble(parts[i]) : Double.NaN;
                }
                data.add(vals);
            }
            return new TelemetryCsv(cols, data);
        }
    }

    private static BufferedReader open(Path path) throws IOException {
        if (!path.getFileName().toString().endsWith(".br")) {
            return Files.newBufferedReader(path);
        }
        String brotli = System.getProperty("brotli.bin", "brotli");
        Process p =
                new ProcessBuilder(brotli, "-dc", path.toAbsolutePath().toString())
                        .redirectError(ProcessBuilder.Redirect.INHERIT)
                        .start();
        return new BufferedReader(new InputStreamReader(p.getInputStream(), StandardCharsets.UTF_8));
    }

    /** Parses a cell; a non-numeric token or a blank becomes NaN. */
    private static double parseDouble(String s) {
        s = s.trim();
        if (s.isEmpty()) {
            return Double.NaN;
        }
        try {
            return Double.parseDouble(s);
        } catch (NumberFormatException e) {
            return Double.NaN;
        }
    }

    public int size() {
        return rows.size();
    }

    public List<String> columns() {
        return columns;
    }

    public boolean has(String col) {
        return index.containsKey(col);
    }

    /** Value of {@code col} in {@code row}, or NaN if the column is absent. */
    public double get(int row, String col) {
        Integer i = index.get(col);
        return (i == null) ? Double.NaN : rows.get(row)[i];
    }
}
