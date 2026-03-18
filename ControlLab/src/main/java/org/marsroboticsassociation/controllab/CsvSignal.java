package org.marsroboticsassociation.controllab;

import com.univocity.parsers.csv.*;

import java.io.*;
import java.nio.file.*;
import java.nio.charset.StandardCharsets;
import java.util.*;

public class CsvSignal {
    private final LinkedHashMap<String, List<Double>> columns = new LinkedHashMap<>();
    private String timeKey;
    private String dataKey;
    private double windowStart = Double.NEGATIVE_INFINITY;
    private double windowEnd = Double.POSITIVE_INFINITY;
    private int rowCount = 0;

    private CsvSignal() {
    }

    public static CsvSignal load(String path) throws IOException {
        Path p = Paths.get(path);

        CsvParserSettings settings = new CsvParserSettings();
        settings.setLineSeparatorDetectionEnabled(true);
        settings.setDelimiterDetectionEnabled(true, ',', ';', '\t', '|');
        settings.setQuoteDetectionEnabled(true);

        settings.setSkipEmptyLines(true);
        settings.setIgnoreLeadingWhitespaces(true);
        settings.setIgnoreTrailingWhitespaces(true);
        settings.setUnescapedQuoteHandling(UnescapedQuoteHandling.STOP_AT_DELIMITER);
        settings.setNullValue("");
        settings.setEmptyValue("");

        settings.getFormat().setComment('\0');

        settings.setHeaderExtractionEnabled(true);

        settings.setMaxCharsPerColumn(1_000_000);
        settings.setMaxColumns(10_000);


        CsvParser parser = new CsvParser(settings);
        Reader r = Files.newBufferedReader(p, StandardCharsets.UTF_8);
        parser.beginParsing(r);

        String[] headers = parser.getContext().headers();
        CsvSignal sig = new CsvSignal();
        for (String h : headers) {
            sig.columns.put(h, new ArrayList<>());
        }

        String[] row;
        while ((row = parser.parseNext()) != null) {
            for (int i = 0; i < headers.length; i++) {
                String h = headers[i];
                String s = i < row.length ? row[i] : "";
                double val = Double.NaN;
                if (s != null && !s.isBlank()) {
                    try {
                        val = Double.parseDouble(s.trim());
                    } catch (NumberFormatException ignored) {
                        val = Double.NaN; // non-numeric fields (logs etc.)
                    }
                }
                sig.columns.get(h).add(val);
            }
            sig.rowCount++;
        }

        parser.stopParsing();
        return sig;
    }

    // -------------------------
    // Methods to mimic CsvSignal
    // -------------------------
    public CsvSignal select(String timeColumn, String dataColumn) {
        if (!columns.containsKey(timeColumn) || !columns.containsKey(dataColumn))
            throw new IllegalArgumentException("Column not found: " + timeColumn + " or " + dataColumn);
        this.timeKey = timeColumn;
        this.dataKey = dataColumn;
        return this;
    }

    public CsvSignal window(double start, double end) {
        this.windowStart = start;
        this.windowEnd = end;
        return this;
    }

    public List<Double> time() {
        return slice(columns.get(Objects.requireNonNull(timeKey)));
    }

    public List<Double> data() {
        return slice(columns.get(Objects.requireNonNull(dataKey)));
    }

    private List<Double> slice(List<Double> src) {
        if (timeKey == null) throw new IllegalStateException("timeKey not set");
        List<Double> t = columns.get(timeKey);
        List<Double> out = new ArrayList<>();
        for (int i = 0; i < src.size(); i++) {
            double tv = t.get(i);
            if (Double.isNaN(tv)) continue;
            if (tv >= windowStart && tv <= windowEnd) out.add(src.get(i));
        }
        return out;
    }

    public OptionalDouble minTime() {
        return columns.containsKey(timeKey)
                ? columns.get(timeKey).stream().mapToDouble(d -> d).filter(d -> !Double.isNaN(d)).min()
                : OptionalDouble.empty();
    }

    public OptionalDouble maxTime() {
        return columns.containsKey(timeKey)
                ? columns.get(timeKey).stream().mapToDouble(d -> d).filter(d -> !Double.isNaN(d)).max()
                : OptionalDouble.empty();
    }

    public Set<String> headers() {
        return Collections.unmodifiableSet(columns.keySet());
    }

    public int rowCount() {
        return rowCount;
    }
}
