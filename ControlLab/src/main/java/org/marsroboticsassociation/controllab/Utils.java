package org.marsroboticsassociation.controllab;

import java.io.BufferedWriter;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;

public class Utils {

    /**
     * Estimate lag (seconds) between raw and filtered signals using discrete cross-correlation.
     * Returns lag in seconds where positive means filtered lags behind raw.
     * This computes correlation for integer sample shifts only.
     */
    public static double estimateLagSeconds(List<Double> raw, List<Double> filt, List<Double> time) {
        int n = Math.min(raw.size(), filt.size());
        if (n < 3) return 0.0;

        // zero-mean
        double meanR = raw.stream().mapToDouble(d -> d).average().orElse(0.0);
        double meanF = filt.stream().mapToDouble(d -> d).average().orElse(0.0);

        double[] r = new double[n];
        double[] f = new double[n];
        for (int i = 0; i < n; i++) {
            r[i] = raw.get(i) - meanR;
            f[i] = filt.get(i) - meanF;
        }

        // precompute std for normalization
        double stdR = 0, stdF = 0;
        for (int i = 0; i < n; i++) {
            stdR += r[i] * r[i];
            stdF += f[i] * f[i];
        }
        stdR = Math.sqrt(stdR);
        stdF = Math.sqrt(stdF);
        if (stdR == 0 || stdF == 0) return 0.0; // constant signals

        int maxLag = Math.min(n / 2, 500); // limit lag in samples
        double bestCorr = Double.NEGATIVE_INFINITY;
        int bestLag = 0;

        for (int lag = -maxLag; lag <= maxLag; lag++) {
            double cross = 0;
            double sumR2 = 0;
            double sumF2 = 0;
            int count = 0;

            for (int i = 0; i < n; i++) {
                int j = i + lag;
                if (j < 0 || j >= n) continue;

                double ri = r[i];
                double fj = f[j];

                cross += ri * fj;
                sumR2 += ri * ri;
                sumF2 += fj * fj;
                count++;
            }

            if (count < 3) continue; // too few samples

            double corr = cross / Math.sqrt(sumR2 * sumF2);

            if (corr > bestCorr) {
                bestCorr = corr;
                bestLag = lag;
            }
        }


        // compute average dt
        double dtAvg = 0;
        int dtCount = Math.min(n - 1, 1000);
        for (int i = 1; i <= dtCount; i++) {
            dtAvg += time.get(i) - time.get(i - 1);
        }
        dtAvg /= dtCount;

        return -bestLag * dtAvg; // positive lag = filtered behind raw
    }


    public static void exportToCsv(Path outFile, List<Double> time, List<Double> raw, List<Double> filtered) throws IOException {
        try (BufferedWriter bw = Files.newBufferedWriter(outFile)) {
            bw.write("time,raw,filtered");
            bw.newLine();
            int n = Math.min(Math.min(time.size(), raw.size()), filtered.size());
            for (int i = 0; i < n; i++) {
                bw.write(String.format("%f,%f,%f", time.get(i), raw.get(i), filtered.get(i)));
                bw.newLine();
            }
        }
    }
}
