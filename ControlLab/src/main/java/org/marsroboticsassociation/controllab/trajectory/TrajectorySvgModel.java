package org.marsroboticsassociation.controllab.trajectory;

import java.util.List;

record TrajectorySvgModel(
        double xMin, double xMax, double minY, double maxY, List<TrajectorySvgSeries> series) {}
