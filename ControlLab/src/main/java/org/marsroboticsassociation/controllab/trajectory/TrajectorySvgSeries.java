package org.marsroboticsassociation.controllab.trajectory;

import org.marsroboticsassociation.controllib.motion.PolynomialCurveSegment;

import java.awt.BasicStroke;
import java.awt.Color;
import java.util.List;

record TrajectorySvgSeries(
        String label,
        List<PolynomialCurveSegment> segments,
        Color color,
        float strokeWidth,
        BasicStroke stroke) {}
