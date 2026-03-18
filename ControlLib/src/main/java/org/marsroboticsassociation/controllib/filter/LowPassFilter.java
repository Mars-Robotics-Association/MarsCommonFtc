package org.marsroboticsassociation.controllib.filter;

public interface LowPassFilter extends Filter {
    void setCutoffHz(double cutoffHz);
}
