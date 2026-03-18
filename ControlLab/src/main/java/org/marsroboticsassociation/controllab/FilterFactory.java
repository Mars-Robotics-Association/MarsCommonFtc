package org.marsroboticsassociation.controllab;

import org.marsroboticsassociation.controllib.filter.BiquadLowPassVarDt;
import org.marsroboticsassociation.controllib.filter.Filter;
import org.marsroboticsassociation.controllib.filter.IIR1LowPassVarDt;

public class FilterFactory {
    public enum Type {LOWPASS, BIQUAD, NONE}

    public static Filter create(Type t, double param1, double param2, double param3) {
        switch (t) {
            case LOWPASS:
                return new IIR1LowPassVarDt(IIR1LowPassVarDt.tauFromCutoffHz(param1));
            case BIQUAD:
                return new BiquadLowPassVarDt(param1, param2);
            case NONE:
            default:
                return new Filter() {
                    private double x;

                    @Override
                    public double update(double x, double dt) {
                        this.x = x;
                        return x;
                    }

                    @Override
                    public double getRate() {
                        return Double.NaN;
                    }

                    @Override
                    public double getValue() {
                        return this.x;
                    }

                    @Override
                    public void reset() {
                    }
                };
        }
    }
}
