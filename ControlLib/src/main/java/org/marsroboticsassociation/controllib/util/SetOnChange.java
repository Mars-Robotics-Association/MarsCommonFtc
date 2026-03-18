package org.marsroboticsassociation.controllib.util;

import java.util.Objects;
import java.util.function.Consumer;
import java.util.function.DoubleConsumer;

public final class SetOnChange<T> {

    /* =========================
     * Backend interface
     * ========================= */
    private interface Backend<T> {
        void set(T value);

        T get();

        default void setDouble(double value) {
            throw new UnsupportedOperationException(
                    "Primitive fast path not supported for this type");
        }
    }

    /* =========================
     * Object backend (generic)
     * ========================= */
    private static final class ObjectBackend<T> implements Backend<T> {
        private final Consumer<T> setter;
        private T value;

        ObjectBackend(T initValue, Consumer<T> setter) {
            this.value = initValue;
            this.setter = setter;
            setter.accept(initValue);
        }

        @Override
        public void set(T value) {
            if (!Objects.equals(value, this.value)) {
                this.value = value;
                setter.accept(value);
            }
        }

        @Override
        public T get() {
            return value;
        }
    }

    /* =========================
     * Double backend (primitive, epsilon + zero snap)
     * ========================= */
    private static final class DoubleBackend implements Backend<Double> {
        private final DoubleConsumer setter;
        private final double epsilon;
        private double value;

        DoubleBackend(double initValue,
                      double epsilon,
                      DoubleConsumer setter) {
            if (epsilon < 0.0) {
                throw new IllegalArgumentException(
                        "epsilon must be >= 0");
            }

            this.epsilon = epsilon;
            this.setter = setter;
            this.value = snap(initValue);
            setter.accept(this.value);
        }

        @Override
        public void set(Double value) {
            // Compatibility (boxed) path
            setDouble(value);
        }

        @Override
        public void setDouble(double v) {
            double snapped = snap(v);
            if (changed(snapped)) {
                this.value = snapped;
                setter.accept(snapped);
            }
        }

        private double snap(double v) {
            return Math.abs(v) < epsilon ? 0.0 : v;
        }

        private boolean changed(double v) {
            if (epsilon == 0.0) {
                return v != value;
            }
            return Math.abs(v - value) > epsilon;
        }

        @Override
        public Double get() {
            // Box only if explicitly requested
            return value;
        }
    }

    /* =========================
     * Public façade
     * ========================= */
    private final Backend<T> backend;

    private SetOnChange(Backend<T> backend) {
        this.backend = backend;
    }

    /* =========================
     * Factory methods
     * ========================= */

    /**
     * Generic object-based factory
     */
    public static <T> SetOnChange<T> of(
            T initValue,
            Consumer<T> setter
    ) {
        return new SetOnChange<>(new ObjectBackend<>(initValue, setter));
    }

    /**
     * Primitive double factory (exact comparison, no snap)
     */
    public static SetOnChange<Double> ofDouble(
            double initValue,
            DoubleConsumer setter
    ) {
        return new SetOnChange<>(
                new DoubleBackend(initValue, 0.0, setter));
    }

    /**
     * Primitive double factory with fixed epsilon and zero snap
     */
    public static SetOnChange<Double> ofDouble(
            double initValue,
            double epsilon,
            DoubleConsumer setter
    ) {
        return new SetOnChange<>(
                new DoubleBackend(initValue, epsilon, setter));
    }

    /* =========================
     * API methods
     * ========================= */

    /**
     * Generic entry point (may box)
     */
    public void set(T value) {
        backend.set(value);
    }

    /**
     * Primitive fast path (allocation-free for Double backend)
     */
    public void set(double value) {
        backend.setDouble(value);
    }

    public T get() {
        return backend.get();
    }
}
