package org.marsroboticsassociation.controllib.util

import java.util.function.Consumer
import java.util.function.DoubleConsumer
import kotlin.math.abs

class SetOnChange<T> private constructor(private val backend: Backend<T>) {

    private interface Backend<T> {
        fun set(value: T)

        fun get(): T

        fun setDouble(value: Double) {
            throw UnsupportedOperationException("Primitive fast path not supported for this type")
        }
    }

    private class ObjectBackend<T>(initValue: T, private val setter: Consumer<T>) : Backend<T> {
        private var value: T = initValue

        init {
            setter.accept(initValue)
        }

        override fun set(value: T) {
            if (value != this.value) {
                this.value = value
                setter.accept(value)
            }
        }

        override fun get(): T = value
    }

    private class DoubleBackend(
        initValue: Double,
        private val epsilon: Double,
        private val setter: DoubleConsumer,
    ) : Backend<Double> {
        private var value: Double

        init {
            require(epsilon >= 0.0) { "epsilon must be >= 0" }
            value = snap(initValue)
            setter.accept(value)
        }

        override fun set(value: Double) {
            setDouble(value)
        }

        override fun setDouble(value: Double) {
            val snapped = snap(value)
            if (changed(snapped)) {
                this.value = snapped
                setter.accept(snapped)
            }
        }

        private fun snap(v: Double): Double = if (abs(v) < epsilon) 0.0 else v

        private fun changed(v: Double): Boolean {
            return if (epsilon == 0.0) {
                v != value
            } else {
                abs(v - value) > epsilon
            }
        }

        override fun get(): Double = value
    }

    fun set(value: T) {
        backend.set(value)
    }

    /**
     * Primitive fast path for the double backend (allocation-free). Prefer this over [set] when
     * holding a [SetOnChange] of [Double] to avoid overload ambiguity with the generic path.
     */
    fun setDouble(value: Double) {
        backend.setDouble(value)
    }

    fun get(): T = backend.get()

    companion object {
        @JvmStatic
        fun <T> of(initValue: T, setter: Consumer<T>): SetOnChange<T> {
            return SetOnChange(ObjectBackend(initValue, setter))
        }

        @JvmStatic
        fun ofDouble(initValue: Double, setter: DoubleConsumer): SetOnChange<Double> {
            return SetOnChange(DoubleBackend(initValue, 0.0, setter))
        }

        @JvmStatic
        fun ofDouble(
            initValue: Double,
            epsilon: Double,
            setter: DoubleConsumer,
        ): SetOnChange<Double> {
            return SetOnChange(DoubleBackend(initValue, epsilon, setter))
        }
    }
}
