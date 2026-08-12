package org.marsroboticsassociation.controllib.filter

/*
 * Copyright (c) 2023, Peter Abeles. All Rights Reserved.
 *
 * This file is part of Efficient Java Matrix Library (EJML).
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

import org.ejml.data.DMatrixRMaj

/**
 * This is an interface for a discrete time Kalman filter with no control input:
 *
 * x<sub>k</sub> = F<sub>k</sub> x<sub>k-1</sub> + w<sub>k</sub>
 * z<sub>k</sub> = H<sub>k</sub> x<sub>k</sub> + v<sub>k</sub>
 *
 * w<sub>k</sub> ~ N(0,Q<sub>k</sub>)
 * v<sub>k</sub> ~ N(0,R<sub>k</sub>)
 *
 * @author Peter Abeles
 */
interface KalmanFilter {
    /**
     * Specify the kinematics model of the Kalman filter. This must be called first before any other
     * functions.
     *
     * @param F State transition matrix.
     * @param Q plant noise.
     * @param H measurement projection matrix.
     */
    fun configure(F: DMatrixRMaj, Q: DMatrixRMaj, H: DMatrixRMaj)

    /**
     * The prior state estimate and covariance.
     *
     * @param x The estimated system state.
     * @param P The covariance of the estimated system state.
     */
    fun setState(x: DMatrixRMaj, P: DMatrixRMaj)

    /** Predicts the state of the system forward one time step. */
    fun predict()

    /**
     * Updates the state provided the observation from a sensor.
     *
     * @param z Measurement.
     * @param R Measurement covariance.
     */
    fun update(z: DMatrixRMaj, R: DMatrixRMaj)

    /**
     * Returns the current estimated state of the system.
     *
     * @return The state.
     */
    val state: DMatrixRMaj

    /**
     * Returns the estimated state's covariance matrix.
     *
     * @return The covariance.
     */
    val covariance: DMatrixRMaj
}
