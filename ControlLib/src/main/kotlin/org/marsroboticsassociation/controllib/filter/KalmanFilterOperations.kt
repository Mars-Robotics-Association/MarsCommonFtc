package org.marsroboticsassociation.controllib.filter

/*
 * Copyright (c) 2022, Peter Abeles. All Rights Reserved.
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
import org.ejml.dense.row.CommonOps_DDRM.addEquals
import org.ejml.dense.row.CommonOps_DDRM.mult
import org.ejml.dense.row.CommonOps_DDRM.multTransA
import org.ejml.dense.row.CommonOps_DDRM.multTransB
import org.ejml.dense.row.CommonOps_DDRM.subtract
import org.ejml.dense.row.CommonOps_DDRM.subtractEquals
import org.ejml.dense.row.factory.LinearSolverFactory_DDRM
import org.ejml.interfaces.linsol.LinearSolverDense

/**
 * A Kalman filter that is implemented using the operations API, which is procedural. Much of the
 * excessive memory creation/destruction has been reduced from the KalmanFilterSimple. A specialized
 * solver is under to invert the SPD matrix.
 *
 * @author Peter Abeles
 */
class KalmanFilterOperations : KalmanFilter {
    // kinematics description
    private lateinit var F: DMatrixRMaj
    private lateinit var Q: DMatrixRMaj
    private lateinit var H: DMatrixRMaj

    // system state estimate
    private lateinit var x: DMatrixRMaj
    private lateinit var P: DMatrixRMaj

    // these are predeclared for efficiency reasons
    private lateinit var a: DMatrixRMaj
    private lateinit var b: DMatrixRMaj
    private lateinit var y: DMatrixRMaj
    private lateinit var S: DMatrixRMaj
    private lateinit var S_inv: DMatrixRMaj
    private lateinit var c: DMatrixRMaj
    private lateinit var d: DMatrixRMaj
    private lateinit var K: DMatrixRMaj

    private lateinit var solver: LinearSolverDense<DMatrixRMaj>

    override fun configure(F: DMatrixRMaj, Q: DMatrixRMaj, H: DMatrixRMaj) {
        this.F = F
        this.Q = Q
        this.H = H

        val dimenX = F.numCols
        val dimenZ = H.numRows

        a = DMatrixRMaj(dimenX, 1)
        b = DMatrixRMaj(dimenX, dimenX)
        y = DMatrixRMaj(dimenZ, 1)
        S = DMatrixRMaj(dimenZ, dimenZ)
        S_inv = DMatrixRMaj(dimenZ, dimenZ)
        c = DMatrixRMaj(dimenZ, dimenX)
        d = DMatrixRMaj(dimenX, dimenZ)
        K = DMatrixRMaj(dimenX, dimenZ)

        x = DMatrixRMaj(dimenX, 1)
        P = DMatrixRMaj(dimenX, dimenX)

        // covariance matrices are symmetric positive semi-definite
        solver = LinearSolverFactory_DDRM.symmPosDef(dimenX)
    }

    override fun setState(x: DMatrixRMaj, P: DMatrixRMaj) {
        this.x.setTo(x)
        this.P.setTo(P)
    }

    override fun predict() {
        // x = F x
        mult(F, x, a)
        x.setTo(a)

        // P = F P F' + Q
        mult(F, P, b)
        multTransB(b, F, P)
        addEquals(P, Q)
    }

    override fun update(z: DMatrixRMaj, R: DMatrixRMaj) {
        // y = z - H x
        mult(H, x, y)
        subtract(z, y, y)

        // S = H P H' + R
        mult(H, P, c)
        multTransB(c, H, S)
        addEquals(S, R)

        // K = PH'S^(-1)
        if (!solver.setA(S)) throw RuntimeException("Invert failed")
        solver.invert(S_inv)
        multTransA(H, S_inv, d)
        mult(P, d, K)

        // x = x + Ky
        mult(K, y, a)
        addEquals(x, a)

        // P = (I-kH)P = P - (KH)P = P-K(HP)
        mult(H, P, c)
        mult(K, c, b)
        subtractEquals(P, b)
    }

    override val state: DMatrixRMaj
        get() = x

    override val covariance: DMatrixRMaj
        get() = P
}
