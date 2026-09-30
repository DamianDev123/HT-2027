package org.firstinspires.ftc.teamcode.next.filters

import kotlin.math.abs

class kalmanFilter(
    initialState: Double,
    initialCovariance: Double
) {
    var q: Double = 0.04
    var gateK: Double = 9.0
    var x: Double = initialState
        //private set

    var p: Double = initialCovariance
        //private set

    fun predict(input: Double) {
        x += input
        p += q
    }

    fun update(measurement: Double, r: Double): Double {
        //x += 0
        val residual = measurement - x
        val threshold = gateK * (p + r)

        if (residual * residual > threshold) {
            return 0.0
        }

        val k = p / (p + r)
        x += k * residual
        p *= (1 - k)
        return x
    }

    fun reset(state: Double, covariance: Double) {
        x = state
        p = covariance
    }
}
