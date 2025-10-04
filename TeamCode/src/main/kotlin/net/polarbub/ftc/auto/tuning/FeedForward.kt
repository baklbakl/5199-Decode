package net.polarbub.ftc.auto.tuning

import com.acmerobotics.roadrunner.DualNum
import com.acmerobotics.roadrunner.Time
import kotlin.math.abs
import kotlin.math.withSign

abstract class FeedForward {
    abstract fun compute(vel: Double, accel: Double) : Double
    open fun compute(vel: DualNum<Time>) : Double = compute(vel[0], vel[1])
}

data class MotorFeedforward (
    @JvmField
    val kS: Double,
    @JvmField
    val kV: Double,
    @JvmField
    val kA: Double,
) : FeedForward() {
    /**
     * @usesMathJax
     *
     * Computes the (normalized) voltage \(k_s \cdot \operatorname{sign}(k_v \cdot v + k_a \cdot a) + k_v \cdot v + k_a \cdot a\).
     *
     * @param[vel] \(v\)
     * @param[accel] \(a\)
     */

    override fun compute(vel: Double, accel: Double) = kS.withSign(vel) + kV * vel + kA * accel
}

data class AsymMotorFeedforward (
    @JvmField
    val kS: Double,
    @JvmField
    val kV: Double,
    @JvmField
    val kASpeedUp: Double,
    @JvmField
    val kASlowDown: Double,
) : FeedForward() {
    /**
     * @param[vel] \(v\)
     * @param[accel] \(a\)
     */

    override fun compute(vel: Double, accel: Double): Double {
        val kA = if((abs(vel) == vel) == (abs(accel) == accel)) {
            kASpeedUp
        } else {
            kASlowDown
        }

        return kS.withSign(vel) + kV * vel + kA * accel
    }
}