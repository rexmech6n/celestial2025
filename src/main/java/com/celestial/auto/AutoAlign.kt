package com.celestial.auto

import com.celestial.Constants
import com.celestial.RobotContainer
import com.celestial.common.math.RelativeMarker
import com.celestial.common.math.Vector2D
import com.celestial.subsystems.SwerveSubsystem
import com.celestial.utils.camera.CameraOutput
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.Command
import java.util.*
import kotlin.math.absoluteValue
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sign

object AutoAlign {
    lateinit var cameraOutput: CameraOutput
    var target: Vector2D? = null
    var lastUpdate: Date = Date()
    var adjustment: RelativeMarker = RelativeMarker(0.0, 0.0)
    var k: Long = 0

    fun init() {
        cameraOutput = CameraOutput(AutoAlignConfiguration.CAMERA_NAME)
    }

    fun update() {
        registerTarget(cameraOutput)
        adjustment = calculateHorizontalAdjustment()
        if(k % 20 == 0L) println("Adjustment: $adjustment")
        k++
    }

    fun generateChassisSpeeds(): ChassisSpeeds {
        return ChassisSpeeds(0.0, min(0.05, adjustment.x.absoluteValue) * 20 * 0.33 * adjustment.x.sign, 0.0)
    }

    fun isAdjustmentDone(): Boolean {
        return false
    }

    private fun calculateHorizontalAdjustment(): RelativeMarker {
        return target?.let {
            val distX = AutoAlignConfiguration.REEF_RELATIVE_MARKER.stripX() - Vector2D.x(it.x)
            RelativeMarker(distX)
        } ?: RelativeMarker.zero()
    }

    private fun registerTarget(t: CameraOutput) {
        t.bestTarget?.takeIf { it.bestCameraToTarget != null && it.bestCameraToTarget.translation != null }?.also {
            val translation3d = it.getBestCameraToTarget().translation
            target = RelativeMarker(-translation3d.y, translation3d.z) + AutoAlignConfiguration.CAMERA_RELATIVE_MARKER
            lastUpdate = Date()
        }
    }
}