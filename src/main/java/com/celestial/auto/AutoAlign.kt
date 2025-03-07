package com.celestial.auto

import com.celestial.Constants
import com.celestial.RobotContainer
import com.celestial.common.math.RelativeMarker
import com.celestial.common.math.Vector2D
import com.celestial.subsystems.SwerveSubsystem
import com.celestial.utils.camera.CameraOutput
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.kinematics.ChassisSpeeds
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

    var state = AutoAlignState.IDLE

    var xPidController = PIDController(AutoAlignConfiguration.AUTO_ALIGN_X_KP, AutoAlignConfiguration.AUTO_ALIGN_X_KI, AutoAlignConfiguration.AUTO_ALIGN_X_KD)
    var ramPidController = PIDController(AutoAlignConfiguration.AUTO_ALIGN_X_KP, AutoAlignConfiguration.AUTO_ALIGN_X_KI, AutoAlignConfiguration.AUTO_ALIGN_X_KD)

    fun setPidConstants(kP: Double, kI: Double, kD: Double) {
        xPidController = PIDController(kP, kI, kD)
        ramPidController = PIDController(kP, kI, kD)
    }

    fun init() {
        cameraOutput = CameraOutput(AutoAlignConfiguration.CAMERA_NAME)
    }

    fun update() {
        registerTarget(cameraOutput)
        when(state) {
            AutoAlignState.IDLE -> {
            }
            AutoAlignState.HORIZONTAL_ALIGN -> {
                if(target == null) {
                    state = AutoAlignState.IDLE
                }
                adjustment = calculateHorizontalAdjustment()
            }
            AutoAlignState.RAMMING -> {
                if(target == null) {
                    state = AutoAlignState.IDLE
                }
                adjustment = calculateRamAdjustment()
            }
            AutoAlignState.DONE -> {
                if(target == null) {
                    state = AutoAlignState.IDLE
                }
            }
        }
        if(k % 20 == 0L) println("Adjustment: $adjustment")
        k++
    }

    fun arm(): ChassisSpeeds {
        return ChassisSpeeds(0.0, min(0.05, adjustment.x.absoluteValue) * 20 * 0.33 * adjustment.x.sign, 0.0)
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

    private fun calculateRamAdjustment(): RelativeMarker {
        return target?.let {
            val dist = AutoAlignConfiguration.REEF_RELATIVE_MARKER.stripY() - Vector2D.y(it.y)
            RelativeMarker(dist)
        } ?: RelativeMarker.zero()
    }

    private fun registerTarget(t: CameraOutput) {
        t.bestTarget?.takeIf { it.bestCameraToTarget != null && it.bestCameraToTarget.translation != null }?.also {
            val translation3d = it.getBestCameraToTarget().translation
            target = RelativeMarker(-translation3d.y, translation3d.x) + AutoAlignConfiguration.CAMERA_RELATIVE_MARKER
            lastUpdate = Date()
        }
    }
}