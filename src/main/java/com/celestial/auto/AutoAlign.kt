package com.celestial.auto

import com.celestial.common.math.RelativeMarker
import com.celestial.common.math.Vector2D
import com.celestial.utils.camera.CameraOutput
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.networktables.DoubleSubscriber
import edu.wpi.first.networktables.DoubleTopic
import edu.wpi.first.networktables.NetworkTableEvent
import edu.wpi.first.networktables.NetworkTableInstance
import java.util.*
import kotlin.math.absoluteValue
import kotlin.math.min
import kotlin.math.sign

object AutoAlign {
    lateinit var cameraOutput: CameraOutput
    var target: Vector2D? = null
        get() {
            if(System.currentTimeMillis() - lastUpdate.time > 4000) {
                field = null
            }
            return field
        }
    var lastUpdate: Date = Date()
    var adjustment: RelativeMarker = RelativeMarker(0.0, 0.0)
    var k: Long = 0

    var state = AutoAlignState.IDLE

    lateinit var xPidController: PIDController
    lateinit var ramPidController: PIDController

    var inst: NetworkTableInstance = NetworkTableInstance.getDefault()
    var pTopic: DoubleTopic = inst.getDoubleTopic("align-P")
    var iTopic: DoubleTopic = inst.getDoubleTopic("align-I")
    var dTopic: DoubleTopic = inst.getDoubleTopic("align-D")

    var pSubscriber: DoubleSubscriber = pTopic.subscribe(0.0)
    var iSubscriber: DoubleSubscriber = iTopic.subscribe(0.0)
    var dSubscriber: DoubleSubscriber = dTopic.subscribe(0.0)

    init {
        inst.addListener(
            pSubscriber,
            EnumSet.of(NetworkTableEvent.Kind.kValueAll)
        ) { event: NetworkTableEvent? ->
            println("P Change")
            setPidConstants(pSubscriber.get(), iSubscriber.get(), dSubscriber.get())
        }

        inst.addListener(
            iSubscriber,
            EnumSet.of(NetworkTableEvent.Kind.kValueAll)
        ) { event: NetworkTableEvent? ->
            println("I Change")
            setPidConstants(pSubscriber.get(), iSubscriber.get(), dSubscriber.get())
        }

        inst.addListener(
            dSubscriber,
            EnumSet.of(NetworkTableEvent.Kind.kValueAll)
        ) { event: NetworkTableEvent? ->
            println("D Change")
            setPidConstants(pSubscriber.get(), iSubscriber.get(), dSubscriber.get())
        }

        setPidConstants(AutoAlignConfiguration.AUTO_ALIGN_X_KP, AutoAlignConfiguration.AUTO_ALIGN_X_KI, AutoAlignConfiguration.AUTO_ALIGN_X_KD)
    }

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
                adjustment = RelativeMarker.zero()
            }
            AutoAlignState.HORIZONTAL_ALIGN -> {
                /*if(target == null) {
                    state = AutoAlignState.IDLE
                }*/
                adjustment = calculateHorizontalAdjustment()
                if(xPidController.atSetpoint()) {
                    //TODO
                    //state = AutoAlignState.RAMMING
                    //update()
                }
            }
            AutoAlignState.RAMMING -> {
                if(target == null) {
                    state = AutoAlignState.IDLE
                }
                adjustment = calculateRamAdjustment()
                if(xPidController.atSetpoint()) {
                    state = AutoAlignState.DONE
                    update()
                }
            }
            AutoAlignState.DONE -> {
                if(target == null) {
                    state = AutoAlignState.IDLE
                }
                adjustment = RelativeMarker.zero()
            }
        }
        if(k % 20 == 0L) println("Adjustment: $adjustment")
        k++
    }

    fun arm() {
        state = AutoAlignState.HORIZONTAL_ALIGN
        xPidController.reset()
        ramPidController.reset()
    }

    fun disarm() {
        state = AutoAlignState.IDLE
        xPidController.reset()
        ramPidController.reset()
    }

    fun generateChassisSpeeds(): ChassisSpeeds {
        return ChassisSpeeds(ranged(ramPidController.calculate(adjustment.y, 0.0)), -ranged(xPidController.calculate(adjustment.x, 0.0)), 0.0)
    }

    fun ranged(d: Double): Double {
        return min(1.0, (d.absoluteValue / 1.33) + (if(d.absoluteValue > 0.05) 0.25 else 0.0)) * d.sign
    }

    fun isAdjustmentDone(): Boolean {
        return state == AutoAlignState.DONE
    }

    private fun calculateHorizontalAdjustment(): RelativeMarker {
        return target?.let {
            val distX = AutoAlignConfiguration.REEF_RELATIVE_MARKER.stripY() - Vector2D.x(it.x)
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