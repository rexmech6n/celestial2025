package com.celestial.auto

import com.celestial.common.math.RelativeMarker
import com.celestial.common.math.Vector2D
import com.celestial.utils.camera.CameraOutput
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.networktables.DoubleSubscriber
import edu.wpi.first.networktables.DoubleTopic
import edu.wpi.first.networktables.NetworkTableEvent
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import java.util.*
import kotlin.math.absoluteValue
import kotlin.math.min
import kotlin.math.sign

object AutoAlign {
    lateinit var cameraOutput: CameraOutput
    var target: RelativeMarker? = null
        get() {
            if(System.currentTimeMillis() - lastUpdate.time > 2000) {
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
    lateinit var thetaPidController: PIDController

    var inst: NetworkTableInstance = NetworkTableInstance.getDefault()
    var xpTopic: DoubleTopic = inst.getDoubleTopic("xRam-P")
    var xiTopic: DoubleTopic = inst.getDoubleTopic("xRam-I")
    var xdTopic: DoubleTopic = inst.getDoubleTopic("xRam-D")

    var xpSubscriber: DoubleSubscriber = xpTopic.subscribe(0.0)
    var xiSubscriber: DoubleSubscriber = xiTopic.subscribe(0.0)
    var xdSubscriber: DoubleSubscriber = xdTopic.subscribe(0.0)

    var thetaPTopic: DoubleTopic = inst.getDoubleTopic("theta-P")
    var thetaITopic: DoubleTopic = inst.getDoubleTopic("theta-I")
    var thetaDTopic: DoubleTopic = inst.getDoubleTopic("theta-D")

    var thetaPSubscriber: DoubleSubscriber = thetaPTopic.subscribe(0.0)
    var thetaISubscriber: DoubleSubscriber = thetaITopic.subscribe(0.0)
    var thetaDSubscriber: DoubleSubscriber = thetaDTopic.subscribe(0.0)

    init {
        inst.addListener(
            xpSubscriber,
            EnumSet.of(NetworkTableEvent.Kind.kValueAll)
        ) { event: NetworkTableEvent? ->
            println("P Change")
            setPidConstants(xpSubscriber.get(), xiSubscriber.get(), xdSubscriber.get())
        }

        inst.addListener(
            xiSubscriber,
            EnumSet.of(NetworkTableEvent.Kind.kValueAll)
        ) { event: NetworkTableEvent? ->
            println("I Change")
            setPidConstants(xpSubscriber.get(), xiSubscriber.get(), xdSubscriber.get())
        }

        inst.addListener(
            xdSubscriber,
            EnumSet.of(NetworkTableEvent.Kind.kValueAll)
        ) { event: NetworkTableEvent? ->
            println("D Change")
            setPidConstants(xpSubscriber.get(), xiSubscriber.get(), xdSubscriber.get())
        }


        inst.addListener(
            thetaPTopic,
            EnumSet.of(NetworkTableEvent.Kind.kValueAll)
        ) { event: NetworkTableEvent? ->
            println("P Change")
            setThetaPidConstants(thetaPSubscriber.get(), thetaISubscriber.get(), thetaDSubscriber.get())
        }

        inst.addListener(
            thetaITopic,
            EnumSet.of(NetworkTableEvent.Kind.kValueAll)
        ) { event: NetworkTableEvent? ->
            println("I Change")
            setThetaPidConstants(thetaPSubscriber.get(), thetaISubscriber.get(), thetaDSubscriber.get())
        }

        inst.addListener(
            thetaDTopic,
            EnumSet.of(NetworkTableEvent.Kind.kValueAll)
        ) { event: NetworkTableEvent? ->
            println("P Change")
            setThetaPidConstants(thetaPSubscriber.get(), thetaISubscriber.get(), thetaDSubscriber.get())
        }

        setPidConstants(AutoAlignConfiguration.AUTO_ALIGN_X_KP, AutoAlignConfiguration.AUTO_ALIGN_X_KI, AutoAlignConfiguration.AUTO_ALIGN_X_KD)
        setThetaPidConstants(AutoAlignConfiguration.AUTO_ALIGN_THETA_KP, AutoAlignConfiguration.AUTO_ALIGN_THETA_KI, AutoAlignConfiguration.AUTO_ALIGN_THETA_KD)


        xpTopic.publish().set(AutoAlignConfiguration.AUTO_ALIGN_X_KP)
        xiTopic.publish().set(AutoAlignConfiguration.AUTO_ALIGN_X_KI)
        xdTopic.publish().set(AutoAlignConfiguration.AUTO_ALIGN_X_KD)
        thetaPTopic.publish().set(AutoAlignConfiguration.AUTO_ALIGN_THETA_KP)
        thetaITopic.publish().set(AutoAlignConfiguration.AUTO_ALIGN_THETA_KI)
        thetaDTopic.publish().set(AutoAlignConfiguration.AUTO_ALIGN_THETA_KD)

    }

    fun setPidConstants(kP: Double, kI: Double, kD: Double) {
        xPidController = PIDController(kP, kI, kD)
        ramPidController = PIDController(AutoAlignConfiguration.AUTO_ALIGN_RAM_KP, AutoAlignConfiguration.AUTO_ALIGN_RAM_KI, AutoAlignConfiguration.AUTO_ALIGN_RAM_KD)
        xPidController.setpoint = 0.0
        //xPidController.setTolerance(0.005)
        ramPidController.setpoint = 0.0
        //ramPidController.setTolerance(0.001)
    }

    fun setThetaPidConstants(kP: Double, kI: Double, kD: Double) {
        thetaPidController = PIDController(kP, kI, kD)
        thetaPidController.setpoint = 0.0
        thetaPidController.setTolerance(4.0)
    }

    fun init() {
        cameraOutput = CameraOutput(AutoAlignConfiguration.CAMERA_NAME)
    }

    fun update() {
        registerTarget(cameraOutput)
        xPidController.calculate(adjustment.x)
        ramPidController.calculate(adjustment.y)
        thetaPidController.calculate(adjustment.azimuth)
        when(state) {
            AutoAlignState.IDLE -> {
                adjustment = RelativeMarker.zero()
            }
            AutoAlignState.HORIZONTAL_ALIGN -> {
                /*if(target == null) {
                    state = AutoAlignState.IDLE
                }*/
                adjustment = calculateHorizontalAdjustment()
                if(adjustment.x.absoluteValue < AutoAlignConfiguration.AUTO_ALIGN_X_THRESHOLD && adjustment.azimuth.absoluteValue < AutoAlignConfiguration.AUTO_ALIGN_AZIMUTH_THRESHOLD) {
                    //TODO
                    println("xPid at setpoint")
                    xPidController.reset()
                    ramPidController.reset()
                    thetaPidController.reset()
                    state = AutoAlignState.RAMMING
                    update()
                }
            }
            AutoAlignState.RAMMING -> {
                adjustment = calculateRamAdjustment()
                if(adjustment.y.absoluteValue < AutoAlignConfiguration.AUTO_ALIGN_RAM_THRESHOLD) {
                    println("ramPid at setpoint")
                    xPidController.reset()
                    ramPidController.reset()
                    thetaPidController.reset()
                    state = AutoAlignState.DONE
                    update()
                }
            }
            AutoAlignState.DONE -> {
                if (k % 20 == 0L) println("AutoAlignState: Done")
                adjustment = RelativeMarker.zero()
            }
        }
        if(k % 50 == 0L) println("state=${state}")

        if (k % 20 == 0L) SmartDashboard.putString("AutoAlign Adjustment:", "adj.x=" + adjustment.x + ", adj.y=" + adjustment.y + ", adj.az=" + adjustment.azimuth)

        //if(k % 20 == 0L) println("Adjustment: $adjustment")
        k++
    }

    fun arm() {
        state = AutoAlignState.HORIZONTAL_ALIGN
        xPidController.reset()
        ramPidController.reset()
        thetaPidController.reset()
    }

    fun disarm() {
        state = AutoAlignState.IDLE
        xPidController.reset()
        ramPidController.reset()
        thetaPidController.reset()
    }

    fun generateChassisSpeeds(): ChassisSpeeds {
        if(state == AutoAlignState.DONE ) return ChassisSpeeds(0.0, 0.0, 0.0)
        if(k % 50 == 0L) println("adj azimuth=${adjustment.azimuth}")
        return ChassisSpeeds(ranged(ramPidController.calculate(adjustment.y, 0.0)), if(state == AutoAlignState.RAMMING) 0.0 else (-rangedBoosted(xPidController.calculate(adjustment.x, 0.0)) * ((20 - min(10.0, adjustment.azimuth.absoluteValue)) / 20)), -rangedTheta(
            toRadians(thetaPidController.calculate(adjustment.azimuth, 0.0))))
    }

    fun toRadians(d: Double): Double {
        return Math.toRadians(d)
    }

    fun rangedBoosted(d: Double): Double {
        return min(1.0, (d.absoluteValue / 1.33)) * d.sign
    }

    fun ranged(d: Double): Double {
        return min(0.88, d.absoluteValue) * d.sign
    }

    fun rangedTheta(theta: Double): Double {
        return min(12.0, theta.absoluteValue) * theta.sign
    }

    fun isAdjustmentDone(): Boolean {
        return state == AutoAlignState.DONE
    }

    private fun calculateHorizontalAdjustment(): RelativeMarker {
        return target?.let {
            val distX = AutoAlignConfiguration.REEF_RELATIVE_MARKER.stripY() - Vector2D.x(it.x)
            RelativeMarker(distX).withAzimuth((it.azimuth.absoluteValue - 180) * it.azimuth.sign)
        } ?: RelativeMarker.zero()
    }

    private fun calculateRamAdjustment(): RelativeMarker {
        return target?.let {
            val dist = AutoAlignConfiguration.REEF_RELATIVE_MARKER.stripX() - Vector2D.y(it.y)
            RelativeMarker(dist)
        } ?: RelativeMarker.zero()
    }

    private fun registerTarget(t: CameraOutput) {
        t.bestTarget?.takeIf { it.bestCameraToTarget != null && it.bestCameraToTarget.translation != null }?.also {
            val cameraToBestTarget = it.getBestCameraToTarget()
            val translation3d = cameraToBestTarget.translation
            val rotation3d = cameraToBestTarget.rotation
            target = RelativeMarker(-translation3d.y, translation3d.x, Math.toDegrees(rotation3d.z)) + AutoAlignConfiguration.CAMERA_RELATIVE_MARKER
            if (k % 20 == 0L) SmartDashboard.putString("Target:", "t.x=" + target?.x + ", t.y=" + target?.y + ", t.az=" + target?.azimuth)
            lastUpdate = Date()
        }
    }
}