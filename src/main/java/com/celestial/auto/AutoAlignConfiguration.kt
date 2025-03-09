package com.celestial.auto

import com.celestial.common.math.RelativeMarker
import com.celestial.utils.camera.CameraOutput
import org.photonvision.targeting.PhotonTrackedTarget
import java.util.Date

object AutoAlignConfiguration {
    const val CAMERA_NAME = "Arducam_OV9281_USB_Camera"
    const val STAGE_MODE = true

    const val ROBOT_CAMERA_HORIZONTAL_DISTANCE = 0.26
    const val APRIL_TAG_REEF_HORIZONTAL_DISTANCE = 0.165
    const val APRIL_TAG_REEF_Y_DISTANCE = 0.35
    const val ZERO = 0.0

    const val AUTO_ALIGN_X_THRESHOLD = 0.015
    const val AUTO_ALIGN_AZIMUTH_THRESHOLD = 1
    const val AUTO_ALIGN_RAM_THRESHOLD = 0.01

    const val AUTO_ALIGN_X_KP = 2.8
    const val AUTO_ALIGN_X_KI = 1.45
    const val AUTO_ALIGN_X_KD = 1.0

    const val AUTO_ALIGN_RAM_KP = 2.66
    const val AUTO_ALIGN_RAM_KI = 0.2
    const val AUTO_ALIGN_RAM_KD = 0.1

    const val AUTO_ALIGN_THETA_KP = 1.0
    const val AUTO_ALIGN_THETA_KI = 0.2
    const val AUTO_ALIGN_THETA_KD = 0.0

    val CAMERA_RELATIVE_MARKER = RelativeMarker(ROBOT_CAMERA_HORIZONTAL_DISTANCE, ZERO)
    val REEF_RELATIVE_MARKER = RelativeMarker(APRIL_TAG_REEF_HORIZONTAL_DISTANCE, APRIL_TAG_REEF_Y_DISTANCE)
}