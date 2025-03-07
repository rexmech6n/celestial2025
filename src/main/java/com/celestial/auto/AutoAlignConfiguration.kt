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
    const val APRIL_TAG_REEF_Y_DISTANCE = 0.28
    const val ZERO = 0.0

    const val AUTO_ALIGN_X_KP = 2.2
    const val AUTO_ALIGN_X_KI = 1.0
    const val AUTO_ALIGN_X_KD = 0.8

    val CAMERA_RELATIVE_MARKER = RelativeMarker(ROBOT_CAMERA_HORIZONTAL_DISTANCE, ZERO)
    val REEF_RELATIVE_MARKER = RelativeMarker(APRIL_TAG_REEF_HORIZONTAL_DISTANCE, APRIL_TAG_REEF_Y_DISTANCE)
}