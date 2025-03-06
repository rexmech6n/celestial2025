package com.celestial.utils.camera

import org.photonvision.PhotonCamera
import org.photonvision.targeting.PhotonTrackedTarget

class CameraOutput(cameraName: String?) {
    private val camera = PhotonCamera(cameraName)
    val bestTarget: PhotonTrackedTarget?
        get() = camera.allUnreadResults.lastOrNull()?.bestTarget
}