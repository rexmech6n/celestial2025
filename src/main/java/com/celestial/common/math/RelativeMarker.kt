package com.celestial.common.math

import kotlin.math.cos
import kotlin.math.sin

@Suppress("MemberVisibilityCanBePrivate")
class RelativeMarker(
    x: Double,
    y: Double,
    val azimuth: Double = 0.0,
): Vector2D(x, y) {
    override operator fun plus(other: Vector2D) = RelativeMarker(x + other.x, y + other.y, azimuth)

    override fun rotate(angle: Double) = RelativeMarker(
        x * cos(angle) - y * sin(angle),
        x * sin(angle) + y * cos(angle),
        azimuth + angle,
    )

    fun withAzimuth(azimuth: Double) = RelativeMarker(x, y, azimuth)

    override fun toString(): String {
        return "RelativeMarker(x=$x, y=$y, azimuth=$azimuth)"
    }

    constructor(vector: Vector2D, azimuth: Double = 0.0): this(vector.x, vector.y, azimuth)

    companion object {
        fun zero() = RelativeMarker(0.0, 0.0)
    }
}