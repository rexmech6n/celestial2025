package com.celestial.common.math

import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

open class Vector2D(
    val x: Double,
    val y: Double,
) {
    open operator fun plus(other: Vector2D) = Vector2D(x + other.x, y + other.y)
    open operator fun minus(other: Vector2D) = Vector2D(x - other.x, y - other.y)
    open operator fun times(scalar: Double) = Vector2D(x * scalar, y * scalar)
    open operator fun div(scalar: Double) = Vector2D(x / scalar, y / scalar)
    open operator fun unaryMinus() = Vector2D(-x, -y)

    open fun dot(other: Vector2D) = x * other.x + y * other.y
    open fun cross(other: Vector2D) = x * other.y - y * other.x
    open fun magnitude() = sqrt(x * x + y * y)
    open fun normalize() = this / magnitude()
    open fun rotate(angle: Double) = Vector2D(
        x * cos(angle) - y * sin(angle),
        x * sin(angle) + y * cos(angle),
    )

    fun stripX() = Vector2D(0.0, y)
    fun stripY() = Vector2D(x, 0.0)

    override fun toString(): String {
        return "Vector2D(x=$x, y=$y)"
    }

    companion object {
        fun x(x: Double) = Vector2D(x, 0.0)
        fun y(y: Double) = Vector2D(0.0, y)
    }
}