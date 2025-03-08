package com.celestial.math

class MovingAverage(
    private val windowSize: Int
) {
    private val window = DoubleArray(windowSize)
    private var index = 0
    private var sum = 0.0

    fun add(value: Double) {
        sum -= window[index]
        window[index] = value
        sum += value
        index = (index + 1) % windowSize
    }

    fun with(value: Double): Double {
        add(value)
        return average()
    }

    fun average() = sum / windowSize
}