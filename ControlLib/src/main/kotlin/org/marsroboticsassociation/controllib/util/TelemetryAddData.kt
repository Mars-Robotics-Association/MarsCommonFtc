package org.marsroboticsassociation.controllib.util

fun interface TelemetryAddData {
    fun addData(caption: String, format: String, value: Any?)
}
