package org.firstinspires.ftc.teamcode.subsystems

import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.ServoEx
import dev.nextftc.hardware.positionable.SetPosition

object Claw : Subsystem {
    private val servo = ServoEx("claw_servo")

    @get:JvmName("open")
    val open = SetPosition(servo, 0.1).requires(this)

    @get:JvmName("close")
    val close = SetPosition(servo, 0.2).requires(this)
}