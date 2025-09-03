package org.firstinspires.ftc.teamcode.subsystems

import dev.nextftc.control.builder.controlSystem
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.controllable.RunToPosition
import dev.nextftc.hardware.impl.MotorEx

object Lift : Subsystem {
    private val motor = MotorEx("lift_motor")

    private val controlSystem = controlSystem {
        posPid(0.005, 0.0, 0.0)
        elevatorFF(0.0)
    }

    @get:JvmName("toLow")
    val toLow = RunToPosition(controlSystem, 0.0).requires(this)

    @get:JvmName("toMiddle")
    val toMiddle = RunToPosition(controlSystem, 500.0).requires(this)

    @get:JvmName("toHigh")
    val toHigh = RunToPosition(controlSystem, 1200.0).requires(this)

    override fun periodic() {
        motor.power = controlSystem.calculate(motor.state)
    }
}