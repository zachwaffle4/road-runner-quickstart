package org.firstinspires.ftc.teamcode.examples

import com.acmerobotics.roadrunner.Pose2d
import dev.nextftc.core.commands.Command
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.ftc.NextFTCOpMode
import org.firstinspires.ftc.teamcode.MecanumDrive
import org.firstinspires.ftc.teamcode.subsystems.Claw
import org.firstinspires.ftc.teamcode.subsystems.Lift
import kotlin.time.Duration.Companion.milliseconds

class OtherCommandsKt : NextFTCOpMode() {
    private val startPose = Pose2d(9.0, 111.0, Math.toRadians(-90.0))
    private val scorePose = Pose2d(16.0, 128.0, Math.toRadians(-45.0))

    val drive: MecanumDrive by onInit {
        MecanumDrive(hardwareMap, startPose)
    }

    val driveCommand: Command by onInit {
        drive.commandBuilder(startPose)
            .splineTo(scorePose.position, scorePose.heading)
            .afterTime(500.milliseconds, Lift.toHigh)
            .stopAndAdd(Claw.open)
            .build()
    }

    init {
        addComponents(
            SubsystemComponent(Lift, Claw)
        )
    }

    override fun onStartButtonPressed() {
        driveCommand.schedule()
    }
}
