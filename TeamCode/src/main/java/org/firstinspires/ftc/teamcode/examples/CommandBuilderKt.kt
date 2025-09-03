package org.firstinspires.ftc.teamcode.examples

import com.acmerobotics.roadrunner.Pose2d
import dev.nextftc.core.commands.Command
import dev.nextftc.ftc.NextFTCOpMode
import org.firstinspires.ftc.teamcode.MecanumDrive

class CommandBuilderKt : NextFTCOpMode() {
    private val startPose = Pose2d(9.0, 111.0, Math.toRadians(-90.0))
    private val scorePose = Pose2d(16.0, 128.0, Math.toRadians(-45.0))

    val drive: MecanumDrive by onInit {
        MecanumDrive(hardwareMap, startPose)
    }
    val driveCommand: Command by onInit {
        drive.commandBuilder(startPose)
            .splineTo(scorePose.position, scorePose.heading)
            .build()
    }

    override fun onStartButtonPressed() {
        driveCommand.schedule()
    }
}
