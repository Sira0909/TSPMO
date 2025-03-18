package org.firstinspires.ftc.teamcode.opmodes.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareRobot;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

public class BasicTeleopForTSPMO extends LinearOpMode {
    @Override
    public void runOpMode () throws InterruptedException {
        HardwareRobot robot = new HardwareRobot(hardwareMap);
        DriveSubsystem drive = new DriveSubsystem(
                robot.rightFront,
                robot.rightBack,
                robot.leftFront,
                robot.leftBack
        );
        waitForStart();
        while (opModeIsActive()) {
            double strafe = gamepad1.left_stick_x;
            double forward = -gamepad1.right_stick_y;
            double turn = gamepad1.right_stick_x;
            drive.driveRobotCentric(strafe,forward,turn);
            if (gamepad1.circle) {

            }
            if (gamepad1.triangle) {

            }
            if (gamepad1.cross) {

            }
            if (gamepad1.square) {

            }
        }
    }
}
