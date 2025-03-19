package org.firstinspires.ftc.teamcode.opmodes.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareRobot;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

public class BasicTeleopForTSPMO extends LinearOpMode {
    
//dude ari, this code isnt optimal. you should look at the teamcode for this season and see how teleop is structured.
    HardwareRobot robot;
    DriveSubsystem drive;

    @Override
    public void runOpMode () throws InterruptedException {
        robot = new HardwareRobot(hardwareMap);
        drive = new DriveSubsystem(
                robot.rightFront,
                robot.rightBack,
                robot.leftFront,
                robot.leftBack
        );
        waitForStart();
        while (opModeIsActive()) {
            boolean claw = true, toggleClaw = false;
            boolean elbow = true, toggleElbow = false;
            if (gamepad1.square && !toggleClaw) {
                toggleClaw = true;
                claw = !claw;
            }
            if (!gamepad1.square) toggleClaw = false;
            if (claw) {
                //alt pos
            }
            else {
                //default
            }
            if (gamepad1.circle && !toggleElbow) {
                toggleElbow = true;
                elbow = !elbow;
            }
            if (elbow) {
                //changed pos
            }
            else {
                //default
            }
            double speed = 0.5;
	        double strafe = gamepad1.left_stick_x;
            double forward = -gamepad1.right_stick_y;
            double turn = gamepad1.right_stick_x;
            drive.driveRobotCentric(strafe * speed,forward * speed,turn * speed);
        }
    }
}

