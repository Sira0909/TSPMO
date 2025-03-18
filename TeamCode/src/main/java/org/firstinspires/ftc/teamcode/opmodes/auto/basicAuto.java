package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareRobot;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;


@Autonomous(name="good auto (sample)")
public static void basicAuto extends LinearOpMode{
	public HardwareRobot robot = new HardwareRobot(hardwareMap);
	public void runOpMode() throws InterruptedException {
		DriveSubsystem drive = new DriveSubsystem(
			robot.rightFront,
			robot.rightBack,
			robot.leftFront,
			robot.leftBack
		);
		waitForStart();
		while (opModeIsActive()) {

		}
		}
	}
