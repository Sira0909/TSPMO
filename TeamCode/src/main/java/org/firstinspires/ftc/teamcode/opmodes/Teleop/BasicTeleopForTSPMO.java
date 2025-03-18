package org.firstinspires.ftc.teamcode.opmodes.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareRobot;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

public class BasicTeleopForTSPMO extends LinearOpMode {
    

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
	        drivecommands();
		    armcommands();
		    letterbuttons();
        }
    }

    public void drivecommands(){
            double speed = 1;
            double strafe = gamepad1.left_stick_x;
            double forward = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            drive.driveRobotCentric(strafe * speed, forward * speed, turn * speed);
    }
    
    public void armcommands(){
	double speed = 1;
	double armup = -gamepad1.right_stick_y;
	boolean armopen= gamepad1.left_bumper;
	boolean armclose=gamepad1.right_bumper;
	//move claw;

    }

    public void letterbuttons(){
	boolean cross = gamepad1.cross;
	boolean circle = gamepad1.circle;
	boolean square = gamepad1.square;
	boolean triangle = gamepad1.triangle;
	if(cross){
	    //macro to allign claw to backboard?
	}
	if(circle){
	    //?
	}
	if(square){
	    //?
	}
	if(triangle){
	    //launch drone?
	}
    }
    

}
