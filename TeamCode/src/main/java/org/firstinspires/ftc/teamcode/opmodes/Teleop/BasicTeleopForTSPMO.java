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
        boolean claw = true, toggleClaw = false;
        boolean elbow = true, toggleElbow = false;
        while (opModeIsActive()) {
	          driveCommands();
            armCommands();
            letterButtons();
        }
    }

    public void driveCommands(){
        double speed = 1;
        double strafe = gamepad1.left_stick_x;
        double forward = -gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;
        drive.driveRobotCentric(strafe * speed, forward * speed, turn * speed);
    }
    
    public void armCommands(){
        double speed = 1;
        double armup = -gamepad1.right_stick_y;
        boolean armopen= gamepad1.left_bumper;
        boolean armclose=gamepad1.right_bumper;
        //move claw;

    }

    public void letterButtons(){
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

