package org.firstinspires.ftc.teamcode.opmodes.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareRobot;
import org.firstinspires.ftc.teamcode.RobotSystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.indepSubsystem;

public class BasicTeleopForTSPMO extends LinearOpMode {
    

    public RobotSystem robot;

    @Override
    public void runOpMode () throws InterruptedException {
        this.robot = new RobotSystem(hardwareMap,this);
        waitForStart();
        boolean claw = true, toggleClaw = false;
        boolean elbow = true, toggleElbow = false;
        while (opModeIsActive()) {
            driveCommands();
            elbowCommands();
            letterButtons();
        }
    }

    public void driveCommands(){
        double speed = 1;
        double strafe = gamepad1.left_stick_x;
        double forward = -gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;
        robot.drive.driveRobotCentric(strafe * speed, forward * speed, turn * speed);
    }
    
    public void elbowCommands(){
        double speed = 1;
        double armUp = -gamepad1.right_stick_y;

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
            robot.inDep.setClawPos(0);
        }
        if(square){
            robot.inDep.setElbowPos(0);
            //?
        }
        if(triangle){
            //launch drone?
        }
    }
}

