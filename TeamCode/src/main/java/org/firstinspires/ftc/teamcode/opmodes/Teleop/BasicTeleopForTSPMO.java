package org.firstinspires.ftc.teamcode.opmodes.Teleop;

import android.graphics.Canvas;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.HardwareRobot;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.RobotSystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Mat;
import org.openftc.apriltag.AprilTagPose;

import java.util.ArrayList;
import java.util.List;

//adding documentation and fixing apriltag stuff - viir 3/20
public class BasicTeleopForTSPMO extends LinearOpMode {

//creates robot as object of compiled robotsystem class w all subsystems
    public RobotSystem robot;

    boolean toggleClaw = false;
    boolean claw = true;
    boolean toggleElbow = false;

    private final RobotConstants ROBOTCONSTANTS=new RobotConstants();

    @Override
    public void runOpMode () throws InterruptedException {
        //define robot object
        this.robot = new RobotSystem(hardwareMap, this);
        //ofc, tweak all of these
        double clawClosed = 0;
        double clawOpen = 1;
        //assuming elbow has infinite positions
        //ari also wanted this
        double elbowPos = gamepad1.right_stick_y;
        //setting pos for claw and elbow
        robot.inDep.closeClaw(); //test servo positions once accessible
        robot.inDep.setElbowPos(elbowPos); //test servo pos as well


        //apriltag intialization
        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder() //creates object of processor class for detection\
                //calling set up methods - drawing and mapping out possible predicted tags
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                //build intializes all of these
                .build();
        VisionPortal visionPortal = new VisionPortal.Builder() //actual cv program built in
                //adding processor inside of camera display to detect tags
                .addProcessor(tagProcessor)
                //adding cam to do the actual detection
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                //setting cam positions
                .setCameraResolution(new Size(640, 480)) // place holder values ask for real size
                .build();



        //  /====    ======        /\       |==\    ======
        // |           ||         /  \      |   |     ||
        //  \===\      ||        /    \     |==/      ||
        //       |     ||       /======\    | \       ||
        //  ====/      ||      /        \   |  \      ||

        waitForStart();
        while (!isStopRequested() && opModeIsActive()) {
            driveCommands();
            elbowCommands();
            aprilTagDetect(tagProcessor);
            liftCommands();
            letterbuttons();
        }
    }
    public void liftCommands() {

        int triggerPower = (int)(gamepad1.left_trigger - gamepad1.right_trigger);
        int LiftPos = triggerPower/ROBOTCONSTANTS.TRIGGERMAX;
        if(gamepad1.cross) {//down
            robot.inDep.LiftDown();
        }
        else if(gamepad1.triangle) {//up
            robot.inDep.LiftUp();
        }
        else {
            robot.inDep.SetLiftDir(LiftPos);
        }
    }
    public void elbowCommands() {

        double triggerPower = gamepad1.right_stick_y;
        double elbow = triggerPower/ROBOTCONSTANTS.ELBOWCONST;
        robot.inDep.adjustElbowPos(elbow);
    }
    public void driveCommands() {
        double speed = 1;
        double strafe = gamepad1.left_stick_x;
        double forward = -gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;
        robot.drive.driveRobotCentric(strafe * speed, forward * speed, turn * speed);
    }
    
    public void letterbuttons(){

            if (gamepad1.triangle) {
                //launch drone?
            }
            if (gamepad1.circle && !toggleClaw) {
                toggleClaw = true;
                claw = !claw;
            }
            if (!gamepad1.circle) {
                toggleClaw = false;
            }
            if (claw) {
                robot.inDep.closeClaw();
            } else {
                robot.inDep.openClaw();
            }
            if(gamepad1.cross){
                //see lift
            }
            if(gamepad1.triangle) {
                //see lift
            }

    }

//method for detection
    public void aprilTagDetect(AprilTagProcessor tagProcessor) {
        if (gamepad1.square) {
            List<AprilTagDetection> detections = tagProcessor.getDetections();
            if (!detections.isEmpty()) {
                telemetry.addLine("AprilTags Detected:");
                for (AprilTagDetection tag : detections) {
                    telemetry.addData("Tag ID", tag.id);
                    telemetry.addData("x", tag.ftcPose.x);
                    telemetry.addData("y", tag.ftcPose.y);
                    telemetry.addData("z", tag.ftcPose.z);
                }
            } else {
                telemetry.addLine("No Tag Detected.");
            }
            telemetry.update();
        }
    }
}



