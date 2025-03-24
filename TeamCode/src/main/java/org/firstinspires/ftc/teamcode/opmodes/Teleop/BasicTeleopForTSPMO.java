package org.firstinspires.ftc.teamcode.opmodes.Teleop;

import android.graphics.Canvas;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.HardwareRobot;
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
        robot.inDep.setClawPos(clawClosed); //test servo positions once accessible
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
                .setCameraResolution(new Size(640,480)) // place holder values ask for real size
                .build();
        waitForStart();
        while (!isStopRequested() && opModeIsActive()) {
            drivecommands();
            armcommands();
            letterbuttons();
            apriltagdetect(tagProcessor);
        }


    }

    public void drivecommands() {
        double speed = 1;
        double strafe = gamepad1.left_stick_x;
        double forward = -gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;
        robot.drive.driveRobotCentric(strafe * speed, forward * speed, turn * speed);
    }

    public void armcommands(){
        double speed = 1;
        double armup = -gamepad1.right_stick_y;
        boolean armopen= gamepad1.left_bumper;
        boolean armclose=gamepad1.right_bumper;
        if(armopen){

        }
        //move claw;

    }

    public void letterbuttons() {
        boolean cross = gamepad1.cross;
        boolean square = gamepad1.square;
        boolean triangle = gamepad1.triangle;
        if (cross) {
            //macro to allign claw to backboard?
        }
        if (square) {
            //?
        }
        if (triangle) {
            //launch drone?
        }
    }
//method for detection
    public void apriltagdetect(AprilTagProcessor tagProcessor) {
        if (gamepad1.circle) {
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



