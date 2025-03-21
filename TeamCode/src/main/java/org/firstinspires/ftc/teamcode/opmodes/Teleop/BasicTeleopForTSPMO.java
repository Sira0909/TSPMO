package org.firstinspires.ftc.teamcode.opmodes.Teleop;

import android.graphics.Canvas;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.HardwareRobot;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Mat;

import java.util.ArrayList;

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

        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
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

    public void apriltagdetect(AprilTagProcessor tagProcessor) {
        if (gamepad1.circle) {
            if (tagProcessor.getDetections().size() > 0) {
                AprilTagDetection tag = tagProcessor.getDetections().get(0);
                telemetry.addLine("AprilTag Detected!");
                telemetry.addLine("x: " + tag.ftcPose.x);
                telemetry.addLine("y: " + tag.ftcPose.y);
                telemetry.addLine("z: " + tag.ftcPose.z);
            } else {
                telemetry.addLine("No Tag Detected.");
            }
            telemetry.update();
        }
    }


}
