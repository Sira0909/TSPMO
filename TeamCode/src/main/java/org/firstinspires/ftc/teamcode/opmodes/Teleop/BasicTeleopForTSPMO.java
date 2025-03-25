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

import java.lang.reflect.Array;
import java.util.ArrayList;

public class BasicTeleopForTSPMO extends LinearOpMode {
    

    HardwareRobot robot;
    DriveSubsystem drive;

    private static double PEX = 0;
    private static double PEY = 0;
    private static double PEYAW = 0;

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
                //.setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480)) // place holder values ask for real size
                .enableLiveView(true)
                .build();

        waitForStart();

        AprilTagDetection target;
        while (!isStopRequested() && opModeIsActive()) {
            drivecommands();
            armcommands();
            letterbuttons();


            }


        }

        public void drivecommands () {
            double speed = 1;
            double strafe = gamepad1.left_stick_x;
            double forward = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            drive.driveRobotCentric(strafe * speed, forward * speed, turn * speed);
        }

        public void armcommands () {
            double speed = 1;
            double armup = -gamepad1.right_stick_y;
            boolean armopen = gamepad1.left_bumper;
            boolean armclose = gamepad1.right_bumper;
            //move claw;

        }

        public void letterbuttons () {
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

        public void tags(AprilTagProcessor tagProcessor){
            if (gamepad1.circle) { // Note: this will only move towards board while circle is held
                ArrayList<AprilTagDetection> detections = tagProcessor.getDetections();
                AprilTagDetection target = null;
                if (!detections.isEmpty()) {
                    for (AprilTagDetection tag : detections) {
                        telemetry.addLine(String.format("XYZ %6.2f %6.2f %6.2f", tag.ftcPose.x, tag.ftcPose.y, tag.ftcPose.z));
                        if (tag.id == 1 || tag.id == 2 || tag.id == 3) { // Blue alliance tags
                            target = tag;
                            break;
                        }
                    }
                }
                if (target != null) {
                    PDcontroller(target);
                }
            }
        }
        public void PDcontroller (AprilTagDetection target){
            double kP = 0.1; double kD = 0.01; // Tune these obv
            double targetRange = 1; //only used if we want to stop at a specific point before the apriltag

            double errorX = target.ftcPose.x;
            double errorY = target.ftcPose.y - targetRange; // subtract 1 bc we dont wanna crash into the tag
            double errorYaw = target.ftcPose.yaw;

            double derivativeX = errorX -PEX ;
            double derivativeY = errorY - PEY;
            double derivativeYaw = errorYaw - PEYAW;

            double strafePower = kP * errorX + kD * derivativeX;
            double forwardPower = kP * errorY + kD * derivativeY;
            double turnPower = kP * errorYaw + kD * derivativeYaw;

            //CONVERTING STRafe FORWARD AND TUIRN POWER INTO -1 to 1 range
            strafePower = Math.max(-1, Math.min(1, strafePower));
            forwardPower = Math.max(-1, Math.min(1, forwardPower));
            turnPower = Math.max(-1, Math.min(1, turnPower));

            drive.driveRobotCentric(strafePower, forwardPower, turnPower);

            PEX = errorX; PEY = errorY; PEYAW = errorYaw;
        }


    }
