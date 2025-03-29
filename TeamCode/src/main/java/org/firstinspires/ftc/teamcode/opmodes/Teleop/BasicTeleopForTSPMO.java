package org.firstinspires.ftc.teamcode.opmodes.Teleop;

import android.graphics.Canvas;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

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

public class BasicTeleopForTSPMO extends LinearOpMode {

    //  ======    |\\    ||  ======  =========
    //    ||      ||\\   ||    ||       ||
    //    ||      || \\  ||    ||       ||
    //    ||      ||  \\ ||    ||       ||
    //    ||      ||   \\||    ||       ||
    //  ======    ||    \\|  ======     ||
    public RobotSystem robot;

    boolean toggleClaw = false;
    boolean claw = true;
    private final RobotConstants ROBOTCONSTANTS = new RobotConstants();


    @Override
    public void runOpMode() throws InterruptedException {
        this.robot = new RobotSystem(hardwareMap, this);
        robot.inDep.closeClaw();
        robot.inDep.setElbowPos(0);//test servo positions once accessible
        //apriltag intialization
        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();
        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480)) // place holder values ask for real size
                .build();


        //  /=====    ========        /\       |====\    ========
        // |             ||          //\\      |     |      ||
        //  \====\       ||         //  \\     |====/       ||
        //        |      ||        //====\\    |  \\        ||
        //  =====/       ||       //      \\   |   \\       ||

        waitForStart();
        while (!isStopRequested() && opModeIsActive()) {
            driveCommands();
            driveToTag(tagProcessor,1, 100, 60); //replace ofc w desired
            liftCommands();
            letterbuttons();
        }
    }


    //  ||       ========   |======  ========
    //  ||          ||      ||          ||
    //  ||          ||      |=====      ||
    //  ||          ||      ||          ||
    //  ======   ========   ||          ||
    public void liftCommands() {
        double triggerPower = (gamepad1.left_trigger - gamepad1.right_trigger);
        robot.inDep.setLiftPos(triggerPower);
    }

    public void driveCommands() {
        double speed = 1;
        double strafe = gamepad1.left_stick_x;
        double forward = -gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;
        robot.drive.driveRobotCentric(strafe * speed, forward * speed, turn * speed);
    }

    public void letterbuttons() {
        double elbowpower = gamepad1.right_stick_y;
        boolean liftUpMacroRunning = false;
        boolean liftDownMacroRunning = false;
        boolean toggleLiftUpMacro = false;
        boolean toggleLiftDownMacro = false;
        double targetPos = 0;
        robot.inDep.setElbowPos(elbowpower);
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
        if (gamepad1.cross && !toggleLiftUpMacro) {
            toggleLiftUpMacro = true;
            liftUpMacroRunning = true;
            toggleLiftDownMacro = true;
        }
        if (liftUpMacroRunning) {
            liftDownMacroRunning = false;
            targetPos = 1350;
        }
        if (gamepad1.triangle && !toggleLiftDownMacro) {
            toggleLiftDownMacro = true;
            liftDownMacroRunning = true;
            toggleLiftUpMacro = true;
        }
        if (liftDownMacroRunning) {
            liftUpMacroRunning = false;
            targetPos = -10;
        }

    }

    //method for detection
    public void driveToTag(AprilTagProcessor tagProcessor, int tagid, int tagFieldX, int tagFieldY) {
        if (gamepad1.circle) { // Note: this will only move towards board while circle is held
            ArrayList<AprilTagDetection> detections = tagProcessor.getDetections();
            if (!detections.isEmpty()) {
                for (AprilTagDetection tag : detections) {
                    double robotFieldX = tag.robotPose.getPosition().x;
                    double robotFieldY = tag.robotPose.getPosition().y;
                    if (tag.id == tagid) { // Blue alliance tags
                        double kP = 0.1;
                        double kD = 0.01; // Tune these obv

                        double errorX = tagFieldX - robotFieldX;
                        double errorY = tagFieldY - robotFieldY;
                        double errorAngle = tag.ftcPose.bearing;
                        double previousErrorX = 0;
                        double previousErrorY = 0;
                        double previousErrorAngle = 0;
                        ElapsedTime timer = new ElapsedTime();
                        while (Math.hypot(errorX, errorY) > 0.5) {  // 0.5 is an acceptable error threshold
                            double deltaTime = timer.seconds();
                            timer.reset();

                            // PD Control
                            double derivativeX = (errorX - previousErrorX) / deltaTime;
                            double derivativeY = (errorY - previousErrorY) / deltaTime;
                            double derivativeAngle = (errorAngle - previousErrorAngle) /deltaTime;

                            double powerX = kP * errorX + kD * derivativeX;
                            double powerY = kP * errorY + kD * derivativeY;
                            double powerTurn = kP * errorAngle + kD * derivativeAngle;

                            robot.drive.driveRobotCentric(powerX, powerY, powerTurn);

                            previousErrorX = errorX;
                            previousErrorY = errorY;
                            previousErrorAngle = errorAngle;

                            errorX = tagFieldX - robotFieldX;
                            errorY = tagFieldY - robotFieldY;
                            errorAngle = tag.ftcPose.bearing;
                        }
                    }
                }
            }
        }
    }
}