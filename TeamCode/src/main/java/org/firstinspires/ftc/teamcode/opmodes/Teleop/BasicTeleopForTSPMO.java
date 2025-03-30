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

//adding documentation and fixing apriltag stuff - viir 3/20
public class BasicTeleopForTSPMO extends LinearOpMode {

    //creates robot as object of compiled robotsystem class w all subsystems
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


    private double PEX = 0;
    private double PEY = 0;
    private double PEYAW = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        //define robot object
        this.robot = new RobotSystem(hardwareMap, this);
        //assuming elbow has infinite positions
        //ari also wanted this
        //setting pos for claw and elbow
        robot.inDep.closeClaw();
        robot.inDep.setElbowPos(0);//test servo positions once accessible
        // test servo pos as well


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


        //  /=====    ========        /\       |====\    ========
        // |             ||          //\\      |     |      ||
        //  \====\       ||         //  \\     |====/       ||
        //        |      ||        //====\\    |  \\        ||
        //  =====/       ||       //      \\   |   \\       ||

        waitForStart();
        while (!isStopRequested() && opModeIsActive()) {
            driveCommands();
            liftCommands();
            letterbuttons();
            driveToTag(tagProcessor, 2);
        }
    }


    //  ||       ========   |======  ========
    //  ||          ||      ||          ||
    //  ||          ||      |=====      ||
    //  ||          ||      ||          ||
    //  ======   ========   ||          ||
    public void liftCommands() {
        double triggerPower = (gamepad1.left_trigger - gamepad1.right_trigger);
        robot.inDep.setElbowPos(triggerPower);
        //make lift up and lift down macros if needed - viir has them already
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
        if (gamepad1.cross) {
            //lift up macro
        }
        if (gamepad1.triangle) {
            //lift down macro
        }

    }

    public void driveToTag(AprilTagProcessor tagProcessor, int tagid) {
        if (gamepad1.circle) { // Note: this will only move towards board while circle is held
            ArrayList<AprilTagDetection> detections = tagProcessor.getDetections();
            AprilTagDetection target = null;
            if (!detections.isEmpty()) {
                for (AprilTagDetection tag : detections) {
                    telemetry.addLine(String.format("XYZ %6.2f %6.2f %6.2f", tag.ftcPose.x, tag.ftcPose.y, tag.ftcPose.z));
                    if (tag.id == tagid) { // Blue alliance tags
                        target = tag;
                        break;
                    }
                }
            }
            if (target != null) {
                PDcontroller(target, 100, 60); //exammple
            }
        }
    }

    public void PDcontroller(AprilTagDetection target, double targetFieldX, double targetFieldY) {
        double kP = 0.1;
        double kD = 0.01; // Tune these obv
        double robotFieldX = target.robotPose.getPosition().x;
        double robotFieldY = target.robotPose.getPosition().y;
        double errorX = targetFieldX - robotFieldX;
        double errorY = targetFieldY - robotFieldY;
        double errorAngle = target.ftcPose.bearing;
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
            double derivativeAngle = (errorAngle - previousErrorAngle) / deltaTime;

            double powerX = kP * errorX + kD * derivativeX;
            double powerY = kP * errorY + kD * derivativeY;
            double powerTurn = kP * errorAngle + kD * derivativeAngle;

            robot.drive.driveRobotCentric(powerX, powerY, powerTurn);  // Replace with your robot’s drive method

            previousErrorX = errorX;
            previousErrorY = errorY;
            previousErrorAngle = errorAngle;

            errorX = targetFieldX - robotFieldX;
            errorY = targetFieldY - robotFieldY;
            errorAngle = target.ftcPose.bearing;
        }
    }
        private boolean xInchRadius (AprilTagProcessor tagProcessor,int num){
            int xInches = num;
            ArrayList<AprilTagDetection> detections = tagProcessor.getDetections();

            for (int i = 0; i < detections.size(); i++) {
                AprilTagDetection tag = detections.get(i);
                if (tag.id == 1 || tag.id == 2 || tag.id == 3) {
                    double x = tag.ftcPose.x;
                    double y = tag.ftcPose.y;
                    double distance = Math.sqrt((x * x) + (y * y));
                    if (distance <= xInches) {
                        return true;
                    }
                }
            }
            return false;
        }
    }




