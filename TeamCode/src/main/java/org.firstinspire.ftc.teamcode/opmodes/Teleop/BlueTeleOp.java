package org.firstinspire.ftc.teamcode.opmodes.Teleop;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspire.ftc.teamcode.RobotConstants;
import org.firstinspire.ftc.teamcode.RobotSystem;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;

public class BlueTeleOp extends LinearOpMode {

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
    double speed = 1;
    boolean LURunning = false;
    boolean LDRunning = false;
    boolean toggleLU = false;
    boolean toggleLD = false;
    private final RobotConstants ROBOTCONSTANTS = new RobotConstants();


    private double PEX = 0;
    private double PEY = 0;
    private double PEBEARING = 0;
    public AprilTagDetection currentTag = null;



    @Override
    public void runOpMode() throws InterruptedException {
        this.robot = new RobotSystem(hardwareMap, this);
        robot.inDep.closeClaw();
        robot.inDep.setElbowPos(0);



        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();
        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
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
            letterButtons();
            detectTags(tagProcessor);
            if (gamepad1.right_bumper && currentTag.id == 3) {
                tags(tagProcessor,3);
            }
            xInchRadius(tagProcessor, 3);
            //or smth add more logic idk
        }

    }
    //  ||       ========   |======  ========
    //  ||          ||      ||          ||
    //  ||          ||      |=====      ||
    //  ||          ||      ||          ||
    //  ======   ========   ||          ||
    public void liftCommands() {
        double kP1 = 0.01;
        double liftTargetPosition = 0; //macro target pos to get lift to
        double triggerPower = gamepad1.right_trigger - gamepad1.left_trigger;
        if (gamepad1.dpad_up && !toggleLU) {
            toggleLU = true;
            liftTargetPosition = 1350;
            LURunning = true;
            LDRunning = false;
            robot.inDep.setElbowPos(1);
        }
        if (!gamepad1.dpad_up) {
            toggleLU = false;
        }
        if (gamepad1.dpad_down && !toggleLD) {
            liftTargetPosition = -10; //target pos aka up lift position
            LURunning = false;
            toggleLD = true;
            LDRunning = true;
            robot.inDep.setElbowPos(0);
        }
        if (!gamepad1.dpad_down) {
            toggleLD = false;
        }
        boolean triggerPressed = gamepad1.left_trigger != 0 || gamepad1.right_trigger != 0;
        if (triggerPressed) {
            LURunning = false;
            LDRunning = false;
        }
        if (LURunning || LDRunning) {
            double liftPosition = robot.inDep.getLiftPos();
            double error = liftTargetPosition - liftPosition;
            double u_t = kP1 * error;
            robot.inDep.setLiftPos(u_t);
            if (Math.abs(error) < 40) {
                LURunning = false;
                LDRunning = false;
            }
        } else {
            robot.inDep.setLiftPos(triggerPower);
        }
    }

    public void driveCommands() {
        double strafe = gamepad1.left_stick_x;
        double forward = -gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;
        robot.drive.driveRobotCentric(strafe * speed, forward * speed, turn * speed);
    }

    public void letterButtons() {
        double elbowpower = gamepad1.right_stick_y;
        robot.inDep.setElbowPos(elbowpower);
        if (gamepad1.dpad_right && !toggleClaw) {
            claw = !claw;
            toggleClaw = true;
        }
        if (!gamepad1.dpad_right) {
            toggleClaw = false;
        }
        if (claw) {
            robot.inDep.closeClaw();
        }
        else {
            robot.inDep.openClaw();
        }
    }
    public void detectTags(AprilTagProcessor tagProcessor) {
        ArrayList<AprilTagDetection> detections = tagProcessor.getDetections();
        if (detections != null) {
            for (AprilTagDetection tag: detections) {
                //display info about all tags in view
                telemetry.addData("X: ", tag.ftcPose.x);
                telemetry.addData("Y: ", tag.ftcPose.y);
                telemetry.addData("Bearing/Angle Error: ", tag.ftcPose.bearing);
                telemetry.addData("Z: ", tag.ftcPose.z);
                telemetry.addData("Robot X: ", tag.robotPose.getPosition().x);
                telemetry.addData("Robot Y: ", tag.robotPose.getPosition().y);
                telemetry.addData("Robot Z: ", tag.robotPose.getPosition().z);
                currentTag = tag;
            }
        }
    }
    public void tags(AprilTagProcessor tagProcessor, int targetTag){
        if (gamepad1.circle) { // Note: this will only move towards board while circle is held
            ArrayList<AprilTagDetection> detections = tagProcessor.getDetections();
            AprilTagDetection target = null;
            if (!detections.isEmpty() && opModeIsActive()) {
                for (AprilTagDetection tag : detections) {
                    if (tag.id == targetTag) {
                        target = tag;
                        break;
                    }
                }
            }
            if (target != null) {
                PDcontroller(target, 100, 100);
            }
        }
    }

    public void PDcontroller (AprilTagDetection target, double tagX, double tagY){
        double kP = 0.1; double kD = 0.01; // Tune these obv

        double errorX = tagX - target.robotPose.getPosition().x;
        double errorY = tagY - target.robotPose.getPosition().y;
        double errorAngle = target.ftcPose.bearing;

        double derivativeX = errorX -PEX ;
        double derivativeY = errorY - PEY;
        double derivativeAngle = errorAngle - PEBEARING;

        double strafePower = kP * errorX + kD * derivativeX;
        double forwardPower = kP * errorY + kD * derivativeY;
        double turnPower = kP * errorAngle + kD * derivativeAngle;

        //CONVERTING STRafe FORWARD AND TUIRN POWER INTO -1 to 1 range
        strafePower = Math.max(-1, Math.min(1, strafePower));
        forwardPower = Math.max(-1, Math.min(1, forwardPower));
        turnPower = Math.max(-1, Math.min(1, turnPower));

        robot.drive.driveRobotCentric(strafePower, forwardPower, turnPower);
        errorX = tagX - target.robotPose.getPosition().x;
        errorY = tagY - target.robotPose.getPosition().y;
        errorAngle = target.ftcPose.bearing;
        PEX = errorX; PEY = errorY; PEBEARING = errorAngle;
    }
    //xinch will reduce strain on driver so we dont mess up at stemtastic ig
    //tune xinches to how much we want to slow down
    public void xInchRadius(AprilTagProcessor tagProcessor, int xInches) {
        ArrayList<AprilTagDetection> detections = tagProcessor.getDetections();
            for (AprilTagDetection tag: detections) {
                    double x = tag.ftcPose.x;
                    double y = tag.ftcPose.y;
                    double distance = Math.sqrt((x * x) + (y * y));
                    if (distance <= xInches) {
                        speed += 0.4;
            }
        }
    }
}
