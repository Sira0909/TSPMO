package org.firstinspires.ftc.teamcode.opmodes.Teleop;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.RobotSystem;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;

//viirs update 3/31 - adding liftup and liftdown macros, closely follows how they did it in regular season code
//also how are we gonna use xinchradius?
public class BasicTeleopForTSPMO_Blue extends LinearOpMode {

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
    private double PEBEARING = 0;



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
            letterButtons();
            tags(tagProcessor);
        }

    }
    //  ||       ========   |======  ========
    //  ||          ||      ||          ||
    //  ||          ||      |=====      ||
    //  ||          ||      ||          ||
    //  ======   ========   ||          ||
    public void liftCommands() {
        double targetPos = 0;
        boolean LURunning = false;
        boolean LDRunning = false;
        boolean toggleLU = false;
        boolean toggleLD = false;
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
            if (Math.abs(error) < 50) {
                LURunning = false;
                LDRunning = false;
            }
            //above stops the macro if error gets too small (lift is close to target pos)
        } else {
            robot.inDep.setLiftPos(triggerPower);
        }
    }

    public void driveCommands() {
        double speed = 1;
        double strafe = gamepad1.left_stick_x;
        double forward = -gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;
        robot.drive.driveRobotCentric(strafe * speed, forward * speed, turn * speed);
    }

    public void letterButtons() {
        double elbowpower = gamepad1.right_stick_y;
        robot.inDep.setElbowPos(elbowpower);
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

        double errorX = target.ftcPose.x;
        double errorY = target.ftcPose.y - 1; // subtract 1 bc we dont wanna crash into the tag
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

        PEX = errorX; PEY = errorY; PEBEARING = errorAngle;
    }

    private boolean xInchRadius(AprilTagProcessor tagProcessor, int num) {
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
