package org.firstinspire.ftc.teamcode.Cindysmomusethisstuff;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspire.ftc.teamcode.RobotConstants;
import org.firstinspire.ftc.teamcode.RobotSystem;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
//TODO: add elbow macros w separate controller
@TeleOp (name = "TOwithApriltags")
public class TeleOpWithAprilTags extends LinearOpMode {
    public RobotSystem robot;
    public double rotationPos;
    public double clawPos;
    public int encoderposs;
    public ElapsedTime runtime = new ElapsedTime();
    public double lastError = 0;
    public double lastError1 = 0;
    public double lastError2 = 0;
    public double lastTime = 0;
    public boolean macroTagRunning = false;
    public boolean drivecompleted = false;
    public double elbowp;
    private boolean clawOpen = true;
    private boolean wasXPressedLastLoop = false;
    private boolean rotdown = true;
    private boolean wasSqpressedlastloop = false;
    public double speed = 0.45;
    public AprilTagDetection lastTagDetected;
    public AprilTagProcessor tagProcessor;
    public VisionPortal visionPortal;
    public boolean wascircelpressedlastloop = false;
    public double clamp(double value, double maxMagnitude) {
        return Math.copySign(Math.min(Math.abs(value), maxMagnitude), value);
    }
    public void reset() {
        lastError = 0;
        lastError1 = 0;
        lastError2 = 0;
        lastTime = 0;
    }
    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        this.robot = new RobotSystem(hardwareMap, this);
        clawPos = RobotConstants.CLOSECLAW;
        robot.inDep.setClawPosition(clawPos);
        rotationPos = RobotConstants.CLAWROTATIONBACKBOARD;
        robot.inDep.setRotationPosition(rotationPos);
        this.tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawCubeProjection(true)
                .build();
        this.visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(robot.hardwareRobot.webcamName)
                .setAutoStopLiveView(false)
                .enableLiveView(true)
                .setAutoStartStreamOnBuild(true)
                .setLiveViewContainerId(cameraMonitorViewId)
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .build();
        visionPortal.setProcessorEnabled(tagProcessor, true);
        waitForStart();
        while (opModeIsActive()) {
            boolean circlePressd = gamepad1.circle;
            encoderposs = robot.inDep.getEncoder(encoderposs);
            elbowp = gamepad1.right_stick_y * 0.5;
            double strafe = -gamepad1.left_stick_y;
            double forward = -gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;
            if (Math.abs(turn) > 0) {
                robot.hardwareRobot.changeInversions();
            }
            if (!macroTagRunning) {
                boolean proximity = xInchRadius(lastTagDetected, 20);
                if (proximity) {
                    speed += -0.15;
                }
                robot.drive.driveRobotCentricPowers(strafe * speed, forward * speed, turn * speed);
            }
            boolean isPressed = gamepad1.dpad_right;
            if (isPressed && !wasXPressedLastLoop) {
                clawOpen = !clawOpen;
                if (clawOpen) {
                    clawPos = RobotConstants.OPENCLAW;
                } else {
                    clawPos = RobotConstants.CLOSECLAW;
                }
            }
            detectTags();
            boolean ispressed = gamepad1.square;
            if (ispressed && !wasSqpressedlastloop) {
                rotdown = !rotdown;
                if (rotdown) {
                    rotationPos = RobotConstants.CLAWROTATIONPIKINGUP;
                } else {
                    rotationPos = RobotConstants.CLAWROTATIONBACKBOARD;
                }
            }
            /// not working :(
            if (circlePressd && !wascircelpressedlastloop) {
                reset();
            }
            if (circlePressd) {
                if (!drivecompleted) {
                    macroTagRunning = true;
                }
            }
            if (macroTagRunning) {
                driveToTag(lastTagDetected); //change ts
                if (drivecompleted) {
                    drivecompleted = false;
                    macroTagRunning = false;
                }
            }
            robot.inDep.setClawPosition(clawPos);
            robot.inDep.setRotationPosition(rotationPos);
            robot.inDep.setElbowPosition(elbowp);
            telemetry.addData("Strafe: ", strafe);
            telemetry.addData("Turn: ", turn);
            telemetry.addData("Forward: ", forward);
            telemetry.addData("Rotation Position: ", rotationPos);
            telemetry.addData("Claw Position: ", clawPos);
            telemetry.addData("Encoder Position: ", encoderposs);
            if (lastTagDetected != null) {
                telemetry.addData("Proximity to closest tag:", xInchRadius(lastTagDetected, 20));
            }
            telemetry.addData("Time:", runtime.time());
            telemetry.addData("MacroTagRunning:", macroTagRunning);
            telemetry.addData("Drive completed:", drivecompleted);
            telemetry.update();
            wasXPressedLastLoop = isPressed;
            wasSqpressedlastloop = ispressed;
            wascircelpressedlastloop = circlePressd;
        }
    }
    /// working
    public void detectTags() {
        ArrayList<AprilTagDetection> detections = tagProcessor.getDetections();
        if(detections != null && !detections.isEmpty()) {
            for(AprilTagDetection tag : detections) {
                telemetry.addLine("AprilTag Detected.");
                telemetry.addData("ID", tag.id);
                telemetry.addData("X (Sideways offset)", tag.ftcPose.x);
                telemetry.addData("Y (Forward/Back Offset)", tag.ftcPose.y);
                telemetry.addData("Z", tag.ftcPose.z);
                telemetry.addData("Bearing", tag.ftcPose.bearing);
                telemetry.addData("Yaw", tag.ftcPose.yaw);
                telemetry.addData("Range: ", tag.ftcPose.range);
                lastTagDetected = tag;
                break;
            }
        } else {
            lastTagDetected = null; // clear old tag when none detected
        }
    }
    /// unfortunately broken
    public void driveToTag(AprilTagDetection tagg) {
        if (tagg == null) {
            return;
        }
        telemetry.addData("Driving to tag of ID: ", tagg.id);
        double errorX =  tagg.ftcPose.x;
        double errorY =  tagg.ftcPose.y;
        double erroryaw = tagg.ftcPose.yaw;
        PD(errorX, errorY, erroryaw);
    }
    //inital deltatime is supposed to be zero
    //also initial error diff is supposed to be zero
    /// unfortunately broken
    public void PD(double errorX, double errorY, double errorYaw) {
        telemetry.addLine("PD in effect");
        double time = runtime.seconds();
        double deltaTime = time - lastTime;

        double kP = 0.03;
        double kD = 0.005;
        if (deltaTime == 0) {
            deltaTime = 0.00001;
        }
        double derivativeX = (errorX - lastError) / deltaTime;
        double derivativeY = (errorY - lastError1) / deltaTime;
        double derivativeYaw = (errorYaw - lastError2) / deltaTime;
        //this condition resolves both of the cases above
        if (lastTime == 0) {
            derivativeX = 0;
            derivativeY = 0;
            derivativeYaw = 0;
        }
        double powerX = kP * errorX + kD * derivativeX;
        double powerY = kP * errorY + kD * derivativeY;
        double powerYaw = kP * errorYaw + kD * derivativeYaw;
        if (powerX > 0.5) {
            powerX = 0.5;
        }
        else if (powerY > 0.5) {
            powerY = 0.5;
        }
        else if (powerYaw > 0.5) {
            powerYaw = 0.5;
        }
        robot.drive.driveRobotCentricPowers(powerX, powerY, powerYaw);
        if (Math.abs(errorY) <= 18 && Math.abs(errorX) <= 0.1 && Math.abs(errorYaw) <= 0.4) {
            drivecompleted = true;
        }
        telemetry.addData("Error X:", errorX);
        telemetry.addData("Error Y: ", errorY);
        telemetry.addData("Error Yaw: ", errorYaw);
        lastError = errorX;
        lastError1 = errorY;
        lastError2 = errorYaw;
        lastTime = time;
    }
    /// working: used to shift speed
    public boolean xInchRadius(AprilTagDetection taggg, double radius) {
        if (lastTagDetected == null) {
            return false;
        }
        else {
            return taggg.ftcPose.range <= radius;
        }
    }
}
