package org.firstinspire.ftc.teamcode.opmodes.Teleop;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspire.ftc.teamcode.HardwareRobot;
import org.firstinspire.ftc.teamcode.RobotSystem;
import org.firstinspire.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.ArrayList;

//TODO: code possible auto and design coordinate system
@TeleOp (name = "Mathnasium TeleOp")
public class MathnasiumTeleOp extends LinearOpMode {
    public RobotSystem robot;
    public ElapsedTime runtime = new ElapsedTime();
    public double rotationPos;
    public double clawPos;
    public int encoderposs;
    private boolean clawOpen = true;
    private boolean wasXPressedLastLoop = false;
    private boolean rotdown = true;
    private boolean wasSqpressedlastloop = false;
    public double elbowpp;
    public double lastError = 0;
    public double elbowp;
    public double lastTime = 0;
    public double speed = 0.4;
    public boolean macroTagRunning = true;
    public boolean isDownMacroRunning = false;
    public boolean isUpMacroRunning = false;
    public boolean isElbowMoveCompleted = false;

    public AprilTagDetection lastDetectedTag;
    public boolean drivecompleted = false;
    AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
            .setDrawAxes(true)
            .setDrawTagID(true)
            .setDrawTagOutline(true)
            .setDrawCubeProjection(true)
            .build();
    VisionPortal visionPortal = new VisionPortal.Builder()
            .addProcessor(tagProcessor)
            .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
            .setCameraResolution(new Size(400,400)) //obv replace
            .setStreamFormat(VisionPortal.StreamFormat.YUY2)
            .enableLiveView(true)
            .setAutoStopLiveView(true)
            .build();
    public void detectTags() {
        ArrayList<AprilTagDetection> detections = tagProcessor.getDetections();
        if(detections != null) {
            telemetry.addLine("AprilTag Detected.");
            for(AprilTagDetection tag : detections) {
                telemetry.addData("ID", tag.id);
                telemetry.addData("X (Sideways offset)", tag.ftcPose.x);
                telemetry.addData("Y (Forward/Back Offset", tag.ftcPose.y);
                telemetry.addData("Z", tag.ftcPose.y);
                telemetry.addData("Bearing", tag.ftcPose);
                telemetry.addData("Yaw", tag.ftcPose.yaw);
                lastDetectedTag = tag;
            }
        }
    }
    public void driveToTag(AprilTagDetection tagg) {
        drivecompleted = false;
        double errorX =  tagg.ftcPose.x;
        double errorY =  tagg.ftcPose.y;
        double erroryaw = tagg.ftcPose.yaw;
        PD(errorX, 0);
        PD(errorY,1);
        PD(erroryaw,2);
    }
    public void elbowMacros(double setpoint) {
        isElbowMoveCompleted = false;
        double errorr = setpoint - encoderposs;
        PD(errorr, 3);
    }
    public void PD (double error, double option) {
        double time = runtime.seconds();
        double kP = 0.02;
        double kD = 0.002;
        double deltaTime = time - lastTime;
        double derivative = (error - lastError) / deltaTime;
        double power = kP * error + kD * derivative;
        if (option == 0) {
            robot.drive.driveRobotCentricPowers(power, 0,0);
            if (Math.abs(error) <= 10) {
                drivecompleted = true;
            }
        }
        else if (option == 1) {
            robot.drive.driveRobotCentricPowers(0,power, 0);
            if (Math.abs(error) <= 20) {
                drivecompleted = true;
            }
        }
        else if (option == 2) {
            robot.drive.driveRobotCentricPowers(0,0,power);
            if (Math.abs(error) <= 20) {
                drivecompleted = true;
            }
        }
        else if (option == 3) {
            elbowp = power;
            if (Math.abs(error) <= 5) {
                isElbowMoveCompleted = true;
            }
        }
        lastError = error;
        lastTime = time;
    }
    public boolean xInchRadius(AprilTagDetection taggg, double radius) {
        boolean check = false;
        if (taggg.ftcPose.range <= radius) {
            check = true;
        }
        return check;
    }
    @Override
    public void runOpMode () throws InterruptedException {
        visionPortal.setActiveCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        visionPortal.setProcessorEnabled(tagProcessor, true);
        this.robot = new RobotSystem(hardwareMap, this);
        clawPos = RobotConstants.CLOSECLAW;
        robot.inDep.setClawPosition(clawPos);
        rotationPos = RobotConstants.CLAWROTATIONBACKBOARD;
        robot.inDep.setRotationPosition(rotationPos);
        waitForStart();
        while (opModeIsActive()) {
            double strafe = -gamepad1.left_stick_x;
            double turn = -gamepad1.right_stick_x;
            double forward = -gamepad1.left_stick_y;
            if (xInchRadius(lastDetectedTag, 4)) {
                speed = 0.2;
            }
            else {
                speed = 0.4;
            }
            robot.drive.driveRobotCentricPowers(strafe * speed, forward * speed, turn * speed);
            encoderposs = robot.inDep.getEncoder(encoderposs);
            if (gamepad1.dpad_left) {
                isUpMacroRunning = true;
            }
            if (!gamepad1.dpad_left) {
                isUpMacroRunning = false;
            }
            if (gamepad1.dpad_down) {
                isElbowMoveCompleted = false;
                isDownMacroRunning = true;
            }
            if (!gamepad1.dpad_down) {
                isElbowMoveCompleted = false;
                isDownMacroRunning = false;
            }
            if (isUpMacroRunning) {
                if (isElbowMoveCompleted) {
                    isUpMacroRunning = false;
                }
                else {
                    elbowMacros(70);
                }
            }
            if (isDownMacroRunning) {
                if (isElbowMoveCompleted) {
                    isDownMacroRunning = false;
                }
                else {
                    elbowMacros(-600);
                }
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
            boolean ispressed = gamepad1.square;
            if (ispressed && !wasSqpressedlastloop) {
                rotdown = !rotdown;
                if (rotdown) {
                    rotationPos = RobotConstants.CLAWROTATIONPIKINGUP;
                } else {
                    rotationPos = RobotConstants.CLAWROTATIONBACKBOARD;
                }
            }
            if (isElbowMoveCompleted) {
                elbowp = -gamepad1.right_stick_y;
            }
            if (gamepad1.circle) {
                macroTagRunning = true;
            }
            if (!gamepad1.circle) {
                macroTagRunning = false;
            }
            if (macroTagRunning) {
                if (drivecompleted) {
                    macroTagRunning = false;
                }
                else {
                    driveToTag(lastDetectedTag); //change ts
                }
            }
            robot.inDep.setElbowPosition(elbowp);
            robot.inDep.setClawPosition(clawPos);
            robot.inDep.setRotationPosition(rotationPos);
            detectTags();
            telemetry.addData("Rotation Position: ", rotationPos);
            telemetry.addData("Claw Position: ", clawPos);
            telemetry.addData("Elbow: ", elbowp);
            telemetry.addData("Encoder Position: ", encoderposs);
            telemetry.addData("Stafe: ", strafe);
            telemetry.addData("Turn: ", turn);
            telemetry.addData("Forward: ", forward);
            telemetry.update();
            wasXPressedLastLoop = isPressed;
            wasSqpressedlastloop = ispressed;
        }
    }
}