package org.firstinspire.ftc.teamcode.opmodes.Teleop;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspire.ftc.teamcode.RobotSystem;
import org.firstinspire.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.ArrayList;

//TODO: WRITE PIKUP + BAKDROP MAROS
@TeleOp (name = "CorrectTeleop")
public class StemtasticTeleop extends LinearOpMode {
    public RobotSystem robot;
    public double rotationPos;
    public double clawPos;
    public int encoderposs;

    private boolean clawOpen = true;
    private boolean wasXPressedLastLoop = false;
    private boolean rotdown = true;
    private boolean wassqpressedlastloop = false;
    public double elbowpp;
    public double elbowp;
    public double speed = 0.4;
    public boolean toggleMacro = false;
    public boolean macroRunning = true;

    public double lastError = 9999;

    public AprilTagDetection lastDetectedTag;
    AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
            .setDrawAxes(true)
            .setDrawTagID(true)
            .setDrawTagOutline(true)
            .setDrawCubeProjection(true)
            .build();
    VisionPortal visionPortal = new VisionPortal.Builder()
            .addProcessor(tagProcessor)
            .setCamera(hardwareMap.get(WebcamName.class, "Webcam"))
            .setCameraResolution(new Size(400,400)) //obv replace
            .setStreamFormat(VisionPortal.StreamFormat.YUY2)
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

    @Override
    public void runOpMode () throws InterruptedException {
        this.robot = new RobotSystem(hardwareMap, this);
        clawPos = RobotConstants.CLOSECLAW;
        robot.inDep.setClawPosition(clawPos);
        rotationPos = RobotConstants.CLAWROTATIONBACKBOARD;
        robot.inDep.setRotationPosition(rotationPos);
        waitForStart();
        while(opModeIsActive()) {
            double strafe = -gamepad1.left_stick_x;
            double turn = -gamepad1.right_stick_x;
            double forward = -gamepad1.left_stick_y;
            robot.drive.driveRobotCentricPowers(strafe * speed, forward * speed, turn * speed);
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
            if (ispressed && !wassqpressedlastloop) {
                rotdown = !rotdown;
                if (rotdown) {
                    rotationPos = RobotConstants.CLAWROTATIONPIKINGUP;
                }
                else {
                    rotationPos = RobotConstants.CLAWROTATIONBACKBOARD;
                }
            }
            elbowp = -gamepad1.right_stick_y;
            if (elbowp >= 0) {
                elbowpp = elbowp * 0.3;
            }
            if (elbowp < 0) {
                elbowpp = elbowp * 0.1;
            }
            encoderposs = robot.inDep.getEncoder(encoderposs);
            robot.inDep.setElbowPosition(elbowpp);
            robot.inDep.setClawPosition(clawPos);
            robot.inDep.setRotationPosition(rotationPos);
            telemetry.addData("Rotation Position: ", rotationPos);
            telemetry.addData("Claw Position: ", clawPos);
            telemetry.addData("Elbow: ", elbowp);
            telemetry.addData("Encoder Position: ", encoderposs);
            telemetry.addData("Stafe: ", strafe);
            telemetry.addData("Turn: ", turn);
            telemetry.addData("Forward: ", forward);
            telemetry.update();
            wasXPressedLastLoop = isPressed;
            wassqpressedlastloop = ispressed;
        }
    }
}