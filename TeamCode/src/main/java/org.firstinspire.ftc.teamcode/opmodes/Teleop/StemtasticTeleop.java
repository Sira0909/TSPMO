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

//TODO: WRITE pickup + backdrop MAROS + A LIMLIGHT VRSION FOR TTING APRILTAGS
//TODO: MAK XINHRAIUS AN RIV TO TAG AN P ONTROLLR AN RIV TO SP PT
//holy shit this keyboard finally works les goooo
@TeleOp (name = "CorrectTeleop")
public class StemtasticTeleop extends LinearOpMode {
    public RobotSystem robot;
    public ElapsedTime runtime = new ElapsedTime();

    public void reset(ElapsedTime resetter) {
        resetter.reset();
    }
    public double rotationPos;
    public double clawPos;
    public int encoderposs;
    public double lastError = 999;

    private boolean clawOpen = true;
    private boolean wasXPressedLastLoop = false;
    private boolean rotdown = true;
    private boolean wassqpressedlastloop = false;
    public double elbowpp;
    public double elbowp;
    public double speed = 0.4;
    public boolean toggleMacroTag = false;
    public boolean macroTagRunning = true;
    public AprilTagDetection tag1;
    public AprilTagDetection tag2;
    public AprilTagDetection tag3;
    public AprilTagDetection tag4;
    public WebcamName am = hardwareMap.get(WebcamName.class, "Wbam");
    public int poweroption = 0;

    public AprilTagDetection lastDetectedTag;
    AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
            .setDrawAxes(true)
            .setDrawTagID(true)
            .setDrawTagOutline(true)
            .setDrawCubeProjection(true)
            .build();
    VisionPortal visionPortal = new VisionPortal.Builder()
            .addProcessor(tagProcessor)
            .setCamera(am)
            .setCameraResolution(new Size(400,400)) //obv replace
            .setStreamFormat(VisionPortal.StreamFormat.YUY2)
            .enableLiveView(true)
            .build();

    public void detectTags() {
        ArrayList<AprilTagDetection> detections = tagProcessor.getDetections();
        if(detections != null) {
            telemetry.addLine("AprilTag Detected.");
            for(AprilTagDetection tag : detections) {
                telemetry.addData("ID", tag.id);
                switch(tag.id) {
                    case 1:
                        tag1 = tag;
                        break;
                    case 2:
                        tag2 = tag;
                        break;
                    case 3:
                        tag3 = tag;
                        break;
                    case 4:
                        tag4 = tag;
                        break;
                }
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
        double errorX = tagg.ftcPose.x;
        double errorY = tagg.ftcPose.y;
        double erroryaw = tagg.ftcPose.yaw;
        PD(errorX, 0);
        PD(errorY,1);
        PD(erroryaw,2);
    }
    public void PD (double error, double option) {
        double kP = 0.02;
        double kD = 0.002;
        lastError = error;
        reset(runtime);
        double deltaTime = runtime.seconds();
        double derivative = (error - lastError) / deltaTime;
        double power = kP * error + kD * derivative;
        if (option == 0) {
            robot.drive.driveRobotCentricPowers(power, 0,0);
        }
        else if (option == 1) {
            robot.drive.driveRobotCentricPowers(0,power, 0);
        }
        else if (option == 2) {
            robot.drive.driveRobotCentricPowers(0,0,power);
        }
    }
    @Override
    public void runOpMode () throws InterruptedException {
        visionPortal.setActiveCamera(am);
        visionPortal.setProcessorEnabled(tagProcessor, true);
        this.robot = new RobotSystem(hardwareMap, this);
        clawPos = RobotConstants.CLOSECLAW;
        robot.inDep.setClawPosition(clawPos);
        rotationPos = RobotConstants.CLAWROTATIONBACKBOARD;
        robot.inDep.setRotationPosition(rotationPos);
        waitForStart();
        while (opModeIsActive()) {
            double time = runtime.time();
            telemetry.addData("Time: ", time);
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
                } else {
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
            detectTags();
        }
    }
}