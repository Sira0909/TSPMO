package org.firstinspire.ftc.teamcode.auto;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspire.ftc.teamcode.RobotConstants;
import org.firstinspire.ftc.teamcode.RobotSystem;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.ArrayList;
//the unwashing has been completed.
@Autonomous(name = "unwashing")
public class Unwashing extends LinearOpMode {
    public double clawPos;
    public double elbowP;
    public double rotationPos;
    public VisionPortal vp;
    public boolean lastClaw = false;
    public boolean lastRotation = false;
    public AprilTagProcessor pro;
    public boolean claw;
    public boolean flip;
    public boolean rot;
    public boolean rotation;
    public double speed;
    public RobotSystem robot;
    public AprilTagDetection currentTag;
    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        pro = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawCubeProjection(true)
                .build();
        vp = new VisionPortal.Builder()
                .addProcessor(pro)
                .enableLiveView(true)
                .setAutoStartStreamOnBuild(true)
                .setAutoStopLiveView(true)
                .setCamera(robot.hardwareRobot.webcamName)
                .setLiveViewContainerId(cameraMonitorViewId)
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .build();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new RobotSystem(hardwareMap, this);
        vp.setProcessorEnabled(pro, true);
        rotationPos = RobotConstants.CLAWROTATIONBACKBOARD;
        clawPos = RobotConstants.OPENCLAW;
        waitForStart();
        while (opModeIsActive()) {
            claw = gamepad1.square;
            if (claw && !lastClaw) {
                flip = !flip;
                if (flip) {
                    clawPos = RobotConstants.OPENCLAW;
                } else {
                    clawPos = RobotConstants.CLOSECLAW;
                }
            }
            rotation = gamepad1.triangle;
            if (rotation && !lastRotation) {
                rot = !rot;
                if (rot) {
                    rotationPos = RobotConstants.CLAWROTATIONBACKBOARD;
                } else {
                    rotationPos = RobotConstants.CLAWROTATIONPIKINGUP;
                }
            }
            detectTags();
            if (currentTag != null && xInchRadius(currentTag, 5)) {
                speed -= 0.2;
            }
            double strafe = gamepad1.left_stick_x;
            double forward = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            robot.drive.driveRobotCentricPowers(strafe * speed, forward * speed, turn * speed);
            elbowP = gamepad1.right_stick_y;
            telemetry.addData("Elbow", elbowP);
            telemetry.addData("Claw", flip);
            telemetry.addData("Rotation", rot);
            telemetry.addData("Strafe", strafe);
            telemetry.addData("Foward", forward);
            telemetry.addData("Turn", turn);
            robot.inDep.setElbowPosition(elbowP);
            robot.inDep.setClawPosition(clawPos);
            robot.inDep.setRotationPosition(rotationPos);
            lastClaw = claw;
            lastRotation = rotation;
        }
    }
    public void detectTags() {
        ArrayList<AprilTagDetection> detections = pro.getDetections();
        if (!detections.isEmpty()) {
            for (AprilTagDetection tag : detections) {
                telemetry.addData("ID", tag.id);
                telemetry.addData("X", tag.ftcPose.x);
                telemetry.addData("Y", tag.ftcPose.y);
                telemetry.addData("Z", tag.ftcPose.z);
                telemetry.addData("Yaw", tag.ftcPose.yaw);
                telemetry.addData("Bearing", tag.ftcPose.bearing);
                telemetry.addData("Range", tag.ftcPose.range);
                currentTag = tag;
                break;
            }
        } else {
            currentTag = null;
        }
        telemetry.update();
    }
    public boolean xInchRadius(AprilTagDetection target, int radius) {
        return target.ftcPose.range <= radius;
    }
}
