package org.firstinspire.ftc.teamcode.auto;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspire.ftc.teamcode.RobotSystem;
import org.firstinspire.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.CameraControl;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Mat;

import java.sql.Array;
import java.util.ArrayList;

@Autonomous(name = "unwashing")
public class Unwashing extends LinearOpMode {
    public VisionPortal vp;
    public AprilTagProcessor pro;
    public double speed;
    public RobotSystem robot;
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
        while (opModeIsActive()) {

        }
    }
    public void detectTags() {
        ArrayList<AprilTagDetection> detections = pro.getDetections();
        for (AprilTagDetection tag : detections) {
            telemetry.addData("ID", tag.id);
            telemetry.addData("X", tag.ftcPose.x);
        }
    }
}
