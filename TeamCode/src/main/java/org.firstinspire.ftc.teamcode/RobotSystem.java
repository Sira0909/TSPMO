package org.firstinspire.ftc.teamcode;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspire.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspire.ftc.teamcode.subsystems.indepSubsystem;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class RobotSystem {

    public final LinearOpMode opMode;
    public final DriveSubsystem drive;
    public final indepSubsystem inDep;
    public final HardwareRobot hardwareRobot;

    public HardwareMap hardwareMap;

    public RobotSystem(HardwareMap hardwareMap, LinearOpMode opMode) {
        this.opMode = opMode;
        this.hardwareRobot = new HardwareRobot(hardwareMap);
        this.drive = new DriveSubsystem(
                hardwareRobot.leftFront,
                hardwareRobot.rightFront,
                hardwareRobot.leftBack,
                hardwareRobot.rightBack
        );
        this.inDep = new indepSubsystem(
                opMode,
                drive,
                hardwareRobot
        );
    }
    public void InitAprilTags(WebcamName am) {
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
                .setAutoStopLiveView(true)
                .build();
    }
}
