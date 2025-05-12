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

    public HardwareMap hardwareMap;

    public RobotSystem(HardwareMap hardwareMap, LinearOpMode opMode) {
        this.opMode = opMode;
        HardwareRobot hardwareRobot = new HardwareRobot(hardwareMap);
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
}
