package org.firstinspire.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspire.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspire.ftc.teamcode.subsystems.LowClawSubsystem;
import org.firstinspire.ftc.teamcode.subsystems.indepSubsystem;

public class RobotSystem {

    public final LinearOpMode opMode;
    public final DriveSubsystem drive;
    public final indepSubsystem inDep;
    public final LowClawSubsystem lowClaw;

    public RobotSystem(HardwareMap hardwareMap, LinearOpMode opMode) {
        this.opMode = opMode;
        HardwareRobot hardwareRobot = new HardwareRobot(hardwareMap);
        this.drive = new DriveSubsystem(
                hardwareRobot.leftFront,
                hardwareRobot.rightFront,
                hardwareRobot.leftBack,
                hardwareRobot.rightBack
        );
        this.inDep = null;
        this.lowClaw = null;
    }
}
