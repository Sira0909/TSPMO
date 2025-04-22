package org.firstinspire.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspire.ftc.teamcode.RobotConstants;

import org.firstinspire.ftc.teamcode.RobotConstants;
public class indepSubsystem extends SubsystemBase {

    private final LowClawSubsystem LOWCLAW;
    //private final CVSubsystem CV;

    public indepSubsystem(
            LinearOpMode opMode,
            DriveSubsystem driveSubsystem,
            LowClawSubsystem LowClaw
            //CVSubsystem cvSubsystem
    ) {
        LOWCLAW = LowClaw;
        //CV = cvSubsystem;
    }

    public void setElbowPos(double ElbowPos) {
        LOWCLAW.setElbowPos(ElbowPos);
    }
    public void setClawPosition(double clawPosition) {
        LOWCLAW.setClawPos(clawPosition);
    }
    public void setRotationPos(double rotationpos) {
        LOWCLAW.setRotationPos(rotationpos);
    }
}