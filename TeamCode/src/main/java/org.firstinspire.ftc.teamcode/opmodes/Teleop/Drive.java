package org.firstinspire.ftc.teamcode.opmodes.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspire.ftc.teamcode.RobotSystem;
@TeleOp (name = "Drive Test")
public class Drive extends LinearOpMode {
    public RobotSystem robot;
    public double speed = 0.4;
    @Override
    public void runOpMode() throws InterruptedException {
        this.robot = new RobotSystem(hardwareMap, this);
        while(opModeIsActive()) {
            double strafe = -gamepad1.left_stick_x;
            double turn = -gamepad1.right_stick_x;
            double forward = -gamepad1.left_stick_y;
            robot.drive.driveRobotCentricPowers(strafe * speed, forward * speed, turn * speed);
        }
    }
}
