package org.firstinspire.ftc.teamcode.Cindysmomusethisstuff;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspire.ftc.teamcode.RobotConstants;
import org.firstinspire.ftc.teamcode.RobotSystem;



@TeleOp (name = "Cindy's mom: use this.")
public class TeleOpNoAprilTags extends LinearOpMode {
    public RobotSystem robot;
    public double rotationPos;
    public double clawPos;
    public int encoderposs;
    public double elbowp;
    private boolean clawOpen = true;
    private boolean wasXPressedLastLoop = false;
    private boolean rotdown = true;
    private boolean wasSqpressedlastloop = false;
    public double speed = 0.45;
    @Override
    public void runOpMode() throws InterruptedException {
        this.robot = new RobotSystem(hardwareMap, this);
        clawPos = RobotConstants.CLOSECLAW;
        robot.inDep.setClawPosition(clawPos);
        rotationPos = RobotConstants.CLAWROTATIONBACKBOARD;
        robot.inDep.setRotationPosition(rotationPos);
        waitForStart();
        while (opModeIsActive()) {
            boolean circlePressd = gamepad1.circle;
            encoderposs = robot.inDep.getEncoder(encoderposs);
            elbowp = gamepad1.right_stick_y * 0.5;
            double strafe = -gamepad1.left_stick_y;
            double forward = -gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;
            if (Math.abs(turn) > 0) {
                robot.hardwareRobot.changeInversions();
            }
            robot.drive.driveRobotCentricPowers(strafe * speed,forward * speed,turn * speed);
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
            //update this
            robot.inDep.setClawPosition(clawPos);
            robot.inDep.setRotationPosition(rotationPos);
            robot.inDep.setElbowPosition(elbowp);
            telemetry.addData("Strafe: ", strafe);
            telemetry.addData("Turn: ", turn);
            telemetry.addData("Forward: ", forward);
            telemetry.addData("Rotation Position: ", rotationPos);
            telemetry.addData("Claw Position: ", clawPos);
            telemetry.addData("Encoder Position: ", encoderposs);
            wasXPressedLastLoop = isPressed;
            wasSqpressedlastloop = ispressed;
        }
    }
}
