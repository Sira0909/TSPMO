package org.firstinspire.ftc.teamcode.opmodes.Teleop;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspire.ftc.teamcode.RobotSystem;
import org.firstinspire.ftc.teamcode.RobotConstants;
//TODO: FIX DRIVE MOTORS AND WRITE DRIVE CODE AND TEST ARM STABILITY, WRITE PIKUP + BAKDROP MAROS
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
    public double speed = 0.5;
    public boolean toggleMacroOne = false;
    public boolean MacroOne = false;

    @Override
    public void runOpMode () throws InterruptedException {
        this.robot = new RobotSystem(hardwareMap, this);
        double lastError = 9999999;
        //this will be fixed once i have the arm
        clawPos = RobotConstants.CLOSECLAW;
        robot.inDep.setClawPosition(clawPos);
        rotationPos = RobotConstants.CLAWROTATIONBACKBOARD;
        robot.inDep.setRotationPosition(rotationPos);
        ElapsedTime runtime = new ElapsedTime(0);
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
            if (gamepad1.triangle && !toggleMacroOne) {
                MacroOne = true;
            }
            toggleMacroOne = gamepad1.triangle;
            if (MacroOne) {
                double target = -659;
                double error = target - encoderposs;
                double kP = 0.01;
                double kD = 0.001;
                error = target - encoderposs;
                double deltaTime = runtime.seconds();
                double derivative = (error - lastError) / deltaTime;
                if (lastError == 9999999) {
                     derivative = 0;
                }
                double u_t = kP * error + kD * derivative;
                elbowp = u_t;
                lastError = error;
                runtime.reset();
                if (error <= 10) {
                    MacroOne = false;
                }
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