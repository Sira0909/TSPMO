package org.firstinspire.ftc.teamcode.opmodes.Teleop;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
    public double speed;

    @Override
    public void runOpMode () throws InterruptedException {
        this.robot = new RobotSystem(hardwareMap, this);
        //this will be fixed once i have the arm
        clawPos = RobotConstants.CLOSECLAW;
        robot.inDep.setClawPosition(clawPos);
        rotationPos = RobotConstants.CLAWROTATIONBACKBOARD;
        robot.inDep.setRotationPosition(rotationPos);
        waitForStart();
        while(opModeIsActive()) {
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;
            double forward = -gamepad1.left_stick_y;
            robot.drive.driveRobotCentric(strafe, forward, turn);
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
            encoderposs = robot.inDep.getEncoder(encoderposs);
            robot.inDep.setElbowPosition(elbowpp);
            robot.inDep.setClawPosition(clawPos);
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
