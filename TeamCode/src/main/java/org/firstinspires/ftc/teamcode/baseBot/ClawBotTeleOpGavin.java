package org.firstinspires.ftc.teamcode.baseBot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.teamcode.outreachBot.ClawBot;
import org.firstinspires.ftc.teamcode.outreachBot.Constants;


@TeleOp(name="GavinBot", group="Iterative Opmode")
public class ClawBotTeleOpGavin extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private ClawBot myRobot = new ClawBot();
    boolean openClaw = true;
    boolean liftThing = true;
    double liftPos = Constants.liftDrive;
    double clawPos = Constants.clawOpenA;
    int countTime = 100;
    public void init() {
        myRobot.initialize(hardwareMap, telemetry);
        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    public void init_loop() {}

    public void start() {
        runtime.reset();
    }

    public void loop() {
        if (countTime != 100) {
            countTime++;

        }
        //Drive motor controls
        double lx = gamepad1.left_stick_x;
        double ly = -gamepad1.left_stick_y;
        double speedMultiplier = 1;
        double rotationMultiplier = .8;

        if(gamepad1.dpad_up){
            ly=1;
            lx=0;
            speedMultiplier = 0.3;
        }
        else if(gamepad1.dpad_down){
            ly=-1;
            lx=0;
            speedMultiplier = 0.3;
        }
        if(gamepad1.dpad_left){
            lx=-1;
            ly=0;
            speedMultiplier = 0.6;
        }
        else if(gamepad1.dpad_right){
            lx=1;
            ly=0;
            speedMultiplier = 0.6;
        }

        double theta = Math.atan2(lx, ly);
        double v_theta = Math.sqrt(lx * lx + ly * ly);
        double v_rotation = gamepad1.right_stick_x;
        myRobot.drive(theta,  speedMultiplier*v_theta, rotationMultiplier*v_rotation);
        if (gamepad1.left_trigger > 0.9 && countTime == 100){
            countTime = 0;
            liftThing = !liftThing;
            if(liftThing) {
                liftPos = Constants.liftTop;
            }
            else {
                liftPos = Constants.liftBot;

            }
        }
        if (gamepad1.left_bumper && countTime == 100) {
            countTime = 0;
            openClaw = !openClaw;
            if (openClaw) {
                clawPos = Constants.clawOpenA;
            }
            else {
                clawPos = Constants.clawCloseA;

            }

        }
        if (Math.abs(lx) < 0.05 || Math.abs(ly) < 0.05 || Math.abs(v_rotation) < 0.05) {
           liftPos = Constants.liftDrive;
        }

        myRobot.setLiftServo(liftPos);
        myRobot.setClawServo(clawPos);
    }

    public void stop() {}
}
