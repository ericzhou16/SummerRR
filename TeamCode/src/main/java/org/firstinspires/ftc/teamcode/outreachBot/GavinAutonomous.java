package org.firstinspires.ftc.teamcode.outreachBot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.baseBot.BaseAutonomousMethods;

@Autonomous(name = "Red Park", group = "Linear Opmode")

public class GavinAutonomous extends BaseAutonomousMethods {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        initializeAuto(hardwareMap, telemetry);
        myRobot.setClawServo(Constants.clawOpenA);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        encoderStraightDrive(6.0, 0.7);
        encoderTurn(35, 0.7, 1.5);
        encoderStraightDrive(10, 0.7);
        encoderStrafeDriveInchesRight(15, 0.7);
        encoderTurn(500, 1, 3);
        myRobot.setClawServo(Constants.clawCloseA);
    }
}