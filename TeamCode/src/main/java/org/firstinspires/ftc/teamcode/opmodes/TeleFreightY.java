package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.SimpleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.DuckSpin;
import org.firstinspires.ftc.teamcode.subsystems.CrabRobot;

import android.util.Log;

@TeleOp
public class TeleFreightY extends LinearOpMode {

    private CrabRobot crabRobot;
    @Override
    public void runOpMode() throws InterruptedException {
        /*
        Intake intake = new Intake(crabRobot);
        Outtake outtake = new Outtake(crabRobot, telemetry);
        DuckSpin duckSpin = new DuckSpin(crabRobot);

        /*
        crabRobot.registerSubsystem(outtake);
        crabRobot.registerSubsystem(intake);
        crabRobot.registerSubsystem(duckSpin);
        */


        int slidecountup = 0;
        int slidecountdown = 0;

        crabRobot = new CrabRobot(this, telemetry);
        /*
        crabRobot.intake.reset();
        crabRobot.outtake.dumpReset();

         */

        crabRobot.mecanumDrive.drivingMode = true;
        waitForStart();

        while (!isStopRequested()) {
            crabRobot.mecanumDrive.setDrivePower(new Pose2d(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x));
            Log.i("power", "gamepad data" + " "
                    + -gamepad1.left_stick_y + " " + gamepad1.left_stick_x + " " + gamepad1.right_stick_x);
            Log.i("mecanumdrive", ""+crabRobot.mecanumDrive.drivingMode);
            crabRobot.update();

            if (gamepad1.left_bumper) {
                crabRobot.mecanumDrive.drivingMode = false;
                crabRobot.mecanumDrive.setTargetDist(200);
                telemetry.addLine("Commence Align mode.");
                //telemetry.addLine("Distance: "+ mecanumDrive.getDistL()+" "+mecanumDrive.getDistR());
/*
                try {
                    Thread.sleep(1000);
                }
                catch (InterruptedException e){
                    ;
                }
 */
            }
/*
            //intaking
            if (gamepad1.x){
                crabRobot.intake.start();
            }
            else if (gamepad1.y){
                crabRobot.intake.stop();
            }

            //slide
            if (gamepad1.b){
                crabRobot.outtake.goUp();
            }
            else if (gamepad1.a){
                crabRobot.outtake.goDown();
            }

            //dump
            if (gamepad1.left_bumper){
                crabRobot.outtake.dump();
            }
            else if (gamepad1.right_bumper){
                crabRobot.outtake.dumpReset();
            }

            if (gamepad2.a) {
                crabRobot.duckSpin.start(0.6);
            }

            if (gamepad2.b) {
                crabRobot.duckSpin.stop();
            }

            telemetry.update();

 */


        }
    }
}
