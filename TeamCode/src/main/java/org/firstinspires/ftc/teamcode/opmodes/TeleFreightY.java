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

import android.util.Log;

@TeleOp
public class TeleFreightY extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(this);

        SimpleMecanumDrive mecanumDrive = new SimpleMecanumDrive(robot, telemetry);
        Intake intake = new Intake(robot);
        Outtake outtake = new Outtake(robot, telemetry);
        DuckSpin duckSpin = new DuckSpin(robot, 0.4);

        robot.registerSubsystem(mecanumDrive);
        robot.registerSubsystem(outtake);
        robot.registerSubsystem(intake);
        robot.registerSubsystem(duckSpin);

        int slidecountup = 0;
        int slidecountdown = 0;

        intake.reset();
        outtake.dumpReset();

        while (!isStopRequested()) {

            robot.update();
            mecanumDrive.setDrivePower(new Pose2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x));

            if (gamepad1.left_bumper) {
                mecanumDrive.drivingMode = false;
                mecanumDrive.setTargetDist(200);
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

            //intaking
            if (gamepad1.x){
                intake.start();
            }
            else if (gamepad1.y){
                intake.stop();
            }

            //slide
            if (gamepad1.b){
                outtake.goUp();
            }
            else if (gamepad1.a){
                outtake.goDown();
            }

            //dump
            if (gamepad1.left_bumper){
                outtake.dump();
            }
            else if (gamepad1.right_bumper){
                outtake.dumpReset();
            }

            if (gamepad2.a) {
                duckSpin.start();
            }

            if (gamepad2.a) {
                duckSpin.stop();
            }

            telemetry.update();
        }
    }
}
