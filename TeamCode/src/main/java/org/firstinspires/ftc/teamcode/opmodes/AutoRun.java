package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.SimpleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.DuckSpin;
import org.firstinspires.ftc.teamcode.subsystems.CrabRobot;
import org.firstinspires.ftc.teamcode.commands.*;

import android.util.Log;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class AutoRun extends LinearOpMode {
    private CrabRobot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new CrabRobot(this, telemetry);

        waitForStart();

        Log.i("mecanumdrive autorun", "mec drive autorun");
        spinDuck();
    }

    public void spinDuck() {

        robot.runCommand(
                new DriveForTime(robot.mecanumDrive, new Pose2d(-0.3, 0, 0),2)
        );

        robot.runCommand(
                new DriveForTime(robot.mecanumDrive, new Pose2d(0, 0.3, 0),1)
        );
/*
        robot.runCommands(
                new Turn(robot.mecanumDrive, Math.toRadians(90)),
                new Spin(robot.slide, 1,1)
        );

 */
    }
}