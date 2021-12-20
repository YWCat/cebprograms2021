package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.robot.Robot;

import org.firstinspires.ftc.teamcode.robot.Robot;

public class  CrabRobot extends Robot {
    public final SimpleMecanumDrive mecanumDrive;
/*
    public final Slide slide;
    public final Intake intake;
    public final Outtake outtake;
    public final DuckSpin duckSpin;
 */




    public CrabRobot(LinearOpMode opMode, Telemetry telemetry) {
        super(opMode);
        mecanumDrive = new SimpleMecanumDrive(this, telemetry);
        registerSubsystem(mecanumDrive);
        Log.i("mecanumdrive crabRobot", "mecDrive CrabRobot");

        /*
        slide = new Slide(this, telemetry);
        registerSubsystem(slide);
        intake = new Intake(this);
        registerSubsystem(intake);
        outtake = new Outtake(this, telemetry);
        registerSubsystem(outtake);
        duckSpin = new DuckSpin(this);
        registerSubsystem(duckSpin);
*/

    }
}
