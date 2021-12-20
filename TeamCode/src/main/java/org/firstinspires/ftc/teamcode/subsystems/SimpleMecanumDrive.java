package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogInputController;
import com.qualcomm.robotcore.hardware.AnalogSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.hardware.Encoder;


import android.util.Log;

import org.firstinspires.ftc.teamcode.hardware.CachingSensor;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.Subsystem;

public class SimpleMecanumDrive implements Subsystem {

    /*
    private int MOTOR_LF= 0;
    private int MOTOR_LR= 3;
    private int MOTOR_RR =2;
    private int MOTOR_RF= 1;
    */
            
    private DcMotorEx[] motors = new DcMotorEx[4];
    private CachingSensor<Float> headingSensor;

    //private AnalogSensor scale_n, scale_p;

    //add distance sensors

    /*
    private DistanceSensor distL;
    private DistanceSensor distR;
    private ColorSensor color;
    */

    private Double[] powers = {0.0, 0.0, 0.0, 0.0};

    //PID stuff

    private PIDFController distPID;
    private PIDFController anglePID;
    private static final PIDCoefficients DIST_PID_COEFFICIENTS = new PIDCoefficients(0.0008, 0, 0);
    private static final PIDCoefficients ANG_PID_COEFFICIENTS = new PIDCoefficients(0.007, 0, 0);

    public boolean drivingMode = true;

    public Telemetry telemetry;

    public SimpleMecanumDrive (Robot robot, Telemetry telemetry) {

        motors[0] = robot.getMotor("DriveLF");
        motors[1] = robot.getMotor("DriveRF");
        motors[2] = robot.getMotor("DriveRR");
        motors[3] = robot.getMotor("DriveLR");


        BNO055IMU imu = robot.getIMU("imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
        headingSensor = new CachingSensor<>(() -> imu.getAngularOrientation().firstAngle);
        robot.addListener(headingSensor);

        //forward power = counterclockwise
        motors[1].setDirection(Direction.REVERSE);
        motors[2].setDirection(Direction.REVERSE);
        //scale_n = robot.getAnalogSensor("NInput");
        //scale_p = robot.getAnalogSensor("PInput");

        /*
        //for mecanum drive purposes
        for (DcMotorEx motor:motors){
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        */

        /*
        distR = robot.getDistanceSensor("distanceR");
        distL = robot.getDistanceSensor("distanceL");
        */
        //color = robot.getColorSensor("color");

        //PID
        distPID = new PIDFController(DIST_PID_COEFFICIENTS);
        anglePID = new PIDFController(ANG_PID_COEFFICIENTS);

        this.telemetry = telemetry;

        distPID.setOutputBounds(-0.3, 0.3);
        anglePID.setOutputBounds(-0.1, 0.1);
    }


    public double getHeading() {
        return headingSensor.getValue();
    }

    /*
    //unlock if minibot
    public double getColorDist() {
        return color.alpha();
    }
    /*
    public double getDistL() {
        return distL.getDistance(DistanceUnit.MM);
    } //make caching sensor?
    public double getDistR() {
        return distR.getDistance(DistanceUnit.MM);
    }
    */

    public void setTargetDist(double targetDistance) {
        distPID.reset();
        anglePID.reset();
        distPID.setTargetPosition(targetDistance);
        anglePID.setTargetPosition(0);
    }

    public void setDrivePower(Pose2d drivePower) { //drivePower = game controller state

        if (drivingMode) {
            Log.i("power", drivePower.getX() + " " + drivePower.getY() + " " + drivePower.getHeading());
            //LF
            powers[0] = drivePower.getX() + drivePower.getY() + drivePower.getHeading();
            //RF
            powers[1] = drivePower.getX() - drivePower.getY() - drivePower.getHeading();
            //RR
            powers[2] = drivePower.getX() + drivePower.getY() - drivePower.getHeading();
            //LR
            powers[3] = drivePower.getX() - drivePower.getY() + drivePower.getHeading();
            /*
            //previous code for ceb?
            powers[0] = drivePower.getX() - drivePower.getY() - drivePower.getHeading();
            powers[1] = drivePower.getX() + drivePower.getY() - drivePower.getHeading();

            powers[2] = drivePower.getX() - drivePower.getY() + drivePower.getHeading();
            powers[3] = drivePower.getX() + drivePower.getY() + drivePower.getHeading();
             */
        }
        else{
            /*
            double powerL = -distPID.update(getDistL()); //change to getDistL()
            //double powerR = -distPID.update(getDistR()) //change to getDistR() and average powerL and powerR
            double angle = Math.atan( (getDistL() - getDistR()) / 152.4 ); //152.4 = dist between sensors
            double angleAdjust = anglePID.update(angle);


            powers[0] = powerL + angleAdjust;
            powers[1] = -powerL + angleAdjust;
            powers[2] = -powerL - angleAdjust;
            powers[3] = powerL - angleAdjust;

            //telemetry.addLine("power: " + powerL + "      Distance: "+ getDistL()+ " "+ (200-getDistL()));
            Log.i("PIDRun", "power" + powerL + "      Distance & error: "+ getDistL()+ " "+ (200-getDistL()));
            Log.i("PIDRun angles", "angle: "+ angle + " angle power"+ angleAdjust +" ");
            Log.i("PIDRun powers", powers[0]+ " "+ powers[1]+ " "+  powers[2]+ " "+ powers[3]);
            telemetry.update();

             */
        }


    }

    @Override
    public void update(TelemetryPacket packet) {

        motors[0].setPower(powers[0]);
        motors[1].setPower(powers[1]);
        motors[2].setPower(powers[2]);
        motors[3].setPower(powers[3]);

        Log.i("mecanumdrive", "Power set"+powers[0]+" "+powers[1]+powers[2]+" "+powers[3]+" ");
        //packet.put("Scale N", scale_n.readRawVoltage());
        //packet.put("Scale P", scale_p.readRawVoltage());
    }

}
