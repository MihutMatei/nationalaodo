package org.firstinspires.ftc.teamcode.util;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.advanced.DetectionPipeline;
import org.firstinspires.ftc.teamcode.drive.advanced.SamplePipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class RobotUtils extends OpMode {
   // OpenCvCamera webcam;

    public DcMotorEx slider;
    public DcMotorEx intake;
    public DcMotor carusel;
    public Servo cuva;
    public DcMotorEx rotire;
    public ColorSensor color;
    public SampleMecanumDrive drive;

    public void startIntake(double power)
    {
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setPower(power);
    }

    public void startOuttake(double power)
    {
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setPower(power);
    }

    public void stopIntake() { intake.setPower(0); }
    public void openIntake() { cuva.setPosition(0.5); }
    public void closeIntake() { cuva.setPosition(0.09); }

    public RobotUtils(HardwareMap hardwareMap)
    {
        drive = new SampleMecanumDrive(hardwareMap);

//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//
//        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        slider = hardwareMap.get(DcMotorEx.class, "slider");

        intake = hardwareMap.get(DcMotorEx.class, "intake");

        carusel = hardwareMap.get(DcMotor.class, "carusel");

        cuva = hardwareMap.get(Servo.class,"cuva");

        rotire = hardwareMap.get(DcMotorEx.class,"rotire");

        color = hardwareMap.get(ColorSensor.class, "color");

       // webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            @Override
//            public void onOpened() {
//                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
//            }
//
//            @Override
//            public void onError(int errorCode) {
//
//            }
//        });
    }
    @Override
    public void init() {
    }

    @Override
    public void loop() {

    }
}