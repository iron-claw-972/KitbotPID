package org.usfirst.frc.team972.robot;

import com.analog.adis16448.frc.ADIS16448_IMU;
import com.analog.adis16448.frc.ADIS16448_IMU.AHRSAlgorithm;
import com.analog.adis16448.frc.ADIS16448_IMU.Axis;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.interfaces.Accelerometer.Range;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends IterativeRobot {

    Victor backRightMotor = new Victor(0);
    Victor backLeftMotor = new Victor(1);
    Victor frontRightMotor = new Victor(2);
    Victor frontLeftMotor = new Victor(3);
    
    ADIS16448_IMU imu = new ADIS16448_IMU();
    
    PIDControl pid;
    
    RobotDrive drive = new RobotDrive(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor);

    Joystick gamepad = new Joystick(0);
    
    public void teleopInit() {
        //imu.calibrate();
        imu.reset();
        pid = new PIDControl(0.013, 0, 0);
        pid.setOutputLimits(-0.5, 0.5);
        //pid.setOutputRampRate(0.05);
        
        pid.setSetpoint(0);
        
    }
    
    double lastError = 0;
    double total = 0;
    double joystickAngles = 0;
    double accelK = 0;
    
    boolean run = false;
    
    public void teleopPeriodic() {
        double currentAngle = (Math.round(imu.getAngleZ() * 100.0) / 100.0) + joystickAngles;
        double pidOutputPower = pid.getOutput(currentAngle);
        
        double leftSpeed = gamepad.getRawAxis(5);
        double rightSpeed = gamepad.getRawAxis(1);
        
        joystickAngles = 0;
        
        if(gamepad.getRawButton(5)) { //left top
            joystickAngles = ((leftSpeed - rightSpeed) * 360);
            drive.tankDrive(pidOutputPower, -pidOutputPower); //maybe?
            System.out.println(pidOutputPower);
        } else if(gamepad.getRawButton(6)) {
            imu.reset();
        } else if(gamepad.getRawButton(1))
        {
            drive.tankDrive((leftSpeed * .6) + (pidOutputPower/4) , (leftSpeed * .6) + (-pidOutputPower/4));
            System.out.println(pidOutputPower);
        } else {
            drive.tankDrive(leftSpeed * 0.6, rightSpeed * 0.6);
        }
        
        System.out.println(imu.getMagZ());
        
    }
    
    public void disabledPeriodic() {
        run = false;
    }
    
    public void autonomousInit() {
        imu.calibrate();
        imu.reset();
        
        pid = new PIDControl(0.012, 0.000, 0);
        pid.setOutputLimits(-0.5, 0.5);
        pid.setSetpoint(0);
        
        System.out.println("Auto will start in 1 second");
        
        waitThread(1000);
        
        run = true;
        
        /*
        while(run) {
            waitThread(1000);
            DriveStraight(1.8, 0.6);
            DriveTurn(90);
            DriveStraight(1.2, 0.55);
            DriveTurn(90);
            DriveStraight(1.8, 0.6);
            DriveTurn(90);
            DriveStraight(1.2, 0.55);
            DriveTurn(90);
        }*/
        
        //AutoDriveInSquareRoutine();
        
        DriveTurn(180);
        waitThread(1500);
        DriveTurn(180);
        waitThread(1500);
        DriveTurn(360);
        waitThread(1500);
        DriveTurn(90);
        
    }
    
    public void AutoDriveInSquareRoutine() {
    	DriveStraight(2.8, 0.6);
    	waitThread(1500);
    	DriveTurn(90);
    	waitThread(1500);
    	
    	DriveStraight(2.8, 0.6);
    	waitThread(1500);
    	DriveTurn(90);
    	waitThread(1500);
    	
    	DriveStraight(2.8, 0.6);
    	waitThread(1500);
    	DriveTurn(90);
    	waitThread(1500);
    	
    	DriveStraight(2.81, 0.6); //go a tiny bit farther on last one.
    	waitThread(1500);
    	DriveTurn(90);
    	waitThread(1500);
    }
    
    public double ease(double t, double b, double c, double d) {
        t /= d/2;
        if (t < 1) return c/2*t*t + b;
        t--;
        return -c/2 * (t*(t-2) - 1) + b;
    };
    
    public void waitThread(double time) {
        try {
            Thread.sleep((long) time);
        } catch (InterruptedException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
    }
    
    public void DriveTurn(double angle) {
    	pid.reset();
    	imu.reset();
    	
    	if(angle > 0) {
    		pid.setOutputLimits(0, 0.7);
    	} else{
    		pid.setOutputLimits(0, -0.7);
    	}
    	
    	
    	pid.setOutputRampRate(0.5);
    	
    	pid.setP(0.015);
    	
    	pid.setD(0.15);
    	pid.setF(.1);
    	
    	int inTheZone = 0;
    	
        double currentAngle = (Math.round(imu.getAngleZ() * 100.0) / 100.0) + (angle * 4) - (lastError * .8);
        double pidOutputPower = pid.getOutput(currentAngle);
        
        System.out.println(pidOutputPower);
        
        while((inTheZone < 25) && run) {
        	currentAngle = (Math.round(imu.getAngleZ() * 100.0) / 100.0) - (angle * 4);
        	pidOutputPower = pid.getOutput(currentAngle);
        	
        	if(Math.abs(currentAngle) < 20) {
        		pid.setOutputLimits(-0.5, 0.5);
        		pid.setI(0.025);
        		pid.setMaxIOutput(0.35);
        		inTheZone++;
        	}
        	
            drive.tankDrive(pidOutputPower, -pidOutputPower);
            System.out.println(pidOutputPower + " " + currentAngle + " " + (angle*4));
            waitThread(20);
        }
        
        pid.setP(0.012);
        
        lastError = currentAngle;
        pid.setF(0);
        pid.setD(0);
        pid.setI(0);
    }
    
    public void DriveStraight(double time, double power) {
        imu.reset();
        pid.reset();
        for(int i=0; i<50 * time; i++) {
            double currentAngle = (Math.round(imu.getAngleZ() * 100.0) / 100.0);
            double pidOutputPower = pid.getOutput(currentAngle);
            
            drive.tankDrive(power + (pidOutputPower/4), power + (-pidOutputPower/4));
            
            waitThread(20);
        }
    }
    
}