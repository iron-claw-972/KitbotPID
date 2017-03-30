package org.usfirst.frc.team972.robot;

import com.analog.adis16448.frc.ADIS16448_IMU;
import com.analog.adis16448.frc.ADIS16448_IMU.AHRSAlgorithm;
import com.analog.adis16448.frc.ADIS16448_IMU.Axis;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.GenericHID.Hand;
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
    
    TimeOfFlight tof = new TimeOfFlight();
    
    RobotDrive drive = new RobotDrive(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor);

    Joystick gamepad = new Joystick(0);
    
    public void teleopInit() {
        //imu.calibrate();
    	tof.port.closePort();
    	tof = new TimeOfFlight();
    	
        imu.reset();
        pid = new PIDControl(0.012, 0, 0);
        pid.setOutputLimits(0.5);
        //pid.setOutputRampRate(0.05);
        pid.setOutputRampRate(0.05);
        pid.setF(0.05);
        pid.setSetpoint(0);
    }
    
    double lastError = 0;
    double total = 0;
    double joystickAngles = 0;
    double accelK = 0;
    
    boolean run = false;
    
    boolean newHeadingClick = false;
    
    public void teleopPeriodic() {
        double currentAngle = (Math.round(imu.getAngleZ() * 100.0) / 100.0) + joystickAngles;
        double pidOutputPower = pid.getOutput(currentAngle);
        
        double leftSpeed = gamepad.getRawAxis(5);
        double rightSpeed = gamepad.getRawAxis(1);
        
        joystickAngles = 0;
        
        
        System.out.println(tof.GetDataInMillimeters());
        
        if(gamepad.getRawButton(5)) { //left top
        	if(newHeadingClick) {
        		pid.reset();
        		imu.reset();
        		newHeadingClick = false;
        	}
            drive.tankDrive(pidOutputPower, -pidOutputPower);
        } else if(gamepad.getPOV() > 0) {
        	
        	double leftSpeedOffseter = 0;
        	
        	if(gamepad.getPOV() == 90) {
        		leftSpeedOffseter = -0.1;
            	if(newHeadingClick) {
            		imu.reset();
            		pid.reset();
            		joystickAngles = 360;
            		newHeadingClick = false;
            	} else{
            		joystickAngles = 360;
            		drive.tankDrive(pidOutputPower + leftSpeedOffseter, -pidOutputPower);
            	}
        	} else if(gamepad.getPOV() == 270) {
        		leftSpeedOffseter = 0.1;
            	if(newHeadingClick) {
            		imu.reset();
            		pid.reset();
            		joystickAngles = -360;
            		newHeadingClick = false;
            	} else {
            		joystickAngles = -360;
            		drive.tankDrive(pidOutputPower + leftSpeedOffseter, -pidOutputPower);
            	}	
        	}
        } else if(gamepad.getRawButton(1))
        {
        	if(newHeadingClick) {
        		imu.reset();
        		pid.reset();
        		newHeadingClick = false;
        	}
        	
        	double leftSpeedOffseter = (rightSpeed > 0) ? 0.05 : -0.06; //ternary operators are great        	
            drive.tankDrive((rightSpeed) + (pidOutputPower * .5) + leftSpeedOffseter, (rightSpeed) + (-pidOutputPower * .5));
            
        } else {
        	newHeadingClick = true;
            drive.tankDrive((leftSpeed * 0.6), rightSpeed * 0.6);
        }
    }
    
    public void disabledPeriodic() {
        run = false;
    }
    
    public void autonomousInit() {
        imu.calibrate();
        imu.reset();
        
    	tof.port.closePort();
    	tof = new TimeOfFlight();
        
        pid = new PIDControl(0.012, 0.000, 0);
        pid.setOutputLimits(-0.5, 0.5);
        pid.setSetpoint(0);
        
        System.out.println("Auto will start now seconds");
        
        run = true;
        
        DriveStraightUntilLessThan(5, 0.6, 800);
        DriveStraightUntilLessThan(2, 0.45, 300);
        //DriveStraight(1, 0.5);
        
        /*
        DriveStraightLogistics(3, 0.5, -55, 1000, 0.5);
        
        waitThread(1200);
        
        DriveStraight(0.1, -0.4); //prime motor voltage
        DriveStraight(0.3, -0.6);
        
        waitThread(500);
        
        DriveTurn(45);
        
        DriveStraight(0.1, 0.4); //prime motor voltage
        DriveStraight(0.2, 0.6);
        
        DriveStraightLogistics(3, 0.5, -60, 1000, 0.5);
        
        DriveStraight(0.1, 0.5); //prime motor voltage
        DriveStraight(2.5, 0.55);
        
        DriveStraightLogistics(2.5, 0.5, 60, 1000, 0.5);
        */
        
        /*
        DriveStraight(2.15, 0.6);
        waitThread(1000);
        DriveTurn(45);
        waitThread(1000);
        DriveStraight(0.5, 0.6);
        waitThread(1000);
        
        DriveStraight(0.5, -0.6);
        waitThread(1000);
        DriveTurn(-45);
        waitThread(1000);
        DriveStraight(2.05, -0.6);
        */
        
        /*
        DriveStraight(0.1, 0.4); //prime motor voltage
        DriveStraight(1.8, 0.6);
        waitThread(100);
        DriveStraight(0.7, 0.55);
        
        waitThread(1000);
        DriveStraight(0.1, -0.55);
        DriveStraight(1.5, -0.6);
        waitThread(100);
        DriveStraight(0.3, -0.55);
        
        
        //move to side for jake
        
        waitThread(2000);
        DriveStraight(0.1, 0.4); //prime motor voltage
        DriveStraight(0.25, 0.5);
        
        waitThread(1000);
        
        DriveTurn(90);
        
        waitThread(500);
        
        DriveStraight(0.1, 0.4); //prime motor voltage
        DriveStraight(1.2, 0.6);
        
        waitThread(1000);
        
        DriveTurn(-90);
        
        waitThread(2000);
        
        DriveTurn(90);
        
        waitThread(2000);
        
        DriveStraight(0.1, -0.4); //prime motor voltage
        DriveStraight(0.6, -0.6);
        
        */
        
        //AutoDriveInSquareRoutine();
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
    
    public void DriveStraightUntilLessThan(double time, double power, double dist) {
    	imu.reset();
        pid.reset();
        
    	double leftOffseter = 0;
    	
    	if(power > 0) {
    		leftOffseter = -0.05;
    	} else{
    		leftOffseter = 0.125;
    	}
    	
        for(int i=0; i<50 * time; i++) {
        	if((tof.GetDataInMillimeters() < dist) && (tof.GetDataInMillimeters() > 0)) {
        		break;
        	}
            double currentAngle = (Math.round(imu.getAngleZ() * 100.0) / 100.0);
            double pidOutputPower = pid.getOutput(currentAngle);
            
            drive.tankDrive(power + (pidOutputPower/4) - leftOffseter, power + (-pidOutputPower/4));
            
            waitThread(20);
        }
        
        drive.tankDrive(0, 0);
    }
    
    public void DriveStraight(double time, double power) {
        imu.reset();
        pid.reset();
        
    	double leftOffseter = 0;
    	
    	if(power > 0) {
    		leftOffseter = -0.05;
    	} else{
    		leftOffseter = 0.125;
    	}
    	
        for(int i=0; i<50 * time; i++) {
            double currentAngle = (Math.round(imu.getAngleZ() * 100.0) / 100.0);
            double pidOutputPower = pid.getOutput(currentAngle);
            
            drive.tankDrive(power + (pidOutputPower/4) - leftOffseter, power + (-pidOutputPower/4));
            
            waitThread(20);
        }
    }
    
    public void DriveTurn(double angle) {
    	pid.reset();
    	imu.reset();
    	
    	double leftOffseter = 0;
    	
    	if(angle > 0) {
    		pid.setOutputLimits(0, 0.7);
    	} else{
    		pid.setOutputLimits(0, -0.7);
    		leftOffseter = 0.125;
    	}
    	
    	pid.setOutputRampRate(0.5);
    	pid.setP(0.015);
    	pid.setD(0.15);
    	pid.setF(.1);
    	
    	int inTheZone = 0;
    	
        double currentAngle = (Math.round(imu.getAngleZ() * 100.0) / 100.0) + (angle * 4) - (lastError * .8);
        double pidOutputPower = pid.getOutput(currentAngle);
        
        System.out.println(pidOutputPower);
        
        while((inTheZone < 50) && run) {
        	currentAngle = (Math.round(imu.getAngleZ() * 100.0) / 100.0) - (angle * 4);
        	pidOutputPower = pid.getOutput(currentAngle);
        	
        	if(Math.abs(currentAngle) < 20) {
        		pid.setOutputLimits(-0.5, 0.5);
        		pid.setI(0.025);
        		pid.setMaxIOutput(0.35);
        		inTheZone++;
        	}
        	
            drive.tankDrive(pidOutputPower + ((pidOutputPower>0) ? (0) : (-leftOffseter)), -pidOutputPower);
            System.out.println(pidOutputPower + " " + currentAngle + " " + (angle*4));
            waitThread(20);
        }
        
        pid.setP(0.012);
        
        lastError = currentAngle;
        pid.setF(0);
        pid.setD(0);
        pid.setI(0);
    }
    
    public double LogisticsCurve(double t, double setPoint, double rate, double midTurn) {
		return setPoint / (1 + Math.exp(-(rate * (t - midTurn))));
    }
    
    public void DriveStraightLogistics(double time, double power, double turnEndPoint, double turnSlope, double turnTime) {
        imu.reset();
        pid.reset();
        
        pid.setP(0.0025);
        pid.setOutputRampRate(0.05);
        
    	double leftOffseter = 0;
    	
    	if(power > 0) {
    		leftOffseter = -0.05;
    	} else{
    		leftOffseter = 0.125;
    	}
    	
        for(int i=0; i<50 * time; i++) {
        	double turnSetpoint;
        	
        	turnSetpoint = LogisticsCurve(i / 50, -turnEndPoint * 4, 2, 1.0);
        	
            double currentAngle = (Math.round(imu.getAngleZ() * 100.0) / 100.0) + turnSetpoint;
            double pidOutputPower = pid.getOutput(currentAngle);
            
            System.out.println(pidOutputPower);
            
            drive.tankDrive(power + (pidOutputPower * .5) - leftOffseter, power + (-pidOutputPower * .5));
            
            waitThread(20);
        }
        
        pid.setOutputRampRate(0);
        pid.setP(0.012);
    }
    
}
