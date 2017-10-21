package ca.mcgill.ecse211.localizationlab;

import ca.mcgill.ecse211.localizationlab.Odometer;
import ca.mcgill.ecse211.localizationlab.LocalizationLab;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;

public class UltrasonicLocalization extends Thread implements UltrasonicController{

	
	private Odometer odometer;
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private EV3MediumRegulatedMotor usMotor;
	private int distance;
	private double deltaTheta;
	private double alpha;
	private double beta;
	private double theta1;
	private double theta2;
	double d = 40;
	double k = 1;
	
	
	public UltrasonicLocalization(Odometer odometer, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, EV3MediumRegulatedMotor usMotor ){
		this.odometer = odometer;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.usMotor = usMotor;
		this.leftMotor.setAcceleration(400);
		this.rightMotor.setAcceleration(400);
	}
	
	public void localize(){
		if(LocalizationLab.fallingEdge == true){
			//rotate counterclockwise until we find value1 to find alpha
			while (distance > d) {
				leftMotor.setSpeed(LocalizationLab.ROTATE_SPEED);
	            rightMotor.setSpeed(LocalizationLab.ROTATE_SPEED);
	            leftMotor.forward();
	            rightMotor.backward();
	        }
			alpha = odometer.getTheta();
			/*System.out.println("");
			System.out.println("");
			System.out.println("");
			System.out.println(alpha);*/
			Sound.beep();
			//find alpha
				
			//rotate clockwise until we find  value1 to find beta
			while(Math.abs(odometer.getTheta()-alpha)<40){
				leftMotor.setSpeed(LocalizationLab.ROTATE_SPEED);
	            rightMotor.setSpeed(LocalizationLab.ROTATE_SPEED);
	            leftMotor.backward();
	            rightMotor.forward();
			}
			while(distance > d){
				leftMotor.setSpeed(LocalizationLab.ROTATE_SPEED);
		        rightMotor.setSpeed(LocalizationLab.ROTATE_SPEED);
		        leftMotor.backward();
		        rightMotor.forward();
			}
			beta = odometer.getTheta();
			//System.out.println(beta);
			Sound.beep();
			//find alpha
			deltaTheta = calculateTheta(alpha,beta);
			odometer.setTheta(odometer.getTheta()+deltaTheta);
			turnTo(0.0);
			leftMotor.stop();
			rightMotor.stop();
			
		}
		
		else if(LocalizationLab.fallingEdge == false){
			//rotate counterclockwise until we find value1 to find alpha
			while (distance < d) {
				leftMotor.setSpeed(LocalizationLab.ROTATE_SPEED);
	            rightMotor.setSpeed(LocalizationLab.ROTATE_SPEED);
	            leftMotor.forward();
	            rightMotor.backward();
	        }
			alpha = odometer.getTheta();
			System.out.println("");
			System.out.println("");
			System.out.println("");
			System.out.println(alpha);
			Sound.beep();
			//find alpha
				
			//rotate clockwise until we find  value1 to find beta
			while(Math.abs(odometer.getTheta()-alpha)<40){
				leftMotor.setSpeed(LocalizationLab.ROTATE_SPEED);
	            rightMotor.setSpeed(LocalizationLab.ROTATE_SPEED);
	            leftMotor.backward();
	            rightMotor.forward();
			}
			while(distance < d){
				leftMotor.setSpeed(LocalizationLab.ROTATE_SPEED);
		        rightMotor.setSpeed(LocalizationLab.ROTATE_SPEED);
		        leftMotor.backward();
		        rightMotor.forward();
			}
			beta = odometer.getTheta();
			System.out.println(beta);
			Sound.beep();
			//find alpha
			deltaTheta = calculateTheta(alpha,beta);
			odometer.setTheta(odometer.getTheta()+deltaTheta);
			turnTo(0.0);
			leftMotor.stop();
			rightMotor.stop();
			
		}
	}
	
	public double calculateTheta (double angleA, double angleB){
		if(LocalizationLab.fallingEdge == false){
			deltaTheta = 45 - (0.5)*(angleA+angleB);
		}
		else{
			deltaTheta = 225 - (0.5)*(angleA+angleB);
		}
		return deltaTheta;
	}

	@Override
	public void processUSData(int distance) {
		this.distance = distance;
		
	}

	@Override
	public int readUSDistance() {
		return this.distance;
	}
	
	void turnTo(double theta){
		double currT = odometer.getTheta();
		double deltaT = theta-currT;
		//makes sure smallest angle
		if(deltaT>180){
			deltaT=deltaT-360;
		}
		else if(deltaT <-180){
			deltaT= deltaT+360;
		}
		// set motors here 
		leftMotor.setSpeed(LocalizationLab.ROTATE_SPEED);
		rightMotor.setSpeed(LocalizationLab.ROTATE_SPEED);
		leftMotor.rotate(convertAngle(LocalizationLab.WHEEL_RADIUS, LocalizationLab.TRACK, deltaT), true);
		rightMotor.rotate(-convertAngle(LocalizationLab.WHEEL_RADIUS, LocalizationLab.TRACK, deltaT), false);

	}
	
	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
	
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}
	
	
}
