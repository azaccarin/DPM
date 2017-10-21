package ca.mcgill.ecse211.localizationlab;

import java.util.ArrayList;

import ca.mcgill.ecse211.localizationlab.Odometer;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class LightLocalization extends Thread{
	
	private Odometer odometer;
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	  private static final Port lsPort = LocalEV3.get().getPort("S2");
	  SensorModes lsSensor = new EV3ColorSensor(lsPort);
	  SampleProvider lsIntensity = lsSensor.getMode("Red");
	  float[] lsData = new float[lsIntensity.sampleSize()];
	  private static final double TILE_WIDTH = 30.48;
	  private boolean isLine = false;
	  private double[] lines = new double[4];
	  private int lineCounter =0;
	  private double xStart;
	  private double yStart;
	  private final double offset = 17;
	  int intensity;
	  int threshold = 20;
	  double deltaT ;
	  
	  public LightLocalization(Odometer odometer, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor){
		  this.leftMotor = leftMotor;
		  this.rightMotor = rightMotor;
		  this.odometer = odometer;
		  this.leftMotor.setAcceleration(400);
		  this.rightMotor.setAcceleration(400);
	  }
	  
	  public void localize(){
		  //turn counter clockwise until both Y lines have been crossed
		  leftMotor.setSpeed(LocalizationLab.ROTATE_SPEED);
		  rightMotor.setSpeed(LocalizationLab.ROTATE_SPEED);
		  leftMotor.rotate(convertAngle(LocalizationLab.WHEEL_RADIUS, LocalizationLab.TRACK, -300), true);
		  rightMotor.rotate(-convertAngle(LocalizationLab.WHEEL_RADIUS, LocalizationLab.TRACK, -300), true);
		  while (lineCounter != 4){
			  lsIntensity.fetchSample(lsData,0);      
		      intensity = (int)(lsData[0]*100.0);
		      if(intensity<threshold){
		    	  Sound.beep();
		    	  lines[lineCounter] = odometer.getTheta();
		    	  lineCounter ++;
		      }
		  } 
		  
		  xStart= -offset*(Math.cos(Math.toRadians((lines[2]-lines[0])*0.5)));
		  odometer.setX(xStart);
		  yStart= -offset*(Math.cos(Math.toRadians((lines[3]-lines[1])*0.5)));
		  odometer.setY(yStart);
		  deltaT = lines[3]-90+(0.5*(lines[2]-lines[0]));
		  while(leftMotor.isMoving()||rightMotor.isMoving()){}
		  System.out.println("");
		  System.out.println("");
		  System.out.println("");
		  System.out.println(odometer.getTheta());
		  System.out.println(deltaT);
		  odometer.setTheta(odometer.getTheta()+deltaT);
		  turnTo(0.0);
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
