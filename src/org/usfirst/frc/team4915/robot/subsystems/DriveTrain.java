package org.usfirst.frc.team4915.robot.subsystems;

import com.ctre.CANTalon.FeedbackDeviceStatus;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.command.Subsystem;
import org.usfirst.frc.team4915.robot.commands.DriveForwardCommand;
import org.usfirst.frc.team4915.robot.Logger;
import org.usfirst.frc.team4915.robot.RobotMap;
import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;


/**
 *
 */
public class DriveTrain extends Subsystem {
    private StringBuilder _sb = new StringBuilder();
    private Logger m_logger = new Logger("Drivetrain", Logger.Level.DEBUG);
    
    /**
     * Encoder is connected directly into the wheel shaft -- 1:1 scaling encoder values
     * Encoder is 250 cycles per revolution. Multiply by 4 to get 1000 encoder ticks per wheel revolution at Talon.
     * 
     * @TODO: Verify experimentally by rotating a wheel 360 degrees and comparing before and after tick counts.
     * Target and sampled position is passed into the equation in native units (i.e. ticks), unless QuadratureEncoder correctly
     * configured with configEncoderCodesPerRev() [TalonSRX Programming 17.2.1]
      */
    // CPR value for our encoder E4P-250-250-N-S-D-D
    private static final int QUAD_ENCODER_CYCLES_PER_REVOLUTION = 250; 
    // Full rotation in ticks -- this is the native units
    private static final int QUAD_ENCODER_TICKS_PER_REVOLUTION = QUAD_ENCODER_CYCLES_PER_REVOLUTION * 4; 
    // Number of ticks per inch -- we have 6" wheels
    private static final int QUAD_ENCODER_TICKS_PER_INCH = (int)(QUAD_ENCODER_TICKS_PER_REVOLUTION/(Math.PI*6));
    // Circumference of our wheel
    private static final double WHEEL_CIRCUMFERENCE = (Math.PI*6);
    
    private static final int ALLOWED_CLOSED_LOOP_ERROR = 50;
    
    // Port motors
    private CANTalon m_portFollowerMotor;
    private static CANTalon m_portMasterMotor;

    // Starboard motors
    private CANTalon m_starboardFollowerMotor;
    private static CANTalon m_starboardMasterMotor;

    // Note: since the motors are grouped together, we only need to control the Masters
    // Used for debugging
    public static List<CANTalon> motors =
            Arrays.asList(m_portMasterMotor, m_starboardMasterMotor);

    public DriveTrain()
    {
        // Create new CANTalons for all our drivetrain motors
        m_portFollowerMotor = new CANTalon(RobotMap.DRIVE_TRAIN_MOTOR_PORT_FOLLOWER);
        m_portMasterMotor = new CANTalon(RobotMap.DRIVE_TRAIN_MOTOR_PORT_MASTER);
        m_starboardFollowerMotor = new CANTalon(RobotMap.DRIVE_TRAIN_MOTOR_STARBOARD_FOLLOWER);
        m_starboardMasterMotor = new CANTalon(RobotMap.DRIVE_TRAIN_MOTOR_STARBOARD_MASTER);

        // Set the Master motor to a control mode and make the follower a follower
        m_portMasterMotor.changeControlMode(TalonControlMode.Position);
        m_portFollowerMotor.changeControlMode(TalonControlMode.Follower);
        m_portFollowerMotor.set(m_portMasterMotor.getDeviceID()); // Sets the master motor for the follower

        // Sets the Master motor to a control mode and make a follower a follower
        m_starboardMasterMotor.changeControlMode(TalonControlMode.Position);
        m_starboardFollowerMotor.changeControlMode(TalonControlMode.Follower);
        m_starboardFollowerMotor.set(m_starboardMasterMotor.getDeviceID()); // Sets the master motor for the follower

        /*
         * Once a “Feedback Device” is selected, the “Sensor Position” and “Sensor Velocity” 
         * signals will update with the output of the selected feedback device. It will also be 
         * multiplied by (-1) if “Reverse Feedback Sensor" is asserted programmatically.
         * [TalonSRX programming 7]
         */
        m_portMasterMotor.setFeedbackDevice(FeedbackDevice.QuadEncoder);
        m_starboardMasterMotor.setFeedbackDevice(FeedbackDevice.QuadEncoder);

        m_portMasterMotor.setInverted(false); // Set direction so that the port motor is *not* inverted
        m_starboardMasterMotor.setInverted(false); // Set direction so that the starboard motor is *not* inverted
        
        /*
         * Typically reverseSensor() is enough to keep sensor in-phase w/ motor
         * reverseOutput() reverses the output of the closed-loop math as an alternative to flip motor direction
         * TalonSRX programming section 7.4
         */
        m_starboardMasterMotor.reverseOutput(true);
        
        m_portMasterMotor.reverseSensor(false);      // @TODO verify that sensor is in-phase w/ the motor
        m_starboardMasterMotor.reverseSensor(true); // @TODO verify that sensor is in-phase w/ the motor

        // Set the number of encoder ticks per wheel revolution -- used for unit scaling (rotations & RPM)
        m_portMasterMotor.configEncoderCodesPerRev(QUAD_ENCODER_CYCLES_PER_REVOLUTION);
        m_starboardMasterMotor.configEncoderCodesPerRev(QUAD_ENCODER_CYCLES_PER_REVOLUTION);

        
        // Enable break mode
        m_portMasterMotor.enableBrakeMode(true);
        m_starboardMasterMotor.enableBrakeMode(true);

        // Reset the encoder position
        m_portMasterMotor.setEncPosition(0);
        m_starboardMasterMotor.setEncPosition(0);
       
        // Setup motor control profile parameters
        /* 
         * Set PID values
         * If you want your mechanism to drive 50% throttle (0-1023) when the error is 1000 (one rotation)
         * Proportional Gain = (0.50 x 1023)/1000 = ~0.511 [TalonSRX programming 10.1]
         * IMPORTANT: you can use the roboRIO utility to tune the PID instead of coding it here
         */
        m_portMasterMotor.setPID(1.0, 0, 0); 
        m_starboardMasterMotor.setPID(1.0, 0, 0); 
        
        /*
         *  Set closed loop error
         *  dictates where motor output is neutral, regardless of calculated results
         *  when closed-loop error is within allowable closed-loop error, P,I,D terms are zeroed
         *  if allowable closed loop error is 28 then ((28/(250*4))*360) is ~10 degree error
         */
        m_portMasterMotor.setAllowableClosedLoopErr(ALLOWED_CLOSED_LOOP_ERROR);    // default is 0
        m_starboardMasterMotor.setAllowableClosedLoopErr(ALLOWED_CLOSED_LOOP_ERROR);

        /*
         * Bounds the output of the closed-loop modes [TalonSRX 10.5]
         * peak output: maximum/strongest motor output allowed during closed-loop
         *      - APIs available to configure forward / reverse peak output
         * nominal output: minimal/weakest output allowed during closed-loop
         *  nominal output ensures that if the closed-loop values are too too weak, motor output is 
         *  large enough to drive the robot. 
         *      - APIs available to configure forward / reverse nominal output
         *  Note: in native units, these represent -1023 (full reverse; -12V) to +1023 (full forward; 12V)
         */
        m_portMasterMotor.configNominalOutputVoltage(+1.0f,  -1.0f);
        m_starboardMasterMotor.configNominalOutputVoltage(+1.0f, -1.0f);
        
        m_portMasterMotor.configPeakOutputVoltage(+3.0f, -3.0f);
        m_starboardMasterMotor.configPeakOutputVoltage(+3.0f, -3.0f);
        
        // max allowable voltage change /sec: reach to 12V after .25sec
        m_portMasterMotor.setVoltageRampRate(48.0);
        m_starboardMasterMotor.setVoltageRampRate(48.0);

        // closed-loop ramp rate
        m_portMasterMotor.setCloseLoopRampRate(48.0);
        m_starboardMasterMotor.setCloseLoopRampRate(48.0);
    }
    
	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		//setDefaultCommand(new DriveForwardCommand());
		
		// Debug for sensor health
		FeedbackDeviceStatus status = m_portMasterMotor.isSensorPresent(FeedbackDevice.QuadEncoder);
		switch (status)
		{
		    case FeedbackStatusPresent:
		        System.out.println("FeedbackStatusPresent");
		        break;
		    case FeedbackStatusNotPresent:
                System.out.println("FeedbackStatusNotPresent");
                break;
		    case FeedbackStatusUnknown:
                System.out.println("FeedbackStatusUnknown");
                break;
        }
	}
	
	/*
	 * Examples use set in rotations -- need to ensure CPR is enabled correctly for rotations
	 * Documentation maybe in correct == In Position mode, distance is in encoder ticks or an analog value, depending on the sensor.
	 *     distance = in inches
	 */
	public void driveInDistance(double distanceInInches)
	{
		double rotations = m_inchesToRotations(distanceInInches);
		double portGet = m_portMasterMotor.get();
		double starGet = m_starboardMasterMotor.get();
		double portPosition = m_portMasterMotor.getPosition();
		double starPosition = m_starboardMasterMotor.getPosition();
		int portEncPosition = m_portMasterMotor.getEncPosition();
		int starEncPosition = m_starboardMasterMotor.getEncPosition();
		
        m_portMasterMotor.set(rotations);
        m_starboardMasterMotor.set(rotations);
        
        //m_logger.info("Get:\t" + portGet + ", " + starGet);
        m_logger.info("GetPosition:\t" + portPosition + ", " + starPosition);
        m_logger.info("GetEncPosition:\t" + portEncPosition + ", " + starEncPosition);
	}
	
   public void driveInDistance(double portDistance, double starboardDistance)
    {
        m_portMasterMotor.set(m_inchesToRotations(portDistance));
        m_starboardMasterMotor.set(m_inchesToRotations(starboardDistance));
    }

   public void stop()
   {
       //m_portMasterMotor.set(0.0);
       //m_starboardMasterMotor.set(0.0);
   }
   
   public boolean withinAllowableClosedLoopError()
   {
	   int portError = m_portMasterMotor.getClosedLoopError();
	   int starError = m_starboardMasterMotor.getClosedLoopError();
	   m_logger.info("EncError: " + portError + ", " + starError);
	   
       if ((portError <= ALLOWED_CLOSED_LOOP_ERROR) && (starError <= ALLOWED_CLOSED_LOOP_ERROR))
       {
           return true;
       }
       return false;
   }

	/*
	 * Useful helper functions
	 */
    private int m_inchesToTicks(double inches)
    {
        return (int)(inches * QUAD_ENCODER_TICKS_PER_INCH); 
    }
    
    private int m_feetToTicks(double feet) {
        return (int)(m_inchesToTicks(feet * 12.0)); 
    }
    
    private double m_inchesToRotations(double inches) {
        return inches/WHEEL_CIRCUMFERENCE; 
    }
    
    /*
     * Zero's encoder values: masters are the only motors w/ encoders!
     * difference between setEncPosition() setPosition() is update rate. 
     *  - setEncPosition() updated every 100ms
     *  - setPosition() updated every 20ms
     */
    public void resetEncPosition() {
        m_portMasterMotor.setEncPosition(0);
        m_starboardMasterMotor.setEncPosition(0);
      
        try
        {
            Thread.sleep(100);
        }
        catch (InterruptedException e)
        {
            // TODO Auto-generated catch block
            e.printStackTrace();
        } /* wait a bit to make sure the setPosition() above takes effect */
    }
  
    public void resetPosition() throws InterruptedException {
        m_portMasterMotor.setPosition(0);
        m_starboardMasterMotor.setPosition(0);
      
        Thread.sleep(100); /* wait a bit to make sure the setPosition() above takes effect */
    }

    public boolean isEncPositionReset() {
        if (m_portMasterMotor.getEncPosition() == 0 && 
            m_starboardMasterMotor.getEncPosition() == 0) {
            return true;
        }
        return false;
    }
	
    public boolean isPositionReset() {
        if (m_portMasterMotor.getPosition() == 0 && 
            m_starboardMasterMotor.getPosition() == 0) {
            return true;
        }
        return false;
    }

    /*
     * Various debug values relating the drive train motors 
     */
    public void smartDashboardDebugDisplay() {
        double motorOutput; 
        
        for (int i=0; i < motors.size(); i++) {
            motorOutput  = motors.get(i).getOutputVoltage() / motors.get(i).getBusVoltage();
               
            _sb.setLength(0);
            _sb.append(String.format("Master[%d] getControlMode: ", motors.get(i).getDeviceID()));
            _sb.append(motors.get(i).getControlMode());
            System.out.println("Master ControlMode: " + _sb);
            SmartDashboard.putString("Master ControlMode: ", _sb.toString());  
        
            _sb.setLength(0);
            _sb.append(String.format("Master[%d] motorOutput: ", motors.get(i).getDeviceID()));
            _sb.append(String.format("%.2f", motorOutput));
            //_sb.append("\tmaxSpeed: ");
            //_sb.append(_maxSpeed);
            //_sb.append("\ttargetSpeed: ");
            //_sb.append(_targetSpeed);
            _sb.append("\tclosedLoopError: ");
            _sb.append(motors.get(i).getClosedLoopError());
            _sb.append("\tgetSpeed(): ");
            _sb.append(String.format("%.2f", motors.get(i).getSpeed())); 
            _sb.append("\tgetSetPoint(): ");
            _sb.append(String.format("%.2f", motors.get(i).getSetpoint()));
            _sb.append("\tgetPosition(): ");
            _sb.append(String.format("%.2f", motors.get(i).getPosition()));            
            _sb.append("\tget(): ");
            _sb.append(String.format("%.2f", motors.get(i).get()));
            SmartDashboard.putString("Info: ", _sb.toString());  
            System.out.println("Info: " + _sb.toString());
        }
        _sb.setLength(0);
        _sb.append("m_portMasterMotor getControlMode: ");
        _sb.append(m_portMasterMotor.getControlMode());
        SmartDashboard.putString("Followers ControlMode Info: ", _sb.toString());      
       
    }
}
