/*
18649 Fall 2013
Group 17
Jiang He(jiangh)
Qiang Zhang(qiangz), Shen Yu(sheny), Jiang He(jiangh), Xianle Wang(xianlew)
 */
package simulator.elevatorcontrol;

import jSimPack.SimTime;
import simulator.elevatorcontrol.Utility.AtFloorArray;
import simulator.elevatorcontrol.Utility.DoorClosedArray;
import simulator.elevatormodules.CarLevelPositionCanPayloadTranslator;
import simulator.elevatormodules.HoistwayLimitSensorCanPayloadTranslator;
import simulator.elevatormodules.LevelingCanPayloadTranslator;
import simulator.framework.Controller;
import simulator.framework.Direction;
import simulator.framework.DoorCommand;
import simulator.framework.Hallway;
import simulator.framework.ReplicationComputer;
import simulator.framework.Side;
import simulator.framework.Speed;
import simulator.payloads.CanMailbox;
import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.CanMailbox.WriteableCanMailbox;
import simulator.payloads.DrivePayload;
import simulator.payloads.DrivePayload.WriteableDrivePayload;
import simulator.payloads.DriveSpeedPayload;
import simulator.payloads.DriveSpeedPayload.ReadableDriveSpeedPayload;
import simulator.payloads.translators.BooleanCanPayloadTranslator;

/**
 * DriveControl uses messages from mEmergencyBrake, mHoistwayLimit, mAtFloor, m
 * mCarWeight, mLevel, mDoorClosed, and mDesiredFloor to determine how to drive the car
 *  
 *
 * @author jiangh
 */

public class DriveControl extends Controller {
	//local physical state
	private WriteableDrivePayload localDrive;
	private ReadableDriveSpeedPayload localDriveSpeed; 
    
	//Network Interface
	//mEmergencyBrake
	private ReadableCanMailbox networkEmergencyBrake;
	private BooleanCanPayloadTranslator mEmergencyBrake;
	
	//mHoistwayLimit
	private ReadableCanMailbox networkHoistwayLimitUp;
	private HoistwayLimitSensorCanPayloadTranslator mHoistwayLimitUp;
	private ReadableCanMailbox networkHoistwayLimitDown;
	private HoistwayLimitSensorCanPayloadTranslator mHoistwayLimitDown;
	
	// mCarLevelPosition
	private ReadableCanMailbox networkCarLevelPosition;
	private CarLevelPositionCanPayloadTranslator mCarLevelPosition;
	
	//mAtFloor array
	private AtFloorArray AtFloor_array;
	private DoorClosedArray DoorClosed_front_array;
	private DoorClosedArray DoorClosed_back_array;
	
	//mLevel
	private ReadableCanMailbox networkLevelUp;
	private LevelingCanPayloadTranslator mLevelUp;
	private ReadableCanMailbox networkLevelDown;
	private LevelingCanPayloadTranslator mLevelDown;
	
	//mDoorMotor
	private ReadableCanMailbox networkDoorMotorFrontLeft; 
	private DoorMotorCanPayloadTranslator mDoorMotorFrontLeft;
	private ReadableCanMailbox networkDoorMotorFrontRight; 
	private DoorMotorCanPayloadTranslator mDoorMotorFrontRight;
	private ReadableCanMailbox networkDoorMotorBackLeft; 
	private DoorMotorCanPayloadTranslator mDoorMotorBackLeft;
	private ReadableCanMailbox networkDoorMotorBackRight; 
	private DoorMotorCanPayloadTranslator mDoorMotorBackRight;
	
	//mDesiredFloor
	private ReadableCanMailbox networkDesiredFloor;
	private DesiredFloorCanPayloadTranslator mDesiredFloor;
	
	//mDrive
	private WriteableCanMailbox networkDrive;
	private DriveCommandCanPayloadTranslator mDrive;
	
	//mDriveSpeed
	private WriteableCanMailbox networkDriveSpeed;
	private DriveSpeedCanPayloadTranslator mDriveSpeed;
	
	// mCarWeight
	// private ReadableCanMailbox networkCarWeight;
	// private CarWeightCanPayloadTranslator mCarWeight;
	
	
	//additional internal state variables
	private Speed commandSpeed = Speed.STOP; 
	private Direction desiredDirection = Direction.STOP; 
	private int currentFloor;
	private double desiredFloorHeight;
	
	private int desiredFloor;
	
	//store the period for the controller
	private double slowdownDistance;
	private SimTime period;
	
	// enumerate states
	private enum State {
	  STOP,
	  LEVEL_UP,
	  LEVEL_DOWN,
	  SLOW_UP,
	  SLOW_DOWN,
	  FAST_UP,
	  FAST_DOWN,
	  WAIT
	}
	
	//state variable initialized to the initial state STOP
	private State state = State.STOP;
	
	/**
     * The arguments listed in the .cf configuration file should match the order and
     * type given here.
     *
     * For your elevator controllers, you should make sure that the constructor matches
     * the method signatures in ControllerBuilder.makeAll().
     */
	public DriveControl(SimTime period, boolean verbose) {
		super("DriveControl", verbose);
		
		this.period = period;
		
		log("Created DriveControl with period = ", period);
		
		//initialize physical state
		localDrive = DrivePayload.getWriteablePayload();
		physicalInterface.sendTimeTriggered(localDrive, period);
		
		localDriveSpeed = DriveSpeedPayload.getReadablePayload();
		physicalInterface.registerTimeTriggered(localDriveSpeed);
		
		//initialize network interface
		/*****************************register network message*****************************/
		//mEmergencyBrake
		networkEmergencyBrake = CanMailbox.getReadableCanMailbox(MessageDictionary.EMERGENCY_BRAKE_CAN_ID);
		mEmergencyBrake = new BooleanCanPayloadTranslator(networkEmergencyBrake);
		canInterface.registerTimeTriggered(networkEmergencyBrake);
		
		//mHoistwayLimit
		networkHoistwayLimitUp = CanMailbox.getReadableCanMailbox(MessageDictionary.HOISTWAY_LIMIT_BASE_CAN_ID + ReplicationComputer.computeReplicationId(Direction.UP));
		mHoistwayLimitUp = new HoistwayLimitSensorCanPayloadTranslator(networkHoistwayLimitUp, Direction.UP);
		canInterface.registerTimeTriggered(networkHoistwayLimitUp);
		
		networkHoistwayLimitDown = CanMailbox.getReadableCanMailbox(MessageDictionary.HOISTWAY_LIMIT_BASE_CAN_ID + ReplicationComputer.computeReplicationId(Direction.DOWN));
		mHoistwayLimitDown = new HoistwayLimitSensorCanPayloadTranslator(networkHoistwayLimitDown, Direction.DOWN);
		canInterface.registerTimeTriggered(networkHoistwayLimitDown);
		
		// mCarLevelPosition
		networkCarLevelPosition = CanMailbox.getReadableCanMailbox(MessageDictionary.CAR_LEVEL_POSITION_CAN_ID);
		mCarLevelPosition = new CarLevelPositionCanPayloadTranslator(networkCarLevelPosition);
		canInterface.registerTimeTriggered(networkCarLevelPosition);
		
		//mLevel
		networkLevelUp = CanMailbox.getReadableCanMailbox(MessageDictionary.LEVELING_BASE_CAN_ID + ReplicationComputer.computeReplicationId(Direction.UP));
		mLevelUp = new LevelingCanPayloadTranslator(networkLevelUp, Direction.UP);
		canInterface.registerTimeTriggered(networkLevelUp);
		
		networkLevelDown = CanMailbox.getReadableCanMailbox(MessageDictionary.LEVELING_BASE_CAN_ID + ReplicationComputer.computeReplicationId(Direction.DOWN));
		mLevelDown = new LevelingCanPayloadTranslator(networkLevelDown, Direction.DOWN);
		canInterface.registerTimeTriggered(networkLevelDown);

		//mDoorMotor
		networkDoorMotorFrontLeft = CanMailbox.getReadableCanMailbox(MessageDictionary.DOOR_MOTOR_COMMAND_BASE_CAN_ID + ReplicationComputer.computeReplicationId(Hallway.FRONT, Side.LEFT));
		mDoorMotorFrontLeft = new DoorMotorCanPayloadTranslator(networkDoorMotorFrontLeft, Hallway.FRONT, Side.LEFT);
		canInterface.registerTimeTriggered(networkDoorMotorFrontLeft);

		networkDoorMotorFrontRight = CanMailbox.getReadableCanMailbox(MessageDictionary.DOOR_MOTOR_COMMAND_BASE_CAN_ID + ReplicationComputer.computeReplicationId(Hallway.FRONT, Side.RIGHT));
		mDoorMotorFrontRight = new DoorMotorCanPayloadTranslator(networkDoorMotorFrontRight, Hallway.FRONT, Side.RIGHT);
		canInterface.registerTimeTriggered(networkDoorMotorFrontRight);

		networkDoorMotorBackLeft = CanMailbox.getReadableCanMailbox(MessageDictionary.DOOR_MOTOR_COMMAND_BASE_CAN_ID + ReplicationComputer.computeReplicationId(Hallway.BACK, Side.LEFT));
		mDoorMotorBackLeft = new DoorMotorCanPayloadTranslator(networkDoorMotorBackLeft, Hallway.BACK, Side.LEFT);
		canInterface.registerTimeTriggered(networkDoorMotorBackLeft);

		networkDoorMotorBackRight = CanMailbox.getReadableCanMailbox(MessageDictionary.DOOR_MOTOR_COMMAND_BASE_CAN_ID + ReplicationComputer.computeReplicationId(Hallway.BACK, Side.RIGHT));
		mDoorMotorBackRight = new DoorMotorCanPayloadTranslator(networkDoorMotorBackRight, Hallway.BACK, Side.RIGHT);
		canInterface.registerTimeTriggered(networkDoorMotorBackRight);
		
		
		//mDesiredFloor
		networkDesiredFloor = CanMailbox.getReadableCanMailbox(MessageDictionary.DESIRED_FLOOR_CAN_ID);
		mDesiredFloor = new DesiredFloorCanPayloadTranslator(networkDesiredFloor);
		canInterface.registerTimeTriggered(networkDesiredFloor);
		
		//mDrive
		networkDrive = CanMailbox.getWriteableCanMailbox(MessageDictionary.DRIVE_COMMAND_CAN_ID);
		mDrive = new DriveCommandCanPayloadTranslator(networkDrive);
		canInterface.sendTimeTriggered(networkDrive, period);
		
		//mDriveSpeed
		networkDriveSpeed = CanMailbox.getWriteableCanMailbox(MessageDictionary.DRIVE_SPEED_CAN_ID);
		mDriveSpeed = new DriveSpeedCanPayloadTranslator(networkDriveSpeed);
		canInterface.sendTimeTriggered(networkDriveSpeed, period);
		
		// state variable
		AtFloor_array = new AtFloorArray(canInterface);
		DoorClosed_front_array = new DoorClosedArray(Hallway.FRONT, canInterface);
		DoorClosed_back_array = new DoorClosedArray(Hallway.BACK, canInterface);
		timer.start(period);
	}
	
	/*
     * The timer callback is where the main controller code is executed.  For time
     * triggered design, this consists mainly of a switch block with a case block for
     * each state.  Each case block executes actions for that state, then executes
     * a transition to the next state if the transition conditions are met.
     */
	@Override
	public void timerExpired(Object callbackData) {
		State newState = state;
		int countdown = 0;
		boolean allDoorClosed;
		boolean allDoorMotorStop;
		boolean levelFlag;
		
		// 1st floor is 0m. getDesiredFloor is within [1,8]
		// unit: millimeter
		desiredFloorHeight = 5000 * (getDesiredFloor() - 1);

		switch (state) {
			case STOP:
                //System.out.println("STOP");
				countdown = 10;
				currentFloor = AtFloor_array.getCurrentFloor();
				allDoorClosed = DoorClosed_front_array.getBothClosed() && DoorClosed_back_array.getBothClosed();
				
				allDoorMotorStop = checkAllDoorMotorStop();
				
				//state actions for 'STOP'
				desiredDirection = getDesiredDirection(mDesiredFloor, currentFloor);
				desiredFloor = getDesiredFloor();
				
				commandSpeed = Speed.STOP;
				localDrive.set(commandSpeed, Direction.STOP);//wxl
				mDrive.set(commandSpeed, Direction.STOP);//wxl
				mDriveSpeed.set(localDriveSpeed.speed(), localDriveSpeed.direction());
								 
//#transition 'T6.1'
				 if ((allDoorClosed == false) && (mLevelUp.getValue() == false)) {
					  newState = State.LEVEL_UP;
				 }
//#transition 'T6.3'
				 else if (allDoorClosed == true && allDoorMotorStop == true && (currentFloor != desiredFloor) && desiredDirection == Direction.UP) {
					 newState = State.WAIT;
				 }
//#transition 'T6.5'
				 else if (allDoorClosed == false && (mLevelDown.getValue() == false)) {
						newState = State.LEVEL_DOWN;
				 }
				 else if (allDoorClosed && allDoorMotorStop  == true && (currentFloor != desiredFloor) && desiredDirection == Direction.DOWN) {
					 newState = State.WAIT;
				 } 
				 else {
					  newState = state;
				 }
				 break;
			case WAIT:
				currentFloor = AtFloor_array.getCurrentFloor();
				desiredDirection = getDesiredDirection(mDesiredFloor, currentFloor);
				if (countdown > 0) {
					countdown -= 1;
					newState = state;
//#transition 'T6.7'
				} else if (desiredDirection == Direction.UP) { 
					newState = State.SLOW_UP;
//#transition 'T6.8'
				} else if (desiredDirection == Direction.DOWN) {
					newState = State.SLOW_DOWN;
//#transition 'T6.4'
				} else {					
					newState = State.STOP;
				}
				break;
			case LEVEL_UP:				 			
				 //state actions for 'LEVEL_UP'
				 desiredDirection = Direction.UP;
				 commandSpeed = Speed.LEVEL;
				 localDrive.set(commandSpeed, desiredDirection);
				 mDrive.set(Speed.LEVEL, desiredDirection);
				 mDriveSpeed.set(localDriveSpeed.speed(), localDriveSpeed.direction());				 
//#transition 'T6.2'
				 if ((mLevelDown.getValue() == true && mLevelUp.getValue()==true) || (mEmergencyBrake.getValue()==true) || (mHoistwayLimitDown.getValue()==true) || (mHoistwayLimitUp.getValue()==true)) {
					  newState = State.STOP;				 
				 } else {
					  newState = state;
				 }
				 break;
			case LEVEL_DOWN:
				 //state actions for 'LEVEL_DOWN'
				 desiredDirection = Direction.DOWN;
				 commandSpeed = Speed.LEVEL;
				 localDrive.set(commandSpeed, desiredDirection);
				 mDrive.set(Speed.LEVEL, desiredDirection);
				 mDriveSpeed.set(localDriveSpeed.speed(), localDriveSpeed.direction());
				 
//#transition 'T6.6'
				 if ((mLevelDown.getValue() == true && mLevelUp.getValue()==true) || (mEmergencyBrake.getValue()==true) || (mHoistwayLimitDown.getValue()==true) || (mHoistwayLimitUp.getValue()==true)) {
					  newState = State.STOP;
				 } else {
					  newState = state;
				 }
				 break;
			case SLOW_UP:
                //System.out.println("SLOW_UP");
                slowdownDistance = ((localDriveSpeed.speed() * 1000) * (localDriveSpeed.speed() * 1000))/2000.0 + 800;
				 currentFloor = AtFloor_array.getCurrentFloor();
			    allDoorClosed = DoorClosed_front_array.getBothClosed() && DoorClosed_back_array.getBothClosed();
				 allDoorMotorStop = checkAllDoorMotorStop();
				 levelFlag = mLevelDown.getValue() && mLevelUp.getValue();
				
				 //state actions for 'SLOW_UP'
				 desiredDirection = Direction.UP;
				 desiredFloor = getDesiredFloor();
				 commandSpeed = Speed.SLOW;
				 localDrive.set(commandSpeed, desiredDirection);
				 mDrive.set(commandSpeed, desiredDirection);
				 mDriveSpeed.set(localDriveSpeed.speed(), localDriveSpeed.direction());
				 
//#transition 'T6.9'
				 if (currentFloor == desiredFloor && levelFlag == false) {
					  newState = State.STOP;
				 }
				 else if(mHoistwayLimitDown.getValue()==true || mHoistwayLimitUp.getValue()==true || mEmergencyBrake.getValue()==true) {
					  newState = State.STOP;
				 }
//#transition 'T6.11'
				 else if((Math.abs(desiredFloorHeight - mCarLevelPosition.getPosition()) > slowdownDistance && desiredFloor > 0) && (localDriveSpeed.speed() >= 0.25)) {	 				 
					newState = State.FAST_UP;
				 }
				 else {
					  newState = state;
				 }
				 break;
			case SLOW_DOWN:
                //System.out.println("SLOW_DOWN");
                slowdownDistance = ((localDriveSpeed.speed() * 1000) * (localDriveSpeed.speed() * 1000))/2000.0 + 800;
				 currentFloor = AtFloor_array.getCurrentFloor();
				 allDoorClosed = DoorClosed_front_array.getBothClosed() && DoorClosed_back_array.getBothClosed();
				 allDoorMotorStop = checkAllDoorMotorStop();
				 levelFlag = mLevelDown.getValue() && mLevelUp.getValue();
				
				 //state actions for 'SLOW_DOWN'
				 desiredDirection = Direction.DOWN;
				 desiredFloor = getDesiredFloor();
				 commandSpeed = Speed.SLOW;
				 localDrive.set(commandSpeed, desiredDirection);
				 mDrive.set(commandSpeed, desiredDirection);
				 mDriveSpeed.set(localDriveSpeed.speed(), localDriveSpeed.direction());
				 
//#transition 'T6.10'
				 if (mHoistwayLimitDown.getValue()==true || mHoistwayLimitUp.getValue()==true || mEmergencyBrake.getValue()==true || (currentFloor == desiredFloor && levelFlag == false)) {
					  newState = State.STOP;
				 }
				 else if(mHoistwayLimitDown.getValue()==true || mHoistwayLimitUp.getValue()==true || mEmergencyBrake.getValue()==true) {
					  newState = State.STOP;
				 }
//#transition 'T6.13'
				 else if((Math.abs(desiredFloorHeight - mCarLevelPosition.getPosition()) > slowdownDistance && desiredFloor > 0) && (localDriveSpeed.speed() >= 0.25)) {
					newState = State.FAST_DOWN;
				 }				 
				 else {
					  newState = state;
				 }
				 break;
			case FAST_UP:
			   	 slowdownDistance = ((localDriveSpeed.speed() * 1000) * (localDriveSpeed.speed() * 1000))/2000.0 + 800;
                 //System.out.println("FAST_UP ");
				 currentFloor = AtFloor_array.getCurrentFloor();
			     allDoorClosed = DoorClosed_front_array.getBothClosed() && DoorClosed_back_array.getBothClosed();
				 allDoorMotorStop = checkAllDoorMotorStop();
				 levelFlag = mLevelDown.getValue() && mLevelUp.getValue();
				
				 //state actions for 'FAST_UP'
				 desiredDirection = Direction.UP;
				 desiredFloor = getDesiredFloor();
				 commandSpeed = Speed.FAST;
				 localDrive.set(commandSpeed, desiredDirection);
				 mDrive.set(commandSpeed, desiredDirection);
				 mDriveSpeed.set(localDriveSpeed.speed(), localDriveSpeed.direction());
				 
//#transition 'T6.12'
				 if ((Math.abs(desiredFloorHeight - mCarLevelPosition.getPosition()) <= slowdownDistance && desiredFloor > 0 /* && localDriveSpeed.speed() < 0.25 */) || (mHoistwayLimitDown.getValue()==true || mHoistwayLimitUp.getValue()==true || mEmergencyBrake.getValue()==true)) {
					  newState = State.SLOW_UP;
				 }
				 else {
					  newState = state;
				 }
				 break;
			case FAST_DOWN:
                //System.out.println("FAST_DOWN");
				 slowdownDistance = ((localDriveSpeed.speed() * 1000) * (localDriveSpeed.speed() * 1000))/2000.0 + 800;
				 currentFloor = AtFloor_array.getCurrentFloor();
			     allDoorClosed = DoorClosed_front_array.getBothClosed() && DoorClosed_back_array.getBothClosed();
				 allDoorMotorStop = checkAllDoorMotorStop();
				 levelFlag = mLevelDown.getValue() && mLevelUp.getValue();
				
				 //state actions for 'FAST_DOWN'
				 desiredDirection = Direction.DOWN;
				 desiredFloor = getDesiredFloor();
				 commandSpeed = Speed.FAST;
				 localDrive.set(commandSpeed, desiredDirection);
				 mDrive.set(commandSpeed, desiredDirection);
				 mDriveSpeed.set(localDriveSpeed.speed(), localDriveSpeed.direction());
				 
//#transition 'T6.14'
				 if ((Math.abs(desiredFloorHeight - mCarLevelPosition.getPosition()) <= slowdownDistance && desiredFloor > 0 /* && localDriveSpeed.speed() < 0.25 */) || (mHoistwayLimitDown.getValue()==true || mHoistwayLimitUp.getValue()==true || mEmergencyBrake.getValue()==true)) {
					  newState = State.SLOW_DOWN;
				 }
				 else {
					  newState = state;
				 }
				 break;
			
			default:
				 throw new RuntimeException("State " + state + " was not recognized.");
		}
		
	  //log the results of this iteration
	  if (state == newState) {
			log("remains in state: ",state);
	  } else {
			log("Transition:",state,"->",newState);
	  }

	  //update the state variable
	  state = newState;

	  //report the current state
	  setState(STATE_KEY,newState.toString());

	  //schedule the next iteration of the controller
	  //you must do this at the end of the timer callback in order to restart
	  //the timer
	  timer.start(period);
	}

	private Direction getDesiredDirection(DesiredFloorCanPayloadTranslator mDesiredFloor, int currentFloor) {
		Direction d;
		if(mDesiredFloor.getFloor() > currentFloor) {
			d = Direction.UP;			
		}
		else if(mDesiredFloor.getFloor() < currentFloor) {
			d = Direction.DOWN;			
		}
		else {
			d = Direction.STOP;			
		}			
		
		return d;
	}
	
	private int getDesiredFloor() {
		int df;
		
		if(mDesiredFloor.getFloor() == 0) {
			df = 1;			
		}
		else {
			df = mDesiredFloor.getFloor();		
		}			
		
		return df;
	}
	
	private boolean checkAllDoorMotorStop() {
		return (mDoorMotorFrontLeft.getDoorCommand() == DoorCommand.STOP) && (mDoorMotorFrontRight.getDoorCommand()==DoorCommand.STOP) && (mDoorMotorBackLeft.getDoorCommand()==DoorCommand.STOP) && (mDoorMotorBackRight.getDoorCommand()==DoorCommand.STOP);
	}
}
