/*
18649 Fall 2013
Group 17
Jiang He (jiangh)
*/

package simulator.elevatorcontrol;

import jSimPack.SimTime;
import simulator.elevatormodules.AtFloorCanPayloadTranslator;
import simulator.elevatormodules.DoorClosedCanPayloadTranslator;
import simulator.elevatormodules.HoistwayLimitSensorCanPayloadTranslator;
import simulator.elevatormodules.LevelingCanPayloadTranslator;
import simulator.framework.Controller;
import simulator.framework.Direction;
import simulator.framework.Hallway;
import simulator.framework.ReplicationComputer;
import simulator.framework.Side;
import simulator.framework.Speed;
import simulator.payloads.CanMailbox;
import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.CanMailbox.WriteableCanMailbox;
import simulator.payloads.DrivePayload;
import simulator.payloads.DrivePayload.WriteableDrivePayload;
import simulator.payloads.EmergencyBrakePayload.ReadableEmergencyBrakePayload;
import simulator.payloads.HoistwayLimitPayload.ReadableHoistwayLimitPayload;
import simulator.payloads.translators.BooleanCanPayloadTranslator;

public class DriveControl extends Controller {
	// private Direction desiredDirection;
	// private Direction movingDirection;
	// private int currentFloor;
	
	//private boolean frontDoorClosed;
	//private boolean BackDoorClosed;
	
	
	//local physical state
	private WriteableDrivePayload localDrive;
    
	//network interface
	//emergency brake
	// private ReadableCanMailbox networkEmergencyBrake;
	// private BooleanCanPayloadTranslator mEmergencyBrake;
	
	//hoistway limit
	// private ReadableCanMailbox networkHoistwayLimitUp;
	// private HoistwayLimitSensorCanPayloadTranslator mHoistwayLimitUp;
	// private ReadableCanMailbox networkHoistwayLimitDown;
	// private HoistwayLimitSensorCanPayloadTranslator mHoistwayLimitDown;
	
	//at floor
	private ReadableCanMailbox[] networkAtFloor;
	private AtFloorCanPayloadTranslator[] mAtFloor;
	// private ReadableCanMailbox networkAtFloorFront;
	// private AtFloorCanPayloadTranslator mAtFloorFront;
	// private ReadableCanMailbox networkAtFloorBack;
	// private AtFloorCanPayloadTranslator mAtFloorBack;
	
	//level
	// private ReadableCanMailbox networkLevelUp;
	// private LevelingCanPayloadTranslator mLevelUp;
	// private ReadableCanMailbox networkLevelDown;
	// private LevelingCanPayloadTranslator mLevelDown;
	
	//door closed
	private ReadableCanMailbox networkDoorClosed;
	private DoorClosedCanPayloadTranslator mDoorClosed;
	// private ReadableCanMailbox networkDoorClosedFront;
	// private BooleanCanPayloadTranslator mDoorClosedFront;
	// private ReadableCanMailbox networkDoorClosedBack;
	// private BooleanCanPayloadTranslator mDoorClosedBack;
	
	//desired floor
	private ReadableCanMailbox networkDesiredFloor;
	private DesiredFloorCanPayloadTranslator mDesiredFloor;
	
	//mDrive
	// private WriteableCanMailbox networkDrive;
	// private DriveCommandCanPayloadTranslator mDrive;
	
	//mDriveSpeed
	// private WriteableCanMailbox networkDriveSpeed;
	// private DriveCommandCanPayloadTranslator mDriveSpeed;
	
	//store the period for the controller
	private SimTime period;
    
	//enumerate states
	// private enum State {
	  // STOP,
	  // LEAVING_FLOOR,
	  // APPROACHING_FLOOR,
	  // IN_THE_MIDDLE_OF_FLOOR,
	// }
	
	private enum State {
	  STOP,
	  MOVING,
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
		
		//initialize network interface
		// networkEmergencyBrake = CanMailbox.getReadableCanMailbox(MessageDictionary.EMERGENCY_BRAKE_CAN_ID);
		// mEmergencyBrake = new BooleanCanPayloadTranslator(networkEmergencyBrake);
		// canInterface.registerTimeTriggered(networkEmergencyBrake);
		
		//mHoistwayLimit
		//?????direction replication???
		// networkHoistwayLimitUp = CanMailbox.getReadableCanMailbox(MessageDictionary.HOISTWAY_LIMIT_BASE_CAN_ID + ReplicationComputer.computeReplicationId(Direction.UP));
		// mHoistwayLimitUp = new HoistwayLimitSensorCanPayloadTranslator(networkHoistwayLimitUp, Direction.UP);
		// canInterface.registerTimeTriggered(networkHoistwayLimitUp);
		
		// networkHoistwayLimitDown = CanMailbox.getReadableCanMailbox(MessageDictionary.HOISTWAY_LIMIT_BASE_CAN_ID + ReplicationComputer.computeReplicationId(Direction.DOWN));
		// mHoistwayLimitDown = new HoistwayLimitSensorCanPayloadTranslator(networkHoistwayLimitDown, Direction.DOWN);
		// canInterface.registerTimeTriggered(networkHoistwayLimitDown);
		
		//mAtFloor
		// instantiate an array of AtFloor class 
		networkAtFloor = new ReadableCanMailbox[8];
		mAtFloor = new AtFloorCanPayloadTranslator[8];
		for (int i = 0; i < 8; i++) {
        	networkAtFloor[i] = CanMailbox.getReadableCanMailbox(
        			MessageDictionary.AT_FLOOR_BASE_CAN_ID + 
        			ReplicationComputer.computeReplicationId(i + 1, Hallway.FRONT));
        	canInterface.registerTimeTriggered(networkAtFloor[i]);
        	mAtFloor[i] = new AtFloorCanPayloadTranslator(networkAtFloor[i], i + 1, Hallway.FRONT);
        }
		  
		// System.out.println("currentFloor is " + currentFloor);
		// networkAtFloorFront = CanMailbox.getReadableCanMailbox(MessageDictionary.AT_FLOOR_BASE_CAN_ID + ReplicationComputer.computeReplicationId(currentFloor, Hallway.FRONT));
		// mAtFloorFront = new AtFloorCanPayloadTranslator(networkAtFloorFront, currentFloor, Hallway.FRONT);
		// canInterface.registerTimeTriggered(networkAtFloorFront);
		
		// networkAtFloorBack = CanMailbox.getReadableCanMailbox(MessageDictionary.AT_FLOOR_BASE_CAN_ID + ReplicationComputer.computeReplicationId(currentFloor, Hallway.BACK));
		// mAtFloorBack = new AtFloorCanPayloadTranslator(networkAtFloorBack, currentFloor, Hallway.BACK);
		// canInterface.registerTimeTriggered(networkAtFloorBack);
		
		//mLevel
		// networkLevelUp = CanMailbox.getReadableCanMailbox(MessageDictionary.LEVELING_BASE_CAN_ID + ReplicationComputer.computeReplicationId(Direction.UP));
		// mLevelUp = new LevelingCanPayloadTranslator(networkLevelUp, Direction.UP);
		// canInterface.registerTimeTriggered(networkLevelUp);
		
		// networkLevelDown = CanMailbox.getReadableCanMailbox(MessageDictionary.LEVELING_BASE_CAN_ID + ReplicationComputer.computeReplicationId(Direction.DOWN));
		// mLevelDown = new LevelingCanPayloadTranslator(networkLevelDown, Direction.DOWN);
		// canInterface.registerTimeTriggered(networkLevelDown);
		
		//mDoorClosed
		networkDoorClosed = CanMailbox.getReadableCanMailbox(MessageDictionary.DOOR_CLOSED_SENSOR_BASE_CAN_ID + ReplicationComputer.computeReplicationId(Hallway.FRONT, Side.RIGHT));
		mDoorClosed = new DoorClosedCanPayloadTranslator(networkDoorClosed, Hallway.FRONT, Side.RIGHT);
		canInterface.registerTimeTriggered(networkDoorClosed);
		
		// networkDoorClosedBack = CanMailbox.getReadableCanMailbox(MessageDictionary.DOOR_CLOSED_SENSOR_BASE_CAN_ID);
		// mDoorClosed = new DoorClosedCanPayloadTranslator(networkDoorClosedBack);
		// canInterface.registerTimeTriggered(networkDoorClosedBack);
		
		//mDesiredFloor
		networkDesiredFloor = CanMailbox.getReadableCanMailbox(MessageDictionary.DESIRED_FLOOR_CAN_ID);
		mDesiredFloor = new DesiredFloorCanPayloadTranslator(networkDesiredFloor);
		canInterface.registerTimeTriggered(networkDesiredFloor);
		
		//mDrive
		// networkDrive = CanMailbox.getWriteableCanMailbox(MessageDictionary.DRIVE_COMMAND_CAN_ID);
		// mDrive = new DriveCommandCanPayloadTranslator(networkDrive);
		// canInterface.sendTimeTriggered(networkDrive, period);
		
		//mDriveSpeed
		// networkDriveSpeed = CanMailbox.getWriteableCanMailbox(MessageDictionary.DRIVE_COMMAND_CAN_ID);
		// mDriveSpeed = new DriveCommandCanPayloadTranslator(networkDriveSpeed);
		// canInterface.sendTimeTriggered(networkDriveSpeed, period);
		
		//this.currentFloor = getCurrentFloor();
		//this.desiredDirection = getDesiredDirection(mDesiredFloor, currentFloor);
		//this.movingDirection = getMovingDirection(mLevelUp, mLevelDown);
		
		timer.start(period);
	}
	
	/*
     * The timer callback is where the main controller code is executed.  For time
     * triggered design, this consists mainly of a switch block with a case block for
     * each state.  Each case block executes actions for that state, then executes
     * a transition to the next state if the transition conditions are met.
     */
	public void timerExpired(Object callbackData) {
		State newState = state;
		int tempCurrFloor;
		// currentFloor = getCurrentFloor();
		// desiredDirection = getDesiredDirection(mDesiredFloor, currentFloor);
		// movingDirection = getMovingDirection(mLevelUp, mLevelDown);
		
		/**********************************
		switch (state) {
			case STOP:
				 //state actions for 'STOP'
				 localDrive.set(Speed.STOP, Direction.STOP);
				 mDrive.set(Speed.STOP, Direction.STOP);
				 // mDriveSpeed.set(Speed.STOP, Direction.STOP);
				 
				 //#transition 'T6.2'
				 if ((currentFloor != mDesiredFloor.getFloor()) && (mDoorClosed.getValue() == true)) {
					  newState = State.LEAVING_FLOOR;
				 } 
				 else {
					  newState = state;
				 }
				 break;
			case APPROACHING_FLOOR:
				 //state actions for 'APPROACHING FLOOR'
				 localDrive.set(Speed.SLOW, movingDirection);
				 mDrive.set(Speed.SLOW, movingDirection);
				 // mDriveSpeed.set(Speed.SLOW, movingDirection);
				 				 
				 //#transition 'T6.1'
				 if ((mAtFloorFront.getValue() == true || mAtFloorBack.getValue() == true) && 
						(mLevelUp.getValue() == true || mLevelDown.getValue() == true) && 
						(currentFloor != mDesiredFloor.getFloor())) {
					  newState = State.STOP;
				 //#transition 'T6.7
				 } else if (mEmergencyBrake.getValue() == true) {
					  newState = State.STOP;
				 } else {
					  newState = state;
				 }
				 break;
			case LEAVING_FLOOR:
				 //state actions for 'LEAVING FLOOR'
				 localDrive.set(Speed.SLOW, desiredDirection);
				 mDrive.set(Speed.SLOW, desiredDirection);
				 // mDriveSpeed.set(Speed.SLOW, desiredDirection);
				 
				 //#transition 'T6.3'
				 if (mAtFloorFront.getValue() == false || mAtFloorBack.getValue() == false) {
					  newState = State.IN_THE_MIDDLE_OF_FLOOR;
				 //#transition 'T6.8 
				 } else if (mEmergencyBrake.getValue() == true) {
					  newState = State.STOP;
				 } else {
					  newState = state;
				 }
				 break;
			case IN_THE_MIDDLE_OF_FLOOR:
				 //state actions for 'IN THE MIDDLE OF FLOOR'
				 localDrive.set(Speed.SLOW, desiredDirection);
				 mDrive.set(Speed.SLOW, desiredDirection);
				 // mDriveSpeed.set(Speed.SLOW, desiredDirection);
				 
				 //#transition 'T6.4'
				 if (mAtFloorFront.getValue() == true || mAtFloorBack.getValue() == true) {
					  newState = State.APPROACHING_FLOOR;
				 //#transition 'T6.5'
				 } else if (mEmergencyBrake.getValue() == true) {
					  newState = State.STOP;
			    //#transition 'T6.6'
				 } else if (mHoistwayLimitUp.getValue() == true || mHoistwayLimitDown.getValue() == true) {
					  newState = State.STOP;
				 } else {
					  newState = state;
				 }
				 break;
			default:
				 throw new RuntimeException("State " + state + " was not recognized.");
		}
		*****************************/
		switch (state) {
			case STOP:
				 //state actions for 'STOP'
				 localDrive.set(Speed.STOP, Direction.STOP);
				 // mDrive.set(Speed.STOP, Direction.STOP);
				 // mDriveSpeed.set(Speed.STOP, Direction.STOP);
				 tempCurrFloor = getCurrentFloor();
//#transition 'T6.1'
				 if ((tempCurrFloor != mDesiredFloor.getFloor()) && (mDoorClosed.getValue() == true)) {
					  newState = State.MOVING;
				 } 
				 else {
					  newState = state;
				 }
				 break;
			case MOVING:
				 //state actions for 'MOVING'
				 localDrive.set(Speed.SLOW, mDesiredFloor.getDirection());
				 // mDrive.set(Speed.SLOW, mDesiredFloor.getDirection());
				 // mDriveSpeed.set(Speed.SLOW, movingDirection);				 				 
				 // System.out.println("mDrive.getDirection() is " + mDrive.getDirection());
				 // System.out.println("mDrive.getSpeed() is " + mDrive.getSpeed());
				 
				 boolean atFLoorFlag = false;
				 tempCurrFloor = 0;
				 for(int i = 0; i < 8; i++) {
					 atFLoorFlag = atFLoorFlag || mAtFloor[i].getValue();
					 if(atFLoorFlag) {
						 tempCurrFloor = i + 1;
						 break;
					 }					 
				 }
				 
//#transition 'T6.2'
				 if ((atFLoorFlag == true) && (tempCurrFloor == mDesiredFloor.getFloor())) {
					  newState = State.STOP;				 
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
	
	private int getCurrentFloor() {
		int i;
		int currentFloor = 1;
		boolean atFLoorFlag = false;
		
		for(i = 0; i < 8; i++) {
		 atFLoorFlag = atFLoorFlag || mAtFloor[i].getValue();
		 if(atFLoorFlag) {
			 currentFloor = i + 1;
			 break;
			}					 
		}
		
		// Utility.AtFloorArray u = new Utility.AtFloorArray(canInterface);			
		// return u.getCurrentFloor();
		return currentFloor;
	}
	
	// private Direction getDesiredDirection(DesiredFloorCanPayloadTranslator mDesiredFloor, int CurrentFloor) {
		// Direction desiredDirection;
		// if(mDesiredFloor.getFloor() > CurrentFloor) {
			// desiredDirection = Direction.UP;			
		// }
		// else if(mDesiredFloor.getFloor() < CurrentFloor) {
			// desiredDirection = Direction.DOWN;			
		// }
		// else {
			// desiredDirection = Direction.STOP;			
		// }			
		
		// return desiredDirection;
	// }
	
	// private Direction getMovingDirection(LevelingCanPayloadTranslator mLevelUp, LevelingCanPayloadTranslator mLevelDown) {
		// Direction movingDirection = Direction.STOP;
		
		// if(mLevelUp.getValue() == true) {
			// movingDirection = Direction.UP;			
		// }
		// else if(mLevelDown.getValue() == true) {
			// movingDirection = Direction.DOWN;				
		// }
		// return movingDirection;
	// }
	
	// private boolean checkFrontDoorClosed() {
		// Utility.DoorClosedArray u = new Utility.DoorClosedArray(Hallway.FRONT, canInterface);
		// return u.getBothClosed();
	// }
}
