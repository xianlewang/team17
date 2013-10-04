/*
18649 Fall 2013
Group 17
Qiang Zhao(qiangz)
(other names would go here)
 */
package simulator.elevatorcontrol;

import jSimPack.SimTime;
import simulator.framework.*;
import simulator.elevatormodules.*;
import simulator.payloads.*;
import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.DoorMotorPayload.WriteableDoorMotorPayload;
import simulator.elevatorcontrol.DesiredFloorCanPayloadTranslator;
import simulator.elevatorcontrol.DriveCommandCanPayloadTranslator;

/*
 * This doorControl uses 
 * <mAtFloor>, <mDesiredFloor>, <mDriveSpeed>,
 * <mDoorOpened>, <mDesiredDwell>, <mCarWeight>
 */
public class DoorControl extends Controller {

    // define state variables
    public static enum State {
        CLOSED,
        BEFORE_OPEN,
        OPENED,
        CLOSING,
    }
    // (in ms) how often this module should send out messages
    private final SimTime period;
    private final Hallway hallway;
    private final Side side;
    // physical interface
    private WriteableDoorMotorPayload door_motor;
    // input network
    private ReadableCanMailbox[] networkAtFloor;
    private AtFloorCanPayloadTranslator[] mAtFloor;
    private ReadableCanMailbox networkDesiredFloor;
    private DesiredFloorCanPayloadTranslator mDesiredFloor;
    private ReadableCanMailbox networkDriveSpeed;
    private DriveCommandCanPayloadTranslator mDriveSpeed;
    private ReadableCanMailbox networkDoorOpened;
    private DoorOpenedCanPayloadTranslator mDoorOpened;
    private ReadableCanMailbox networkDoorClosed;
    private DoorClosedCanPayloadTranslator mDoorClosed;
    private ReadableCanMailbox networkCarWeight;
    private CarWeightCanPayloadTranslator mCarWeight;
    private int mDesiredDwell = 100;
    private int MAX_Weight = 14000;
    // State variables
    private State currentState;
    private int countdown = 0;
    private int Dwell = mDesiredDwell;
    public DoorControl(SimTime period, Hallway hallway, Side side, boolean verbose) {
        super ("DoorControl" + ReplicationComputer.makeReplicationString(hallway, side), verbose);
        
        // set local variables to match constructor parameters
        this.period = period;
        this.side = side;
        this.hallway = hallway;
        this.currentState = State.CLOSED;
        log("Create Door Control with time period = ", period);
        // instantiate physical interface objects
        door_motor = DoorMotorPayload.getWriteablePayload(hallway, side);
        // register the door_motor signal to be sent periodically
        physicalInterface.sendTimeTriggered(door_motor, period);
        
        // instantiate message objects
        networkDesiredFloor = CanMailbox.getReadableCanMailbox(MessageDictionary.DESIRED_FLOOR_CAN_ID);
        networkDriveSpeed = CanMailbox.getReadableCanMailbox(MessageDictionary.DRIVE_COMMAND_CAN_ID);
        networkDoorOpened = CanMailbox.getReadableCanMailbox(MessageDictionary.DOOR_OPEN_SENSOR_BASE_CAN_ID + 
        		ReplicationComputer.computeReplicationId(hallway, side));
        networkCarWeight = CanMailbox.getReadableCanMailbox(MessageDictionary.CAR_WEIGHT_CAN_ID);
        networkDoorClosed = CanMailbox.getReadableCanMailbox(MessageDictionary.DOOR_CLOSED_SENSOR_BASE_CAN_ID + 
        		ReplicationComputer.computeReplicationId(hallway, side));
        mDesiredFloor = new DesiredFloorCanPayloadTranslator(networkDesiredFloor);
        mDriveSpeed = new DriveCommandCanPayloadTranslator(networkDriveSpeed);
        mDoorOpened = new DoorOpenedCanPayloadTranslator(networkDoorOpened, hallway, side);
        mDoorClosed = new DoorClosedCanPayloadTranslator(networkDoorClosed, hallway, side);
        mCarWeight = new CarWeightCanPayloadTranslator(networkCarWeight);
        canInterface.registerTimeTriggered(networkDesiredFloor);
        canInterface.registerTimeTriggered(networkDriveSpeed);
        canInterface.registerTimeTriggered(networkDoorOpened);
        canInterface.registerTimeTriggered(networkDoorClosed);
        canInterface.registerTimeTriggered(networkCarWeight);
        // instantiate an array of AtFloor class 
        networkAtFloor = new ReadableCanMailbox[8];
        mAtFloor = new AtFloorCanPayloadTranslator[8];
        for (int i = 0; i < 8; i++) {
        	networkAtFloor[i] = CanMailbox.getReadableCanMailbox(
        			MessageDictionary.AT_FLOOR_BASE_CAN_ID + 
        			ReplicationComputer.computeReplicationId(i + 1, hallway));
        	canInterface.registerTimeTriggered(networkAtFloor[i]);
        	mAtFloor[i] = new AtFloorCanPayloadTranslator(networkAtFloor[i], i + 1, hallway);
        }
        // ready now
        timer.start(period);
    }
	@Override
	public void timerExpired(Object callbackData) {
		State nextState = currentState;
		switch (nextState) {
			case CLOSED:
				Dwell = mDesiredDwell;
				door_motor.set(DoorCommand.STOP);
				int desFloor = mDesiredFloor.getFloor();
//#transition 'T 5.1'
				if (mAtFloor[desFloor - 1].getValue()) {
					if (mDriveSpeed.getSpeed() == Speed.STOP ||
							mDriveSpeed.getDirection() == Direction.STOP) {
						nextState = State.BEFORE_OPEN;
					}
				}
//#transition 'T 5.6'
				else if (mCarWeight.getWeight() >= MAX_Weight) {
					nextState = State.BEFORE_OPEN;
				} else {
					nextState = currentState;
				}
				break;
			case BEFORE_OPEN:
				door_motor.set(DoorCommand.OPEN);
				countdown = Dwell;
//#transition 'T 5.2'
				if (mDoorOpened.getValue()) {
					nextState = State.OPENED;
				} else {
					nextState = currentState;
				}
				break;
			case OPENED:
				door_motor.set(DoorCommand.STOP);
				if (countdown > 0) {
					nextState = currentState;
					countdown -= 1;
				}
//#transition 'T 5.3'
				else {
					nextState = State.CLOSING;
				}
				break;
			case CLOSING:
				door_motor.set(DoorCommand.NUDGE);
				countdown = 0;
//#transition 'T 5.4'
				if (mDoorClosed.getValue()) {
					nextState = State.CLOSED;
				}
//#transition 'T 5.5'
				else if (mCarWeight.getWeight() >= MAX_Weight) {
					nextState = State.BEFORE_OPEN;
				}
				else {
					nextState = currentState;
				}
				break;
			default:
				throw new RuntimeException("State " + currentState + " wasn't recognized");
		}
		if (currentState == nextState) {
			log("remains in state: ", currentState);
		} else {
			log("Transition: ", currentState, " -> ", nextState);
		}
		// update the state variable
		currentState = nextState;
		// report the current state
		setState(STATE_KEY, nextState.toString());
		
		timer.start(period);
	}
		
}
