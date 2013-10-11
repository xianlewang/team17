/*
18649 Fall 2013
Group 17
Qiang Zhang(qiangz)
(other names would go here)
 */
package simulator.elevatorcontrol;

import jSimPack.SimTime;
import simulator.elevatorcontrol.Utility.AtFloorArray;
import simulator.elevatormodules.AtFloorCanPayloadTranslator;
import simulator.elevatormodules.DoorClosedCanPayloadTranslator;
import simulator.framework.Controller;
import simulator.framework.Direction;
import simulator.framework.Hallway;
import simulator.framework.ReplicationComputer;
import simulator.framework.Side;
import simulator.payloads.CanMailbox;
import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.CarLanternPayload;
import simulator.payloads.CarLanternPayload.WriteableCarLanternPayload;

public class LanternControl extends Controller{
	// define state variables
	public static enum State {
		Lantern_Off,
		No_On,
		Lantern_On,
	}
	// (in ms) how often this module should send out message
	private final SimTime period;
	private final Direction direction;
	// physical interface
	private WriteableCarLanternPayload car_lantern;
	// input network
	private AtFloorArray mAtFloor_array;
	private ReadableCanMailbox networkDesiredFloor;
	private DesiredFloorCanPayloadTranslator mDesiredFloor;
	private ReadableCanMailbox[] networkDoorClosed;
	private DoorClosedCanPayloadTranslator[] mDoorClosed;
	// state variable
	private State currentState;
	Direction DesiredDirection;
	public LanternControl(SimTime period, Direction direction, boolean verbose) {
		super("LanternControl" + ReplicationComputer.makeReplicationString(direction), verbose);
		this.period = period;
		this.direction = direction;
		this.currentState = State.Lantern_Off;
		log("Create CarLantern Control with tim period = ", period);
		// instantiate the physical interface objects
		car_lantern = CarLanternPayload.getWriteablePayload(direction);
		// register the car lantern signal to be sent periodically
		physicalInterface.sendTimeTriggered(car_lantern, period);
		
		// instantiate message objects
		networkDesiredFloor = CanMailbox.getReadableCanMailbox(MessageDictionary.DESIRED_FLOOR_CAN_ID);
		mDesiredFloor = new DesiredFloorCanPayloadTranslator(networkDesiredFloor);
		canInterface.registerTimeTriggered(networkDesiredFloor);
		// instantiate the 4 doors
		networkDoorClosed = new ReadableCanMailbox[4];
		mDoorClosed = new DoorClosedCanPayloadTranslator[4];
		// front left 0; front right 1; back left 2; back right 3
		networkDoorClosed[0] = CanMailbox.getReadableCanMailbox(MessageDictionary.DOOR_CLOSED_SENSOR_BASE_CAN_ID + ReplicationComputer.computeReplicationId(Hallway.FRONT, Side.LEFT));
		networkDoorClosed[1] = CanMailbox.getReadableCanMailbox(MessageDictionary.DOOR_CLOSED_SENSOR_BASE_CAN_ID + ReplicationComputer.computeReplicationId(Hallway.FRONT, Side.RIGHT));
		networkDoorClosed[2] = CanMailbox.getReadableCanMailbox(MessageDictionary.DOOR_CLOSED_SENSOR_BASE_CAN_ID + ReplicationComputer.computeReplicationId(Hallway.BACK, Side.LEFT));
		networkDoorClosed[3] = CanMailbox.getReadableCanMailbox(MessageDictionary.DOOR_CLOSED_SENSOR_BASE_CAN_ID + ReplicationComputer.computeReplicationId(Hallway.BACK, Side.RIGHT));
		// instantiate mDoorClosed
		mDoorClosed[0] = new DoorClosedCanPayloadTranslator(networkDoorClosed[0], Hallway.FRONT, Side.LEFT);
		mDoorClosed[1] = new DoorClosedCanPayloadTranslator(networkDoorClosed[1], Hallway.FRONT, Side.RIGHT);
		mDoorClosed[2] = new DoorClosedCanPayloadTranslator(networkDoorClosed[2], Hallway.BACK, Side.LEFT);
		mDoorClosed[3] = new DoorClosedCanPayloadTranslator(networkDoorClosed[3], Hallway.BACK, Side.RIGHT);
		// register door closed
		canInterface.registerTimeTriggered(networkDoorClosed[0]);
		canInterface.registerTimeTriggered(networkDoorClosed[1]);
		canInterface.registerTimeTriggered(networkDoorClosed[2]);
		canInterface.registerTimeTriggered(networkDoorClosed[3]);
		// instantiate an array of AtFloor class
		mAtFloor_array = new AtFloorArray(canInterface);
		// ready now
		timer.start(period);
	}
	
	@Override
	public void timerExpired(Object callbackData) {
		// TODO Auto-generated method stub
		// cases Lantern_Off, No_On, Turn_Up_On, Turn_Down_On
		State nextState = currentState;
		boolean allClose;
		switch (nextState) {
			case Lantern_Off:
				car_lantern.set(false);
				// code for desired direction for this simple design
				if (mDesiredFloor.getFloor() == 8) {
					DesiredDirection = Direction.STOP;
				} else if (mDesiredFloor.getFloor() < 8 && mDesiredFloor.getFloor() > 1) {
					DesiredDirection = Direction.UP;
				} else {
					DesiredDirection = Direction.DOWN;
				}
				// should be DesiredDirection = mDesiredFloor.getDirection();
				for (int i = 0; i < 4; i++) {
//#transition 'T 7.1'
					if (mDoorClosed[i].getValue() == false && DesiredDirection == Direction.STOP) {
						nextState = State.No_On;
						break;
					}
//#transition 'T 7.2'
					else if (mDoorClosed[i].getValue() == false) {
						nextState = State.Lantern_On;
						break;
					}
					else {
						nextState = State.Lantern_Off;
					}
				}
				break;
			case No_On:
				car_lantern.set(false);
				allClose = true;
				for (int i = 0; i < 4; i++) {
					allClose &= mDoorClosed[i].getValue();
				}
//#transition 'T 7.3'
				if (allClose) {
					nextState = State.Lantern_Off;
				}
				else {
					nextState = State.No_On;
				}
				break;
			case Lantern_On:
				car_lantern.set(true);
				allClose = true;
				for (int i = 0; i < 4; i++) {
					allClose &= mDoorClosed[i].getValue();
				}
//#transition 'T 7.4'
				if (allClose) {
					nextState = State.Lantern_Off;
				}
				else {
					nextState = State.Lantern_On;
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
