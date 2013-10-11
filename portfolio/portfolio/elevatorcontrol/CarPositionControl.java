/*
 18649 Fall 2013
 Group 17
 Shen Yu(sheny)
 (other names would go here)
 */
package simulator.elevatorcontrol;

import jSimPack.SimTime;
import simulator.elevatormodules.CarLevelPositionCanPayloadTranslator;
import simulator.framework.Controller;
import simulator.framework.Hallway;
import simulator.payloads.CanMailbox;
import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.CarPositionIndicatorPayload;
import simulator.payloads.CarPositionIndicatorPayload.WriteableCarPositionIndicatorPayload;
import simulator.elevatorcontrol.Utility.AtFloorArray;;

public class CarPositionControl extends Controller{
	/***************************************************************************
     * Declarations
     **************************************************************************/
    //note that inputs are Readable objects, while outputs are Writeable objects
	//input
	private AtFloorArray AtFloorArray;
	//received desired floor message
	private ReadableCanMailbox networkDesiredFloor;
    //translator for the desired floor message
    private DesiredFloorCanPayloadTranslator mDesiredFloor;
    //received car level position message
    private ReadableCanMailbox networkCarLevelPosition;
    //translator for the car level position message
    private CarLevelPositionCanPayloadTranslator mCarLevelPosition;
    //output
    private WriteableCarPositionIndicatorPayload CarPositionIndicator;
    
    //store the period for the controller
    private SimTime period;
    private Hallway front = Hallway.FRONT;
    private Hallway back = Hallway.BACK;
    //enumerate states
    private enum State{
    	STATE_FIRST,
    	STATE_SECOND,
    	STATE_THIRD,
    	STATE_FOURTH,
    	STATE_FIFTH,
    	STATE_SIXTH,
    	STATE_SEVENTH,
    	STATE_EIGHTH
    }
    //state variable initialized to the initial state STATE_LIGHT_OFF
    private State state = State.STATE_FIRST;
    /**
     * The arguments listed in the .cf configuration file should match the order and
     * type given here.
     *
     */
    public CarPositionControl(SimTime period, boolean verbose) {
    	//call to the Controller superclass constructor is required
    	super("CarPositionControl", verbose);
    	//stored the constructor arguments in internal state
    	this.period = period;
    	
    	log("Created CarPositionControl with period = ", period);
    	//initialize input
    	networkCarLevelPosition = CanMailbox.getReadableCanMailbox(MessageDictionary.CAR_LEVEL_POSITION_CAN_ID);
    	mCarLevelPosition = new CarLevelPositionCanPayloadTranslator(networkCarLevelPosition);
    	canInterface.registerTimeTriggered(networkCarLevelPosition);
    	
    	networkDesiredFloor = CanMailbox.getReadableCanMailbox(MessageDictionary.DESIRED_FLOOR_CAN_ID);
    	mDesiredFloor = new DesiredFloorCanPayloadTranslator(networkDesiredFloor);
    	canInterface.registerTimeTriggered(networkDesiredFloor);
    	
    	AtFloorArray = new AtFloorArray(canInterface);
    	//initialize output
    	CarPositionIndicator = CarPositionIndicatorPayload.getWriteablePayload();
    	physicalInterface.sendTimeTriggered(CarPositionIndicator, period);
    	
    	timer.start(period);
    }
    
    public void timerExpired(Object callbackData) {
    	State nextstate = state;
    	switch (state) {
    	case STATE_FIRST:
    		//state actions for 'STATE_FIRST'
    		CarPositionIndicator.set(1);
//#transition 'T10.1'
    		if (AtFloorArray.isAtFloor(2, back) == true) {
    			nextstate = State.STATE_SECOND;
    		}
    		else {
    			nextstate = state;
    		}
    		break;
    	case STATE_SECOND:
    		//state actions for 'STATE_SECOND'
    		CarPositionIndicator.set(2);
//#transition 'T10.2'
    		if (AtFloorArray.isAtFloor(3, front) == true) {
    			nextstate = State.STATE_THIRD;
    		}
//#transition 'T10.3'
		    else if (AtFloorArray.isAtFloor(1, front) == true || AtFloorArray.isAtFloor(1, back) == true) {
			    nextstate = State.STATE_FIRST;
		    }
		    else {
		    	nextstate = state;
		    }
		    break;
    	case STATE_THIRD:
    		//state actions for 'STATE_THIRD'
    		CarPositionIndicator.set(3);
//#transition 'T10.4'
    		if (AtFloorArray.isAtFloor(4, front) == true) {
    			nextstate = State.STATE_FOURTH;
    		}
//#transition 'T10.5'
		    else if (AtFloorArray.isAtFloor(2, back) == true) {
			    nextstate = State.STATE_SECOND;
		    }
		    else {
		    	nextstate = state;
		    }
		    break;
    	case STATE_FOURTH:
    		//state actions for 'STATE_FOURTH'
    		CarPositionIndicator.set(4);
//#transition 'T10.6'
    		if (AtFloorArray.isAtFloor(5, front) == true) {
    			nextstate = State.STATE_FIFTH;
    		}
//#transition 'T10.7'
		    else if (AtFloorArray.isAtFloor(3, front) == true) {
			    nextstate = State.STATE_THIRD;
		    }
		    else {
		    	nextstate = state;
		    }
		    break;
    	case STATE_FIFTH:
    		//state actions for 'STATE_FIFTH'
    		CarPositionIndicator.set(5);
//#transition 'T10.8'
    		if (AtFloorArray.isAtFloor(6, front) == true) {
    			nextstate = State.STATE_SIXTH;
    		}
//#transition 'T10.9'
		    else if (AtFloorArray.isAtFloor(4, front) == true) {
			    nextstate = State.STATE_FOURTH;
		    }
		    else {
		    	nextstate = state;
		    }
		    break;
    	case STATE_SIXTH:
    		//state actions for 'STATE_SIXTH'
    		CarPositionIndicator.set(6);
//#transition 'T10.10'
    		if (AtFloorArray.isAtFloor(7, front) == true || AtFloorArray.isAtFloor(7, back) == true) {
    			nextstate = State.STATE_SEVENTH;
    		}
//#transition 'T10.11'
		    else if (AtFloorArray.isAtFloor(5, front) == true) {
			    nextstate = State.STATE_FIFTH;
		    }
		    else {
		    	nextstate = state;
		    }
		    break;
    	case STATE_SEVENTH:
    		//state actions for 'STATE_SEVENTH'
    		CarPositionIndicator.set(7);
//#transition 'T10.12'
    		if (AtFloorArray.isAtFloor(8, front) == true) {
    			nextstate = State.STATE_EIGHTH;
    		}
//#transition 'T10.13'
		    else if (AtFloorArray.isAtFloor(6, front) == true) {
			    nextstate = State.STATE_SIXTH;
		    }
		    else {
		    	nextstate = state;
		    }
		    break;
    	case STATE_EIGHTH:
    		//state actions for 'STATE_EIGHTH'
    		CarPositionIndicator.set(8);
//#transition 'T10.14'
    		if (AtFloorArray.isAtFloor(7, front) == true || AtFloorArray.isAtFloor(7, back) == true) {
    			nextstate = State.STATE_SEVENTH;
    		}
		    else {
		    	nextstate = state;
		    }
		    break;
    	default:
    		throw new RuntimeException("State " + state + " was not recognized.");
    	}
    	//log the results of this iteration
        if (state == nextstate) {
            log("remains in state: ",state);
        } else {
            log("Transition:",state,"->",nextstate);
        }
        //update the state variable
        state = nextstate;

        //report the current state
        setState(STATE_KEY,nextstate.toString());

        //schedule the next iteration of the controller
        //you must do this at the end of the timer callback in order to restart
        //the timer
        timer.start(period);
    }

}
