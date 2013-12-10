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
import simulator.payloads.CanMailbox;
import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.CarPositionIndicatorPayload;
import simulator.payloads.CarPositionIndicatorPayload.WriteableCarPositionIndicatorPayload;
import simulator.elevatorcontrol.Utility.NearestFloor;;

public class CarPositionControl extends Controller{
	/***************************************************************************
     * Declarations
     **************************************************************************/
    //note that inputs are Readable objects, while outputs are Writeable objects
	//input
	private NearestFloor NearestFloor;
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
    //enumerate states
    private enum State{
    	STATE_INITIAL,
    	STATE_FLOOR
    }
    //state variable initialized to the initial state STATE_LIGHT_OFF
    private State state = State.STATE_INITIAL;
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
    	
    	NearestFloor = new NearestFloor(canInterface);
    	
    	//initialize output
    	CarPositionIndicator = CarPositionIndicatorPayload.getWriteablePayload();
    	physicalInterface.sendTimeTriggered(CarPositionIndicator, period);
    	
    	timer.start(period);
    }
    
    @Override
	public void timerExpired(Object callbackData) {
    	State nextstate = state;
    	switch (state) {
    	case STATE_INITIAL:
    		//state actions for 'STATE_INITIAL'
    		CarPositionIndicator.set(1);
//#transition 'T10.1'
    		if (mCarLevelPosition.getPosition() > 2500) {
    			nextstate = State.STATE_FLOOR;
    		}		
    		else {
    			nextstate = State.STATE_INITIAL;
    		}
    		break;
    	case STATE_FLOOR:
    		//state actions for 'STATE_FLOOR'
    		CarPositionIndicator.set(NearestFloor.getvalue());
//#transition 'T10.2'
    		if (mCarLevelPosition.getPosition() <= 2500) {
    			nextstate = State.STATE_INITIAL;
    		}
		    else {
		    	nextstate = State.STATE_FLOOR;
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
