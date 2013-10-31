/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package simulator.elevatorcontrol;

import java.util.HashMap;

import simulator.elevatormodules.AtFloorCanPayloadTranslator;
import simulator.elevatormodules.CarLevelPositionCanPayloadTranslator;
import simulator.elevatormodules.DoorClosedCanPayloadTranslator;
import simulator.payloads.CANNetwork;
import simulator.framework.Elevator;
import simulator.framework.Hallway;
import simulator.framework.Harness;
import simulator.framework.ReplicationComputer;
import simulator.framework.Side;
import simulator.payloads.CanMailbox;
import simulator.payloads.CanMailbox.ReadableCanMailbox;

/**
 * This class provides some example utility classes that might be useful in more
 * than one spot.  It is okay to create new classes (or modify the ones given
 * below), but you may not use utility classes in such a way that they constitute
 * a communication channel between controllers.
 *
 * @author justinr2
 */
public class Utility {

    public static class DoorClosedArray {

        HashMap<Integer, DoorClosedCanPayloadTranslator> translatorArray = new HashMap<Integer, DoorClosedCanPayloadTranslator>();
        public final Hallway hallway;

        public DoorClosedArray(Hallway hallway, CANNetwork.CanConnection conn) {
            this.hallway = hallway;
            for (Side s : Side.values()) {
                int index = ReplicationComputer.computeReplicationId(hallway, s);
                ReadableCanMailbox m = CanMailbox.getReadableCanMailbox(MessageDictionary.DOOR_CLOSED_SENSOR_BASE_CAN_ID + index);
                DoorClosedCanPayloadTranslator t = new DoorClosedCanPayloadTranslator(m, hallway, s);
                conn.registerTimeTriggered(m);
                translatorArray.put(index, t);
            }
        }

        public boolean getBothClosed() {
            return translatorArray.get(ReplicationComputer.computeReplicationId(hallway, Side.LEFT)).getValue() &&
                    translatorArray.get(ReplicationComputer.computeReplicationId(hallway, Side.RIGHT)).getValue();
        }
    }
    public static class NearestFloor {
    	private ReadableCanMailbox networkCarLevelPosition;
    	private CarLevelPositionCanPayloadTranslator mCarLevelPosition;
    	
    	public NearestFloor(CANNetwork.CanConnection conn) {
    		networkCarLevelPosition = CanMailbox.getReadableCanMailbox(MessageDictionary.CAR_LEVEL_POSITION_CAN_ID);
    		mCarLevelPosition = new CarLevelPositionCanPayloadTranslator(networkCarLevelPosition);
    		conn.registerTimeTriggered(networkCarLevelPosition);
    	}
    	
    	public int getvalue() {
    		int value = 0;
    		if (mCarLevelPosition.getPosition() <= 2500)
    			value = 1;
    		else if (mCarLevelPosition.getPosition() > 2500 && mCarLevelPosition.getPosition() <= 7500)
    			value = 2;
    		else if (mCarLevelPosition.getPosition() > 7500 && mCarLevelPosition.getPosition() <= 12500)
    			value = 3;
    		else if (mCarLevelPosition.getPosition() > 12500 && mCarLevelPosition.getPosition() <= 17500)
    			value = 4;
    		else if (mCarLevelPosition.getPosition() > 17500 && mCarLevelPosition.getPosition() <= 22500)
    			value = 5;
    		else if (mCarLevelPosition.getPosition() > 22500 && mCarLevelPosition.getPosition() <= 27500)
    			value = 6;
    		else if (mCarLevelPosition.getPosition() > 27500 && mCarLevelPosition.getPosition() <= 32500)
    			value = 7;
    		else if (mCarLevelPosition.getPosition() > 32500)
    			value = 8;
    		return value;
    	}
    	
    	public int getLevelPosition() {
    		return mCarLevelPosition.getPosition();
    	}
    }
    public static class AtFloorArray {

        public HashMap<Integer, AtFloorCanPayloadTranslator> networkAtFloorsTranslators = new HashMap<Integer, AtFloorCanPayloadTranslator>();
        public final int numFloors = Elevator.numFloors;

        public AtFloorArray(CANNetwork.CanConnection conn) {
            for (int i = 0; i < numFloors; i++) {
                int floor = i + 1;
                for (Hallway h : Hallway.replicationValues) {
                    int index = ReplicationComputer.computeReplicationId(floor, h);
                    ReadableCanMailbox m = CanMailbox.getReadableCanMailbox(MessageDictionary.AT_FLOOR_BASE_CAN_ID + index);
                    AtFloorCanPayloadTranslator t = new AtFloorCanPayloadTranslator(m, floor, h);
                    conn.registerTimeTriggered(m);
                    networkAtFloorsTranslators.put(index, t);
                }
            }
        }
        
        public boolean isAtFloor(int floor, Hallway hallway) {
            return networkAtFloorsTranslators.get(ReplicationComputer.computeReplicationId(floor, hallway)).getValue();
        }

        public int getCurrentFloor() {
            int retval = MessageDictionary.NONE;
            for (int i = 0; i < numFloors; i++) {
                int floor = i + 1;
                for (Hallway h : Hallway.replicationValues) {
                    int index = ReplicationComputer.computeReplicationId(floor, h);
                    AtFloorCanPayloadTranslator t = networkAtFloorsTranslators.get(index);
                    if (t.getValue()) {
                        if (retval == MessageDictionary.NONE) {
                            //this is the first true atFloor
                            retval = floor;
                        } else if (retval != floor) {
                            //found a second floor that is different from the first one
                            throw new RuntimeException("AtFloor is true for more than one floor at " + Harness.getTime());
                        }
                    }
                }
            }
            return retval;
        }
    }
}
