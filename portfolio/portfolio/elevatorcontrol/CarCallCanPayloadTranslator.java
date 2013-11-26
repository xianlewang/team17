/**
* 18649 Fall 2013
* Group 17
* Qiang Zhang(qiangz), Shen Yu(sheny), Jiang He(jiangh), Xianle Wang(xianlew)
**/
package simulator.elevatorcontrol;

import simulator.framework.Hallway;
import simulator.framework.ReplicationComputer;
import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.CanMailbox.WriteableCanMailbox;

public class CarCallCanPayloadTranslator extends GenericBooleanTranslator {
	
	/**
     * Constructor for use with WriteableCanMailbox objects
     * @param payload
     */
    public CarCallCanPayloadTranslator(WriteableCanMailbox payload, int floor, Hallway hallway) {
        super(payload, MessageDictionary.CAR_CALL_BASE_CAN_ID + ReplicationComputer.computeReplicationId(floor, hallway), 
        	  "CarCall" + ReplicationComputer.makeReplicationString(floor, hallway));
    }

    /**
     * Constructor for use with ReadableCanMailbox objects
     * @param payload
     */

    public CarCallCanPayloadTranslator(ReadableCanMailbox payload, int floor, Hallway hallway) {
    	super(payload, MessageDictionary.CAR_CALL_BASE_CAN_ID + ReplicationComputer.computeReplicationId(floor, hallway), 
    		  "CarCall" + ReplicationComputer.makeReplicationString(floor, hallway));
    }
	
}
