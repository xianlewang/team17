/**
* 18649 Fall 2013
* Group 17
* Qiang Zhang(qiangz), Shen Yu(sheny), Jiang He(jiangh), Xianle Wang(xianlew)
**/
package simulator.elevatorcontrol;

import simulator.framework.Direction;
import simulator.framework.Hallway;
import simulator.framework.ReplicationComputer;
import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.CanMailbox.WriteableCanMailbox;

public class HallCallCanPayloadTranslator extends GenericBooleanTranslator {
	
	/**
     * Constructor for use with WriteableCanMailbox objects
     * @param payload
     */
    public HallCallCanPayloadTranslator(WriteableCanMailbox payload, int floor, Hallway hallway, Direction direction) {
        super(payload, MessageDictionary.HALL_CALL_BASE_CAN_ID + ReplicationComputer.computeReplicationId(floor, hallway, direction), "HallCall" + 
              ReplicationComputer.makeReplicationString(floor, hallway, direction));
    }

    /**
     * Constructor for use with ReadableCanMailbox objects
     * @param payload
     */

    public HallCallCanPayloadTranslator(ReadableCanMailbox payload, int floor, Hallway hallway, Direction direction) {
    	super(payload, MessageDictionary.HALL_CALL_BASE_CAN_ID + ReplicationComputer.computeReplicationId(floor, hallway, direction), "HallCall" + 
              ReplicationComputer.makeReplicationString(floor, hallway, direction));
    }
	
}
