/**
* 18649 Fall 2013
* Group 17
* Qiang Zhang(qiangz), Shen Yu(sheny), Jiang He(jiangh), Xianle Wang(xianlew)
**/
package simulator.elevatorcontrol;


import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.CanMailbox.WriteableCanMailbox;

public class EmergencyBrakeCanPayloadTranslator extends GenericBooleanTranslator {
	
	/**
     * Constructor for use with WriteableCanMailbox objects
     * @param payload
     */
    public EmergencyBrakeCanPayloadTranslator(WriteableCanMailbox payload) {
        super(payload, MessageDictionary.EMERGENCY_BRAKE_CAN_ID, "EmergencyBrake");
    }

    /**
     * Constructor for use with ReadableCanMailbox objects
     * @param payload
     */

    public EmergencyBrakeCanPayloadTranslator(ReadableCanMailbox payload) {
    	super(payload, MessageDictionary.EMERGENCY_BRAKE_CAN_ID, "EmergencyBrake");
    }
	
}
