#!/usr/bin/env python3
"""Only dispatch experiment phases; hardware commands are owned by agent nodes."""
import json
import threading
import rospy
from std_msgs.msg import String,Int32
from ne_tvft.protocol import Supervisor,decode,encode
from ne_tvft.sss import create_timer


def main():
    rospy.init_node("ne_tvft_supervisor")
    config=rospy.get_param("/ne_tvft/config")
    supervisor=Supervisor(config)
    lock=threading.RLock()
    session=rospy.Publisher("/ne_tvft/session",String,queue_size=10,latch=True)
    events=rospy.Publisher("/ne_tvft/events",String,queue_size=50)
    def command(msg):
        with lock:
            okay,reason=supervisor.command(msg.data,rospy.Time.now().to_sec())
            if not okay:
                rospy.logwarn("command rejected: %s",reason)
            events.publish(String(data=encode({"version":1,"command":msg.data,"accepted":okay,
                "reason":reason,"stamp":rospy.Time.now().to_sec(),"session_id":supervisor.session_id})))
    def telemetry(msg):
        try:
            with lock:
                supervisor.report(decode(msg.data),rospy.Time.now().to_sec())
        except (ValueError,TypeError,KeyError) as exc:
            rospy.logwarn_throttle(2,"Invalid telemetry: %s" % exc)
    def recorder(msg):
        try:
            with lock:
                supervisor.recorder_report(decode(msg.data),rospy.Time.now().to_sec())
        except (ValueError,TypeError) as exc:
            rospy.logwarn_throttle(2,"Invalid recorder status: %s" % exc)
    rospy.Subscriber("/command",Int32,command,queue_size=10)
    rospy.Subscriber("/ne_tvft/telemetry",String,telemetry,queue_size=100)
    rospy.Subscriber("/ne_tvft/recorder_status",String,recorder,queue_size=10)
    def tick(event):
        with lock:
            session.publish(String(data=encode(supervisor.tick(rospy.Time.now().to_sec()))))
            while supervisor.events:
                item=supervisor.events.pop(0)
                item["version"]=1
                events.publish(String(data=encode(item)))
    create_timer(.05,tick,config.get("simulation",{}).get("backend","hardware"))
    rospy.spin()


if __name__=="__main__":
    main()
