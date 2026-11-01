#!/usr/bin/env python3
"""Ideal ROS-interface emulator, NOT a PX4 flight-dynamics simulator.

All external names are prefixed /mock. It exercises the actual ROS state
machine, per-agent nodes, recorder and message/command paths without hardware.
"""
import math
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped,TwistStamped,Twist
from sensor_msgs.msg import Imu
from mavros_msgs.msg import State,ExtendedState,PositionTarget
from mavros_msgs.srv import CommandBool,CommandBoolResponse,SetMode,SetModeResponse
from ne_tvft.core import circle
from ne_tvft.adapters import PlanarTransform


def external(topic):
    return topic if topic.startswith('/mock/') else '/mock'+topic


class Fleet:
    def __init__(self,config):
        self.config=config
        self.robots={}
        self.services=[]
        for gi,group in enumerate(config['groups']):
            for ji,rid in enumerate([group['leader']]+group['followers']):
                c=config['robots'][rid]
                xy=np.array(group['initial_xy'],dtype=float)
                if ji:
                    xy+=circle(config,gi,ji-1,0.)[0]
                r={'p':np.r_[xy,0.], 'v':np.zeros(3),'a':np.zeros(3),'cmd':None,
                   'last_cmd':-1.,'mode':'MANUAL','armed':False,'config':c}
                r['pose_transform']=PlanarTransform(c.get('transform'))
                r['velocity_transform']=PlanarTransform(c.get('velocity_transform',c.get('transform')))
                r['setpoint_transform']=PlanarTransform(c.get('setpoint_transform'))
                self.robots[rid]=r
                r['pose_pub']=rospy.Publisher(external(c['pose_topic']),PoseStamped,queue_size=5)
                r['vel_pub']=rospy.Publisher(external(c['velocity_topic']),TwistStamped if c['velocity_type']=='TwistStamped' else Twist,queue_size=5)
                r['imu_pub']=rospy.Publisher(external(c['imu_topic']),Imu,queue_size=5)
                typ=Twist if c['kind']=='ugv' else PositionTarget
                rospy.Subscriber(external(c['command_topic']),typ,lambda msg,name=rid:self.command(name,msg),queue_size=1)
                if c['kind']=='uav':
                    r['state_pub']=rospy.Publisher(external(c['state_topic']),State,queue_size=5)
                    r['extended_pub']=rospy.Publisher(external(c['extended_state_topic']),ExtendedState,queue_size=5)
                    self.services.append(rospy.Service(external(c['arming_service']),CommandBool,lambda req,name=rid:self.arm(name,req)))
                    self.services.append(rospy.Service(external(c['mode_service']),SetMode,lambda req,name=rid:self.mode(name,req)))
        self.last=rospy.Time.now().to_sec()
        rospy.Timer(rospy.Duration(.02),self.tick)

    def command(self,name,msg):
        self.robots[name]['cmd']=msg
        self.robots[name]['last_cmd']=rospy.Time.now().to_sec()

    def arm(self,name,req):
        self.robots[name]['armed']=req.value
        return CommandBoolResponse(success=True,result=0)

    def mode(self,name,req):
        self.robots[name]['mode']=req.custom_mode
        return SetModeResponse(mode_sent=True)

    @staticmethod
    def limit(v,bound):
        return v*min(1.,bound/max(np.linalg.norm(v),1e-12))

    def tick(self,event):
        now=rospy.Time.now().to_sec();dt=min(max(now-self.last,0.),.04);self.last=now
        for rid,r in self.robots.items():
            c=r['config'];cmd=r['cmd'];previous=r['v'].copy()
            if c['kind']=='ugv':
                r['v']=np.zeros(3) if cmd is None or now-r['last_cmd']>.4 else np.array([cmd.linear.x,cmd.linear.y,0.])
            elif r['mode']=='AUTO.LAND':
                r['v']=np.array([0.,0.,-.2]) if r['p'][2]>.01 else np.zeros(3)
                if r['p'][2]<=.01:
                    r['armed']=False
            elif not r['armed'] or r['mode']!='OFFBOARD' or cmd is None or now-r['last_cmd']>.4:
                r['v']*=.9
            elif not cmd.type_mask & 7:
                target=r['setpoint_transform'].point_to_world([cmd.position.x,cmd.position.y,cmd.position.z])
                r['v']=self.limit(1.5*(target-r['p']),.45)
            elif not cmd.type_mask & 448:
                acceleration=r['setpoint_transform'].vector_to_world([cmd.acceleration_or_force.x,cmd.acceleration_or_force.y,cmd.acceleration_or_force.z])
                r['v']+=acceleration*dt
            else:
                r['v']=r['setpoint_transform'].vector_to_world([cmd.velocity.x,cmd.velocity.y,cmd.velocity.z])
            r['p']+=r['v']*dt;r['p'][2]=max(0.,r['p'][2])
            r['a']=(r['v']-previous)/max(dt,1e-6)
            pose=PoseStamped();pose.header.stamp=rospy.Time.from_sec(now);pose.header.frame_id=c['pose_frame']
            pose.pose.position.x,pose.pose.position.y,pose.pose.position.z=r['pose_transform'].point_from_world(r['p'])
            pose.pose.orientation.z=math.sin(-r['pose_transform'].yaw/2)
            pose.pose.orientation.w=math.cos(-r['pose_transform'].yaw/2)
            r['pose_pub'].publish(pose)
            tw=Twist();tw.linear.x,tw.linear.y,tw.linear.z=(r['v'] if c['velocity_frame']=='body'
                else r['velocity_transform'].vector_from_world(r['v']))
            if c['velocity_type']=='TwistStamped':
                vs=TwistStamped();vs.header=pose.header;vs.twist=tw;r['vel_pub'].publish(vs)
            else:
                r['vel_pub'].publish(tw)
            imu=Imu();imu.header=pose.header;imu.orientation.w=1.
            imu.linear_acceleration.x,imu.linear_acceleration.y,imu.linear_acceleration.z=r['a']+np.array([0.,0.,9.80665])
            r['imu_pub'].publish(imu)
            if c['kind']=='uav':
                state=State();state.header=pose.header;state.connected=True;state.armed=r['armed'];state.mode=r['mode']
                r['state_pub'].publish(state)
                ex=ExtendedState();ex.header=pose.header;ex.landed_state=1 if r['p'][2]<.02 else 2
                r['extended_pub'].publish(ex)


if __name__=='__main__':
    rospy.init_node('ne_tvft_mock_fleet')
    fleet=Fleet(rospy.get_param('/ne_tvft/config'))
    rospy.spin()
