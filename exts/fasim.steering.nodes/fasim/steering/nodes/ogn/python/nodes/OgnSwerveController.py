import weakref

import omni.graph.core as og
import omni.timeline

from fasim.steering.nodes.swerve import SwerveMemory, compute_swerve

_LIVE = weakref.WeakSet()
_TIMELINE_SUB = None


def _on_timeline(event):
    if event.type != int(omni.timeline.TimelineEventType.STOP):
        return
    for memory in list(_LIVE):
        memory.reset()


def _ensure_timeline_subscription():
    global _TIMELINE_SUB
    if _TIMELINE_SUB is not None:
        return
    _TIMELINE_SUB = omni.timeline.get_timeline_interface().get_timeline_event_stream().create_subscription_to_pop(
        _on_timeline
    )


class OgnSwerveController:
    @staticmethod
    def internal_state():
        memory = SwerveMemory()
        _LIVE.add(memory)
        _ensure_timeline_subscription()
        return memory

    @staticmethod
    def compute(db) -> bool:
        memory = db.per_instance_state
        pos_cmd, vel_cmd, steer_joints, wheel_joints = compute_swerve(
            memory,
            profile=str(db.inputs.profile),
            vx=db.inputs.Desired_Linear_Velocity_X,
            vy=db.inputs.Desired_Linear_Velocity_Y,
            omega_cmd=db.inputs.Desired_Angular_Velocity,
            linear_gain=db.inputs.linearGain,
            angular_gain=db.inputs.angularGain,
            flip_vel_scale=db.inputs.flipVelocityScale,
            dt=db.inputs.deltaSeconds,
            msg_count=db.inputs.msgCount,
        )
        db.outputs.Steer_Position_Command = pos_cmd
        db.outputs.Wheel_Velocity_Command = vel_cmd
        db.outputs.Steer_Joints = list(steer_joints)
        db.outputs.Wheel_Joints = list(wheel_joints)
        db.outputs.execOut = og.ExecutionAttributeState.ENABLED
        return True
