运行时层 (Runtime)
===================

主循环所使用的数据结构、估计器、控制器、调度器、安全与遥测组件。

核心数据总线
------------

.. automodule:: src.runtime.pose_snapshot
   :members:

.. automodule:: src.runtime.pose_bus
   :members:

.. automodule:: src.runtime.health_bus
   :members:

.. automodule:: src.runtime.link_quality_bus
   :members:

.. automodule:: src.runtime.link_state_bus
   :members:

.. automodule:: src.runtime.command_plan
   :members:

任务状态机与启动模式
--------------------

.. automodule:: src.runtime.mission_fsm
   :members:

.. automodule:: src.runtime.manual_input_port
   :members:

.. automodule:: src.runtime.manual_leader_state
   :members:

.. automodule:: src.runtime.manual_leader_reference
   :members:

估计与控制
----------

.. automodule:: src.runtime.affine_frame_estimator
   :members:

.. automodule:: src.runtime.follower_controller_base
   :members:

.. automodule:: src.runtime.follower_controller
   :members:

.. automodule:: src.runtime.follower_controller_v2
   :members:

调度、安全与失败策略
--------------------

.. automodule:: src.runtime.scheduler
   :members:

.. automodule:: src.runtime.safety_manager
   :members:

.. automodule:: src.runtime.failure_policy
   :members:

.. automodule:: src.runtime.landing_flow
   :members:

Telemetry 与离线采样
--------------------

.. automodule:: src.runtime.telemetry
   :members:

.. automodule:: src.runtime.telemetry_replay
   :members:

.. automodule:: src.runtime.mission_telemetry_reporter
   :members:

.. automodule:: src.runtime.offline_swarm_sampler
   :members:
