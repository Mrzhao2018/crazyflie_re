Crazyflie AFC Swarm
====================

基于 **Crazyflie + Lighthouse + cflib** 的仿射编队控制实验基线。本文档汇总
``src/`` 各层模块的 API 参考，以及仓库 ``doc/`` 下持续维护的专题 Markdown 资料。

项目门面、CLI 与配置摘要仍以仓库根目录的 ``README.md`` 为准；工程优化、
通信加固、离线调参与项目结构说明则在本站“专题文档”中可直接阅读。

.. note::

   本文档当前覆盖到 **PR10/PR11（2026-04-19）**。默认编队为 10 机，
   leaders ``{1, 4, 7, 8}``、followers ``{2, 3, 5, 6, 9, 10}``、
   radio groups ``{0, 1, 2}``。

分层结构一览
------------

- ``config/`` —— yaml 配置（fleet / mission / comm / safety / startup）的加载与校验
- ``domain/`` —— 编队模型、AFC、stress matrix、leader / follower reference、mission profile
- ``runtime/`` —— pose bus、affine frame、controller、scheduler、safety、failure policy、telemetry
- ``adapters/`` —— cflib link / transport、group executor pool、pose source、键盘输入
- ``app/`` —— 统一 CLI、真机主循环、preflight、离线 replay / viz / compare

专题文档
--------

.. toctree::
   :maxdepth: 2
   :caption: 专题文档

   topics/index

API 参考
--------

.. toctree::
   :maxdepth: 2
   :caption: API 参考

   api/config
   api/domain
   api/runtime
   api/adapters
   api/app
   api/web

索引
====

* :ref:`genindex`
* :ref:`modindex`
