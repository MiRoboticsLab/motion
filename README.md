# cyberdog_motion

*L91主控运动管理*

### 软件架构

[设计文档](blob:https://xiaomi.f.mioffice.cn/29484161-1078-4de7-9dee-0ac4db7e3b0f)

- motion_manager：运动管理单元，所有L91软件调用运动能力，均由该模块负责管理；
- navigation2：导航单元，所有L91导航能力均由该模块负责；
- motion_action：运动指令单元，所有对运动的调用均经过该模块转发；
- navigation_core: 导航包装模块，提供对导航功能的封装以及启动管理等功能。