
---

# **ROV 控制栈更新规范与问题复盘报告**

版本：2025.12
作者：架构组
适用范围：`pwm_control_program` 全体开发人员

---

# 1. 背景

在 2025-12 的一次系统更新中，我们为控制循环引入了：

* 统一控制器接口 `IController`
* 控制模式重构
* 新的 `PwmLogger` 功能
* 控制循环与输入层的数据流修订

更新后暴露一系列编译错误与结构耦合问题，说明现有团队成员在扩展控制栈时缺乏一套统一的开发指南。

为避免今后的扩展导致项目崩溃，特制定本规范。

---

# 2. 本次更新暴露的典型问题

以下问题均在更新过程中实际出现，可视为控制栈的“高频错误区”。

## 2.1 新接口/类定义后未同步 include，导致找不到类型

典型报错：

```
error: 'PwmLogger' in namespace 'rovctrl::io' does not name a type
error: ‘ControllerBase’ is not a member of ‘rovctrl::controllers’
```

原因：

* 新增的 `PwmLogger` 文件未在 `control_loop.hpp` 中 include
* 原来的 `ControllerBase` 已废弃，但部分文件仍引用旧名称

性质：**经典头文件依赖不一致问题。**

---

## 2.2 引入新抽象接口但未更新已有类继承关系

典型报错：

```
error: invalid use of incomplete type ‘IController’
```

原因：

* `manual_controller.hpp` 仍继承旧接口
* forward declaration 存在，但没有包含完整定义（编译器只看到 incomplete type）

性质：**接口更新未全局同步。**

---

## 2.3 控制器、模式枚举、类型命名不一致

典型报错：

```
error: ‘Manual’ is not a member of ‘ControlMode’
```

实际枚举为：

```cpp
MANUAL
```

但部分文件使用：

```cpp
Manual
```

性质：**跨文件命名不一致导致全局编译失败。**

---

## 2.4 ControlLoop 参数列表变更后，上层 app_main 未同步更新

典型报错：

```
no matching function for call to ControlLoop(...)
```

原因：

* ControlLoop.cpp 更新了构造函数结构
* app_main.cpp 仍按照旧参数调用

性质：**接口变动未进行全工程同步替换。**

---

## 2.5 控制输出结构更新未同步，导致 compute 入口不匹配

控制输出从原始实现改为：

```cpp
ControlOutput {
    ThrusterArray thruster_command;
    bool has_thruster_command;
}
```

但旧控制器未设置标志位，造成：

* PWM 不输出
* 控制链不完整
* 日志不记录

性质：**跨模块的数据结构变更未同步更新行为规范。**

---

# 3. 问题根源总结

上述问题属于典型的**控制栈耦合性工程问题**，根本原因包括：

### 3.1 缺少统一的“控制系统更新流程”

开发者更新一个模块时，没有意识到它影响其他文件。

### 3.2 类型、接口、枚举的“单点变更没有全局替换”

例如：

* ControllerBase → IController
* MANUAL → ControlMode::MANUAL

必须全工程一致，否则系统无法编译。

### 3.3 缺少接口文档、依赖关系说明

新人往往不了解模块间的结构关系。

### 3.4 项目缺少统一的“工程约束”

例如：

* 禁止未 include 就使用类
* 禁止 forward declaration 替代完整引用（除非专门设计）
* 禁止在公共 API 中隐式更改参数类型

---

# 4. 未来功能更新必须遵循的工程流程（强制）

以下为**控制栈开发规范**，所有成员必须遵循。
适用于 control_core / controllers / io / platform 全模块。

---

# 4.1 Step 1：明确功能属于哪个层（必做）

```
输入层（InputProvider）
控制层（Controller）
执行层（PWM 客户端）
公共类型（ControlState / ControlOutput）
```

不同层次有不同影响范围。

---

# 4.2 Step 2：分析依赖链（必做）

例如新增控制器：

```
manual_controller.hpp
  → controller_base.hpp (IController)
    → control_loop.hpp (调用 compute)
      → app_main.cpp (构造控制器)
```

更新前必须思考：

* 哪些文件引用该类？
* 哪些文件需要 include？
* 是否需要修改构造函数？
* 是否需要更新枚举或类型定义？

---

# 4.3 Step 3：全工程搜索替换旧 API（必做）

例如：

```
ControllerBase → IController  
Manual → MANUAL
```

必须统一替换，避免部分文件引用旧接口。

---

# 4.4 Step 4：对所有依赖层执行 include 修复（必做）

检查：

* 头文件是否引用正确路径
* 命名空间是否一致
* forward declaration 是否足够

---

# 4.5 Step 5：更新 app_main / 控制循环逻辑（必做）

如果构造函数或数据结构更新，必须同步更新：

* app_main.cpp
* control_loop.hpp/cpp

这是这次问题最主要来源。

---

# 4.6 Step 6：编译 + 链接验证（必做）

必须保证：

* 无编译错误
* 无 undefined reference
* 无找不到类型

---

# 4.7 Step 7：运行测试（必做）

必须验证：

* 控制器能输出
* PWM 安全层能下发
* 日志能记录
* 按键退出生效

---

# 5. 为团队准备的“更新 Checklist”（贴在办公室墙上）

```
[ ] 明确功能层次
[ ] 建立影响范围列表
[ ] 新建文件 + CMakeLists 添加
[ ] include 路径检查
[ ] 命名空间检查
[ ] 全工程搜索替换旧 API
[ ] 更新控制循环（ControlLoop）
[ ] 更新 app_main
[ ] 更新相关控制器或输入源
[ ] 编译通过
[ ] 运行验证
[ ] 写入提交说明（包含影响范围）
```

---

# 6. 结论

本次更新暴露出控制栈的结构耦合问题，但同时也让我们发现：

* 当前架构已具规模，更新需要专业流程
* 必须建立严格工程规范
* 控制系统开发不能依赖“运气编译过”
* 要从“写代码”转向“维护控制系统生态”


---
