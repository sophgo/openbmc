# Linux 设备驱动开发学习资源

## Knowledge

### 核心教材（必读）

- [Book: _Linux Device Drivers, 3rd Edition_ (LDD3) by Corbet, Rubini & Kroah-Hartman](https://lwn.net/Kernel/LDD3/)
  驱动开发的经典教材，虽然基于 2.6 内核但概念和框架至今适用。免费在线阅读。使用场景：理解驱动的基本概念、字符设备、并发控制、内存管理。

- [Book: _Linux 设备驱动开发详解_ by 宋宝华](https://item.jd.com/12213824.html)
  中文版，内容更现代，覆盖设备树、I2C/SPI 驱动、平台驱动等。使用场景：系统学习驱动框架、查找具体驱动的实现方法。

- [Kernel Documentation: Driver API](https://www.kernel.org/doc/html/latest/driver-api/index.html)
  官方驱动 API 文档，最权威的参考。使用场景：查找具体子系统的 API、理解框架设计意图。

### 内核源码（最佳实践参考）

- [Linux Kernel Source: drivers/](https://github.com/torvalds/linux/tree/master/drivers)
  真实驱动代码，最好的学习材料。使用场景：学习同类驱动的实现方式、理解最佳实践、模仿代码风格。

- [Bootlin Elixir Cross Referencer](https://elixir.bootlin.com/linux/latest/source)
  在线内核源码浏览器，支持交叉引用跳转。使用场景：快速查找函数定义、理解调用链。

### 专题文档

- [Device Tree Specification](https://www.devicetree.org/specifications/)
  设备树官方规范。使用场景：理解设备树语法、binding 文档、属性含义。

- [Kernel Documentation: driver-model](https://www.kernel.org/doc/html/latest/driver-api/driver-model/index.html)
  Linux 设备模型文档（bus/device/driver 框架）。使用场景：理解驱动的注册、匹配、probe 流程。

- [I2C/SMBus Protocol](https://www.kernel.org/doc/html/latest/i2c/index.html)
  I2C 子系统文档。使用场景：I2C 驱动开发、SMBus 协议细节。

## Wisdom (Communities)

- [Linux Kernel Mailing List (LKML)](https://lkml.org/)
  内核开发者的主战场，可以看到驱动 review 过程。使用场景：学习上游驱动的代码规范、理解 review 标准。

- [r/kernel](https://reddit.com/r/kernel)
  Linux 内核讨论社区。使用场景：提问、看别人的问题、了解常见坑。

- [Stack Overflow: linux-kernel tag](https://stackoverflow.com/questions/tagged/linux-kernel)
  大量驱动开发相关问题和答案。使用场景：遇到具体问题时搜索。

## Gaps

- 中文高质量驱动教程较少，主要依赖英文资料
- 缺乏针对 BSP 工程师的系统化教程（大多面向应用开发者）
