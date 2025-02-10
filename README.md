```
parallels@ubuntu-linux-22-04-02-desktop:~/Documents/ros2/ros2_ws/src$ ros2 run pkg1 my_node 
[INFO] [1739170977.112164290] [node1]: Published: Hello from Node 1
[INFO] [1739170977.112539997] [node1]: Node1 running in process ID: 343969
[INFO] [1739170977.112783705] [node2]: Received: Hello from Node 1
[INFO] [1739170977.112844580] [node2]: Node2 running in process ID: 343969
[INFO] [1739170978.111657793] [node1]: Published: Hello from Node 1
[INFO] [1739170978.111819918] [node1]: Node1 running in process ID: 343969
[INFO] [1739170978.112124625] [node2]: Received: Hello from Node 1
[INFO] [1739170978.112570707] [node2]: Node2 running in process ID: 343969
[INFO] [1739170979.110799674] [node1]: Published: Hello from Node 1
[INFO] [1739170979.110934799] [node1]: Node1 running in process ID: 343969
[INFO] [1739170979.111086298] [node2]: Received: Hello from Node 1
[INFO] [1739170979.111105840] [node2]: Node2 running in process ID: 343969
[INFO] [1739170980.111506469] [node1]: Published: Hello from Node 1
[INFO] [1739170980.111690094] [node1]: Node1 running in process ID: 343969
[INFO] [1739170980.111914260] [node2]: Received: Hello from Node 1
[INFO] [1739170980.111955426] [node2]: Node2 running in process ID: 343969
^C[INFO] [1739170980.443017796] [rclcpp]: signal_handler(signum=2)
parallels@ubuntu-linux-22-04-02-desktop:~/Documents/ros2/ros2_ws/src$ 
```


在ROS 2中，**Component Node** 支持两种组合方式：**静态组合（Static Composition）** 和 **动态组合（Dynamic Composition）**。这两种方式的主要区别在于组件的加载时机和管理方式。

---

### 1. **静态组合（Static Composition）**
静态组合是指在编译时确定组件的加载方式，组件在程序启动时被加载到同一个进程中。这种方式适合组件关系固定、不需要运行时动态调整的场景。

#### 实现方式
静态组合通常通过直接在主程序中实例化组件来实现。

#### 示例代码
```cpp
#include "rclcpp/rclcpp.hpp"
#include "component_a.hpp"
#include "component_b.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    // 创建执行器
    auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

    // 实例化组件A和组件B
    auto component_a = std::make_shared<ComponentA>();
    auto component_b = std::make_shared<ComponentB>();

    // 将组件添加到执行器
    executor->add_node(component_a);
    executor->add_node(component_b);

    // 运行执行器
    executor->spin();

    rclcpp::shutdown();
    return 0;
}
```

#### 特点
- **编译时确定**：组件的加载关系在编译时确定。
- **简单直接**：适合组件关系固定的场景。
- **性能高效**：组件在进程启动时加载，无需额外的动态加载开销。

---

### 2. **动态组合（Dynamic Composition）**
动态组合是指在运行时动态加载和管理组件。通过 `rclcpp_components` 提供的工具，可以在运行时根据需要加载或卸载组件。这种方式适合需要灵活调整组件关系的场景。

#### 实现方式
动态组合通过 `rclcpp_components::ComponentManager` 实现，允许在运行时加载组件。

#### 示例代码
```cpp
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/component_manager.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    // 创建执行器
    auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

    // 创建组件管理器
    auto component_manager = std::make_shared<rclcpp_components::ComponentManager>(executor);

    // 动态加载组件A和组件B
    component_manager->load_component("component_a", "ComponentA");
    component_manager->load_component("component_b", "ComponentB");

    // 运行执行器
    executor->spin();

    rclcpp::shutdown();
    return 0;
}
```

#### 特点
- **运行时动态加载**：组件可以在运行时根据需要加载或卸载。
- **灵活性强**：适合需要动态调整组件关系的场景。
- **插件化架构**：支持插件式扩展，便于系统扩展和维护。

---

### 3. **静态组合 vs 动态组合**

| 特性                | 静态组合                          | 动态组合                          |
|---------------------|-----------------------------------|-----------------------------------|
| **加载时机**         | 编译时确定                        | 运行时动态加载                    |
| **灵活性**           | 较低，组件关系固定                | 较高，组件关系可动态调整          |
| **实现复杂度**       | 简单直接                          | 需要组件管理器支持                |
| **适用场景**         | 组件关系固定的场景                | 需要动态调整组件关系的场景        |
| **性能**             | 较高，无动态加载开销              | 较低，动态加载会带来一定开销      |

---

### 4. **如何选择**
- 如果你的应用场景中组件关系固定，且不需要运行时动态调整，可以选择 **静态组合**。
- 如果你的应用场景需要动态加载、卸载组件，或者组件关系需要灵活调整，可以选择 **动态组合**。

---

### 5. **总结**
ROS 2 的 **Component Node** 提供了静态组合和动态组合两种方式，分别适用于不同的场景。静态组合适合组件关系固定的场景，而动态组合则提供了更高的灵活性，适合需要动态调整组件关系的场景。根据具体需求选择合适的组合方式，可以有效提升系统的性能和可维护性。

