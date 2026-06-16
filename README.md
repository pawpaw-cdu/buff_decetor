本开源项目是Ultra_Vision的子项目，参考了华南理工大学2025年能量机关算法，引入贪心算法和跟踪器进行丢失的扇叶匹配。
显著的解决了由于扇叶丢失导致模型频繁重加载引起的跳变。

项目结构:
--build      # 存放cmake相关文件和生成的可执行文件
--config     # 配置文件
  --detector.yaml  
  --detector.yaml
--include    #相关头文件，包含接口和相关数据结构定义
  --config.hpp
  --detector.hpp
  --tracker.hpp
  --types.hpp
  --utils.hpp
--src      #具体实现和示例主函数
  --config.cpp
  --detector.cpp
  --main.cpp    # 示例主函数
  --tracker.cpp
  --utils.cpp
--CMakeLists.txt
--README.md
--buff_red.mp4  # 测试视频

