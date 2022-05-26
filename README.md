# 版本
- ROS Noetic
- openCV 4.2.0
- Gazebo11

# 非ROS依賴
- vindereu/openCV-model/cv_py

# 安裝(非ROS依賴需要記得另外安裝)
1. `cd <工作區域>/src`
2. `git clone https://github.com/vindereu/gazebo-moveit.git`
3. `rospack profile`
4. `rosdep install test_gazebo_simple arm arm_description`
5. `cd .. && catkin_make`

# 使用方式
參考doc資料夾內容
