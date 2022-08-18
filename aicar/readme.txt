智慧城市交通工程：
-----------------------------------------------------------------
一、工程结构：
 bin	项目脚本，通过desktop启动
 desktop	启动快捷方式，直接在文件夹下启动
	desktop下有个save_map.desktop文件，可以保存slam建图后的地图到本项目用
	install.desktop 为bin目录下的脚本修改执行权限
 launch	项目启动launch文件
 maps	案例用到的rosmap配置文件
 script	程序源码
vnode	智云车源码

二、使用说明：
1. 将aicar文件夹复制到/home/zonesion/catkin_ws/src目录下。
2. 从桌面文件管理器进入到目录/home/zonesion/catkin_ws/src/aicar/desktop目录下。
3. 双击install.desktop运行脚本进行应用权限修改。
4. 如果需要创建桌面快捷方式，只需将aicar.desktop复制到/home/zonesion/Desktop目录下

三、项目演示
1. 本项目需要NPU协处理模块、广角摄像头。
2. 将车辆放在地图出发位1号停车位，通过文件管理器进入到目录
    /home/zonesion/catkin_ws/src/aicar/desktop下双击aicar.desktop运行。
3. 车辆将自动进行路径规划，识别交通标志实现自动驾驶。