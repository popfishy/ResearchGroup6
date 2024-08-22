#### 一、使用说明

1. 请[自行配置PX4和XTDrone环境](https://www.yuque.com/xtdrone/manual_cn/basic_config_13)(具体为ros noetic + px4 1.13)，下载QGC地面站，并将地面站QGroundControl.AppImage放在scripts文件夹下。
2. 阅读scripts脚本目录下的start.sh文件，修改相应配置，'dji'中修改为自己电脑密码。

```
echo 'dji' | sudo -S chmod 777 QGroundControl.AppImage
```

3. 运行scripts脚本目录下的start.sh即可。目前为正式版本。现对代码进行说明：
   - TEST_FLAG为是否开启测试，为0时接收公司方数据；为1时为测试模式，模拟公司方发布数据。
   - **测试版本模拟公司发送数据**，即在仿真初始化完毕后，运行/communication/scripts下的company_to_group6.py程序，发布话题"/ResearchGroup5ResultTest"给receive_company_data.py进行接收。正式版本不需要模拟发送公司数据，但是需要修改receive_company_data.py中接收话题名称为"/ResearchGroup5Result”。（两个版本不同的具体原因：ROS不能存在两个相同名称的话题）
   - communication/json文件夹下为各个课题具体数据，test.json为测试数据，用于测试版本。目前该数据由课题五提供，存在每个任务规划的path最后两个点相同，导致路径生成代码报错。（目前已解决）
   - 数据处理部分需注意，所有WGS84坐标系均转化为MAVROS使用的ENU坐标系，之后PX4接收到来自MAVROS的ENU坐标系数据后，会自行转化为NED坐标系。同理，无人机数据发送给公司时，需将来自gazebo的ENU数据转化为WGS84数据。
   - QGC地面站参数通过mavros_msgs下的ParamSet服务进行修改，具体代码可参考fixed_wing_formation下的QGC_param_set.py代码。
4. 输入说明：
   ```
      {
         "agentId": 102501,                       无人机ID编号
         "allowDelayTime": 50,                    允许的任务最大延迟时间
         "endAltitude": 200,                      任务终点高度
         "beginTime": 3816,                       任务开始时间
         "expectedDuration": 300,                 任务预计时长
         "expectedQuantity": 3,                   任务期望单元的数量
         "endLatitude": 24.79647,                 任务终点纬度
         "endLongitude": 120.904095,              任务终点经度
         "order": 1,                              任务优先级
         "phase_name": "打击阶段",                 
         "status": 0,                             暂时无用
         "task_flag": "p206",                     任务编号
         "task_name": "歼灭1",             
         "task_phase": 4,                         任务阶段   
         "task_type": 5,                          任务类型
         "velocity": 45,                          无人机速度
         "latitude": 24.4963,                     任务起点纬度
         "longitude": 119.641,                    任务起点经度
         "altitude": 200                          任务起点高度
         "path"：[]                               课题五规划结果
       }
   ```
   输出说明：**根据位置信息、速度矢量和姿态角能够确定无人机在当前空间内的位姿。**
   ```
   {
      "key": "SwarmTrajectoryResults",                        
      "name": "ResearchGroup6",          
      "timestamp": 473.716,                                    当前公司仿真时间
      "agents": [
         {
               "agentId": 102501,                              无人机ID编号
               "velocity_x": -2.2437286137346362e-07,         （无人机速度，与Gazebo保持一致
               "velocity_y": -1.597075669263053e-07,           具体为ENU东北天坐标系，公司未明确
               "velocity_z": -4.2074896141192085e-08,          说明其需要的具体坐标系）
               "latitude": 24.4819,                            当前无人机在WGS84坐标系下的位置
               "longitude": 119.61599999892212,
               "altitude": 0.12300364708060345,
               "roll": 1.249150830426465e-10,                  机体坐标系下的姿态角
               "pitch": 0.0013865438033745109,
               "yaw": 1.845275799380215e-13
         }
      ]
   }
   ```


#### 二、后续工作

1. 四旋翼代码接入：思考如何修改QGC中的EKF参数（可参考QGC_param_set.py）     思考如何模拟深度数据（越简单越好）
2. 如何解决固定翼飞机的local坐标系原点为px4上电位置（获取固定翼飞机在gazebo中初始化的位置表，课题七还没给我们）
3. 空速无法到达60m/s，可能需要等比例缩放数据。（可行解决办法：[修改gazebo中plane模型动力相关参数，修改混控器](https://www.yuque.com/xtdrone/manual_cn/control_mapping)等）
 - 最终采用比例缩放数据，除了对目标点相对位置进行缩放外，需具体对无人机队形进行处理。

