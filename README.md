# 潜入式开发文档

---

## 更新日志

### 2.0.0.0.20160307_α

- 框架搭建
- 基础功能实现
- 已经实现的功能如下：
> - 能够通过二维码视觉信息进行精确导航，达到精度±5mm & 正负1°；
> - 能够实现AGV的目标朝向的指定，即可以从任何方向携带货架行走（前后左右）；

---

## 算法框架

### 检测部分

此部分需要实现AGV识别到货架位置，并给出与自身位置的关系；此部分由**戴舒炜**等人开发，通过一个向上的摄像头，观测货架底部的二维码，计算出二者的位置关系信息。


### 导航部分

此部分根据检测部分的位置信息就行导航，又**刘金勇**等人开发，导航方式大致如下：
> 先调整一个维度上的偏差 --- 角度微调 --- 调整另一个维度上偏差


---

## 施工要求

### 1、二维码贴的要精确
### 2、到点设置要准确


---
## 参数配置

### 1、模块运行相关参数

修改`/params`文件夹下`NR_AGV_param.xml`文件，添加潜入式导航部分参数，示例如下：

```xml
<Docking>
  <ElevatorLenth>2.2</ElevatorLenth>
  <ElevatorInDist>1.5</ElevatorInDist>
  <ElevatorOutDist>1.5</ElevatorOutDist>
  <DockingInDist>0.3</DockingInDist>
  <DockingOutDist>0.7</DockingOutDist>
  <DockingSafeDist>1.0</DockingSafeDist>
</Docking>
```

### 2、log配置参数

修改`/params`文件夹下`log4cpp.conf`文件，添加`DOCK`部分的参数，示例如下：

```conf
#子category及其appender
log4cpp.category.DOCK = DEBUG,A5
additivity.DOCK = false
log4cpp.appender.A5=RollingFileAppender
log4cpp.appender.A5.fileName=./log/Docking_Navigation.log
log4cpp.appender.A5.maxFileSize=3000000
log4cpp.appender.A5.maxBackupIndex=7
log4cpp.appender.A5.append=true
log4cpp.appender.A5.layout=PatternLayout
log4cpp.appender.A5.layout.ConversionPattern=%d [%t] %p %c - %m%n
```
