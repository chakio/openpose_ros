[![MIT licensed](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)
# OpenPose_ROS
![result](https://github.com/chakio/openpose_ros/blob/master/media/openpose3D.gif)  
[OpenPose](https://github.com/CMU-Perceptual-Computing-Lab/openpose)をROS化し、深度情報を用いて3次元化します。

## Description
[ROS](http://wiki.ros.org/ja)(Robot Operating System)にて[OpenPose](https://github.com/CMU-Perceptual-Computing-Lab/openpose)を用いた関節位置推定を行うためのプログラムです。  
本来、OpenPoseはRGBカメラを用いた2次元関節位置情報を取得できるソフトウエアです。   
ロボットを制御する際などには3次元関節位置が必要となるため、RGB-Dカメラを用いることで関節位置の3次元座標を取得しました。  

## Feature
* OpenPoseの3次元化：PCD(Point Cloud Data)の画素マッチングによる３次元座標取得
* TFの管理：OpenPoseの処理時間の考慮（処理時間、通信時間の考慮）
* 出力：今回はsensor_msgs::Pointcloud2を使用(新規メッセージの作成は複数PC上でのSubscribeの際手間)。オリジナルのFieldを定義可能なためconfidenceなどもやりとり可能

## Requirement 
* OpenPose : v1.4.0
* ROS : kinetic  
* Camera : Xtion pro live

## Setup
* OpenPose v1.4.0を[インストール方法](https://github.com/CMU-Perceptual-Computing-Lab/openpose/blob/master/doc/installation.md)に従いインストールした後に、  [Python API](https://github.com/CMU-Perceptual-Computing-Lab/openpose/blob/master/doc/installation.md#python-api)をインストール

* （C++ APIなどの選択肢もあるが、CV::BridgeなどがOpencvのバージョンなどの問題でうごかなかった）

* （openpose_wrapper.py というサンプルコードを改変し、ros化したが、スレッドが別れないようにする工夫が必要です。（あやふや））

* [dockerについてはこちら](https://github.com/chakio/openpose_ros/tree/master/dockerfile/README.md)

## Usage
* ```rosrun openpose_ros pub_pcd.py```

または
* ```roslaunch openpose_ros openpose_ros.launch```

## Lisence
[MIT](https://github.com/chakio/openpose_ros/blob/master/LICENSE)

## Author
[chakio](https://github.com/chakio)