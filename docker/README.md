## Description
openposeをROS上で動かすためのdockerfileです。

## Requirement
* docker
* NVIDIAのGPU
* Xtion pro live

## Usage
* nvidia-docker2のインストール 
    * ホストのPCにインストールされているnvidiaドライバのバージョンの更新  
        (400番代以上が理想 geforce 1060や1660の場合はnvidia-418)
        * 古いドライバの削除
            ```sh
            $ sudo apt-get --purge remove nvidia-*
            $ sudo apt-get --purge remove cuda-*
        * 新しいドライバのインストール
            ```sh
            $ sudo add-apt-repository ppa:graphics-drivers/ppa
            $ sudo apt-get update
            $ sudo apt-get install nvidia-418
            $ sudo reboot
        * 確認
            ```sh 
            $ nvidia-smi
    * nvidia-docker2のインストール
        ```sh 
        $ curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | \sudo apt-key add -
        $ curl -s -L https://nvidia.github.io/nvidia-docker/ubuntu16.04/amd64/nvidia-docker.list | \
        $ sudo tee /etc/apt/sources.list.d/nvidia-docker.list
        $ sudo apt-get update
        $ sudo apt-get install -y nvidia-docker2
        $ sudo pkill -SIGHUP dockerd
    * 確認
        ```sh 
        $ docker run --runtime=nvidia --rm nvidia/cuda nvidia-smi
* openpose_rosパッケージのクローン
    ```sh 
    $ git clone https://github.com/chakio/openpose_ros.git
* ビルド
    ```sh 
    $ cd /path/to/openpose_ros/dockerfile
    $ ./build
* 実行   
    * xtion pro liveをPCに接続して、以下のコマンドを実行
    ```sh 
    $ cd /path/to/openpose_ros/dockerfile
    $ ./run
    $ roslaunch openpose_ros openpose_ros.launch
## Author
[chakio](https://github.com/chakio)