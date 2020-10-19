# darknet_ros_docker
## 準備
1. dockerのインストールとユーザー情報の登録&再起動
   ```bash
   curl https://get.docker.com | sh
   sudo usermod -aG docker $USER
   sudo reboot
   ```

2. dockerを常時起動するように設定
    ```bash
    sudo systemctl start docker
    sudo systemctl enable docker
    ```

3. dockerのバージョンの確認（バージョンが表示されていればOK）
   ```bash
   docker --version
   ```

4. docker-composeのインストール  
    最新バージョンを以下のページで確認し、それに合わせてインストール  
    [https://github.com/docker/compose/releases](https://github.com/docker/compose/releases)
   ```bash
   # 例　バージョン1.16.1の場合
   # sudo curl -L "https://github.com/docker/compose/releases/download/1.25.4/docker-compose-$(uname -s)-$(uname -m)" -o /usr/local/bin/docker-compose
   sudo curl -L https://github.com/docker/compose/releases/download/[バージョン]/docker-compose-`uname -s`-`uname -m` -o /usr/local/bin/docker-compose

   sudo chmod +x /usr/local/bin/docker-composee
   ```

5. docker-composeのバージョンの確認（バージョンが表示されていればOK）
    ```
    docker-compose --version
    ```

6. cudaとnvidia-driverのインストール
   ```bash
   # すでにcudaとnvidia-driverが入ってる場合は一旦削除
   sudo apt-get --purge remove cuda*
   sudo apt-get --purge remove nvidia*
   sudo apt-get purge nvidia*
   sudo apt-get autoremove
   sudo apt-get autoclean
   sudo rm -rf /usr/local/cuda*
   # nvidia-driverとcudaのインストール
   wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/cuda-ubuntu1804.pin
	sudo mv cuda-ubuntu1804.pin /etc/apt/preferences.d/cuda-repository-pin-600
	sudo apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/7fa2af80.pub
	sudo add-apt-repository "deb http://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/ /"
	sudo apt-get update
	sudo apt-get -y install cuda
   # ドライバのインストールが終わったら再起動
   sudo reboot
   # パスの設定
   echo 'export PATH=/usr/local/cuda-10.0/bin${PATH:+:${PATH}}' >> ~/.bashrc
   echo 'export LD_LIBRARY_PATH=/usr/local/cuda-10.0/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}' >> ~/.bashrc
   source ~/.bashrc
   sudo ldconfig
   ```

7. nvidia-dockerのインストール
   ```bash
   # リポジトリの設定
   curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
   distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
   curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | \
   sudo tee /etc/apt/sources.list.d/nvidia-docker.list
   sudo apt-get update
   # Key の追加
   curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
   # 古いバージョンを削除してインストール
   docker volume ls -q -f driver=nvidia-docker | xargs -r -I{} -n1 docker ps -q -a -f volume={} | xargs -r docker rm -f
   sudo apt-get purge nvidia-docker
   sudo apt-get install nvidia-docker2
   sudo pkill -SIGHUP dockerd
   ```

8. nvidia-dockerのインストールができたかの確認  
   nvidia-smiが出力されたら成功  
   ```bash
   docker run --runtime=nvidia --rm nvidia/cuda nvidia-smi
   ```

## 使い方  
1. このリポジトリをcloneする(サブモジュールがあるので--recusiveが必要)
   ```bash
   git clone --recursive [リポジトリのURL]
   ```

2. 重みファイルのダウンロード  
   サーバにyolov3.weights置いてあるので、そちらからダウンロードして、以下のリポジトリに配置する。
   ```bash
   # 重みファイルを配置する場所
   darknet_ros_docker/catkin_ws/src/darknet_ros/darknet_ros/yolo_network_config/weights
   ```

3. dockerイメージをbuild
   ```bash
   cd darknet_ros_docker
    docker-compose build
   ```
4. dockerコンテナを起動
   ```bash
   docker-compose up -d
   ```

5. ディスプレイの使用許可を与える(本当は非推奨の方法)  
   ```bash
   # この設定はPCを再起動するたびに消える
   xhost +local:user
   ```

6. dockerコンテナ内に移動して、usb_camを起動
   ```bash
   # 移動
   docker-compose exec darknet_ros /bin/bash
   # usbカメラが認識されているかの確認
   # 内蔵カメラがある場合は検索結果が複数出てくる
   ls dev/video*
   # usb_camの起動
   # video0の部分は適宜変える
   # これを実行するとそのターミナルは操作できなくなる
   roslaunch usb_cam usb_cam.launch video_device:=/dev/video0
   ```
7. 別のターミナルを開いて、今度はコンテナでdarknet_rosを実行
   ```bash
   # コンテナ内に移動
   cd darknet_ros_docker
   docker-compose exec darknet_ros /bin/bash
   # darknet_rosを実行
   roslaunch darknet_ros darknet_ros_usb_cam.launch
   ```

## トラブルシューティング
### darknet_ros実行時に以下のエラーが発生してプログラムが終了する
   ```
   Loading weights from /root/catkin_ws/src/darknet_ros/darknet_ros/yolo_network_config/weights/yolov3.weights...Done!
   Waiting for image.
   [darknet_ros-1] process has died [pid 1702, exit code -11, cmd /root/catkin_ws/devel/lib/darknet_ros/darknet_ros __name:=darknet_ros __log:=/root/.ros/log/399e88fe-799f-11ea-a8cc-d43b04d7f15e/darknet_ros-1.log].
   log file: /root/.ros/log/399e88fe-799f-11ea-a8cc-d43b04d7f15e/darknet_ros-1*.log
   ```
   これは重みファイルweightsが正しく配置できていないことが多い  
</br>

### GUI(OpenCVのウィンドウやrvizが表示できない)
5.をもう一回すれば解決することが多い  
<br/>

### usb_cam.launchが起動しない
以下のことを行うと解決することが多い  
- usbカメラがきちんと認識されているかを確認  
   ( ls /dev/video* で確認する)  
- video_device:=/dev/video0のvideo0の部分を変える  
