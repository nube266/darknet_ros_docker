# darknet_ros_docker
## 準備
1. dockerのインストールとユーザー情報の登録&再起動
   ```bash
   curl https://get.docker.com | sh
   sudo usermod -aG docker $USER
   sudo reboot
   ```

2. dockerを常時起動するように設定
    ```
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
   # 1.61の確認
   sudo curl -L https://github.com/docker/compose/releases/download/1.16.1/docker-compose-`uname -s`-`uname -m` -o /usr/local/bin/docker-compose
   sudo chmod +x /usr/local/bin/docker-compose
   ```

5. docker-composeのバージョンの確認（バージョンが表示されていればOK）
    ```
    docker-compose --version
    ```

## 使い方  
1. docker内でGUIを使うためにターミナルを開いて以下のコマンドを入力  
    ```bash
    xhost +
    ```
2. dockerファイルがある位置に移動
    ```bash
    cd darknet_ros_docker
    ```
3. dockerイメージをbuild
   ```bash
    docker-compose build
   ```
4. dockerコンテナを起動
   ```bash
   docker-compose up -d
   ```
5. dockerコンテナ内に移動して、usb_camを起動
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
6. 別のターミナルを開いて、今度はコンテナでdarknet_rosを実行
   ```bash
   # コンテナ内に移動
   cd darknet_ros_docker
   docker-compose exec darknet_ros /bin/bash
   # darknet_rosを実行
   roslaunch darknet_ros darknet_ros_usb_cam.launch
   ```
