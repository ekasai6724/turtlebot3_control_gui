# turtlebot3_control_gui
ROS2/QTで作成したTURTLEBOT3手動運転GUI

## ダウンロード
```sh
$ cd ~/colcon_ws/src
$ git clone https://github.com/ekasai6724/turtlebot3_control_gui.git
```
`~/colcon_ws`はcolconのワークスペースディレクトリです（任意の名前で可）

## ビルド
```sh
$ cd ~/colcon_ws
$ colcon build --symlink-install
```
## 実行前の準備
```sh
export ROS_DOMAIN_ID=32 #TURTLEBOT3
export TURTLEBOT3_MODEL=burger
export QT_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/qt5/plugins/
```
以上の内容を`~/.bashrc`に記入して（`Qt_PLUGIN_PATH`は環境によって変わるかもしれません）、
```sh
$ source ~/.bashrc
```
を実行してください。

## 起動コマンド
```sh
$ ros2 run turtlebot3_control_gui turtlebot3_control_gui
```
