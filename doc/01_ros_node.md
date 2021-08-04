# ROSチュートリアル第1回

### 1.簡単なROSノードの作成

1. **パッケージを作る**

```sh
# catkin_create_pkg [package name] [dependent packages]
catkin_create_pkg ros_tutorial roscpp rospy image_transport cv_bridge
```





2. **画像を用意する**

```bash
roscd ros_tutorial
mkdir image

# image配下に画像を置く
```





3. **ソースファイルをコーディングする**

ファイルのパスは変更する。

具体的な内容についてはソースコード中のコメントを参照する



3. **CMakeLists.txtを更新する**



3. **ビルドする**

```bash
# $HOME 以外の場所にワークスペースを作っている場合は適宜変更する
cd ~/catkin_ws
catkin_make
```

CMakeLists.txtを読み込んでワークスペース中のすべてのパッケージの依存関係をチェックした後、コンパイル、リンクを行う。





5. **実行してみる**

```bash
# terminal 1
roscore

# terminal 2
rosrun ros_tutorial ros_tutorial_image_publisher


# terminal 3
rosrun ros_tutorial ros_tutorial_image_subscriber
```





## 2.orb_slam2_rosでの実行

orb_slam2_testの、トピック名のみを変える	



### 参考資料

