# ROSチュートリアル第1回

## はじめに

#### 目的

実際の開発の中でよく使われるであろうROS開発知識を、**ROSの概念やツールの包括的な説明を放棄することにより短時間で**、そして**実際に手を動かすことで確実に**身につけること。



#### 前提条件、知識

- ROSのインストール

- 基本的なlinuxコマンドや操作




#### 内容

1. ROSのパブリッシャとサブスクライバを書く
   - パッケージを作る
   - 画像を読み込んでpublishするノード
   - 画像をsubscribeして保存するノード
2. launchファイルを書く
   - 1で作成したノードをまとめて起動するlaunchファイル
   - 引数やパラメータを使うlaunchファイル
3. 応用：orb_slam2_rosパッケージ
   - orb_slam2_rosのlaunchファイルの必要なところを変更して、1で作成したノードで画像が与えられることを確かめる
   - ソースコードの一部を見てみる



### 1 簡単なROSノードの作成

#### パッケージを作る

```sh
# ROSのワークスペースに移動
# $HOME 以外の場所にワークスペースを作っている場合は適宜変更する
cd ~/catkin_ws/src

# パッケージを作成するコマンド
# catkin_create_pkg [package name] [dependent packages]
catkin_create_pkg ros_tutorial roscpp rospy image_transport cv_bridge
```

まず、ROSのパッケージを上記のコマンドで作成する。依存関係はパッケージを作成した後でも追加することができる。

上記コマンドが正しく実行されれば、`ros_tutorial`というフォルダが新しく作成される。



- ROSのパッケージの意義について

ROSでは、パッケージと呼ばれる単位でプログラムを管理する。パッケージで区別してプログラムを区切ると、プログラムの再利用がしやすくなる。簡単に言えば、処理Aを行うプログラムと処理Bを行うプログラムを合わせてモジュールXを最初に作成し、その後、処理Aと処理Cを合わせたモジュールYを作成したくなったとき、処理Aと処理Bが密結合したコードになっていると、プログラムの再利用がしにくい。処理を分けて、その間のインタフェース（データ通信、依存関係の記述など）の仕組みが整えられていることがROSの特徴である。



- 自動的に作られるファイルについて

パッケージを作成すると、フォルダと、`package.xml`、`CMakeLists.txt`が作成される。

`packege.xml`は、パッケージ名、パッケージの作者、ライセンス、パッケージの依存関係、メタデータを記述することができるファイルである。パッケージ名は、フォルダの名前で決まるのではなく、このxmlファイルの中身で決まる。`rospack find [package name]`や`roscd [package name]`でこれらを確かめられる。

`CMakeLists.txt`は、ビルドについての設定ファイルであり、cmakeというC++のビルドで使われるツールで使われるファイルでもある。どのソースコードを使ってどのバイナリ（PCが直接実行できるファイル）を作るか、どの外部ライブラリを使うか、PC中にインストールするならどの場所にインストールするのか、などを記述する。このファイル自体はROS以外のC++プログラムでも使われるものだが、ROSでは一般的に使われているcmakeの文法に加えて、独自の命令も使う。



#### 画像を用意する

```bash
roscd ros_tutorial
mkdir image

# image配下に画像をダウンロードして置く
```

今回のチュートリアルで使う画像を配置する。



#### ソースファイルをコーディングする

`src/image_subscriber.cpp`、`src/image_publisher.cpp`のソースファイルをコーディングする。本リポジトリのソースコード中にある画像ファイルのパスは、各自の環境に合わせて変更する。

`src/image_subscriber.cpp`は、トピック通信で画像をサブスクライブ（受信）してファイルにして保存するノード、``src/image_publisher.cpp`は、画像ファイルを読み込んでトピック通信でパブリッシュ（送信）するノードである。

プログラミングの具体的な内容についてはソースコード中のコメントを参照。はじめに`src/image_publisher.cpp`を書き写し、そのあとで`src/image_subscriber.cpp`を写経するとよい。



応用として、これらの処理をクラスで記述した`src/image_subscriber_class.cpp`、`src/image_publisher_class.cpp`のサンプルも用意してある。近年の主流のプログラミングパラダイムであるオブジェクト指向に基づく実装方法である。



#### CMakeLists.txtを更新する

ROSのビルドにはcmakeを拡張したcatkinのビルドシステムを用いる。

cmakeの文法については様々なものがある。インストールしてプログラムを公開する際にはきちんと書かなければならないが、最低限以下が使えれば何かしらのブログラムは実行できるだろう。説明の詳細は調べるかネット上のパッケージでの使い方を読むなどして学ぶと良い。

- `cmake_minimum_required`
  cmakeの最低バージョンを指定する

- `project`
  プロジェクト名の指定。ROSの場合、必ずパッケージ名と同じにしなくてはならない

- `find_package`
  外部のライブラリを検索し、指定されたものがコンピュータ内に存在するかチェックする。ROSのパッケージについてはもちろん、ROSに関連しないライブラリ（例えば、OpenCV、Eigen、g2o、octomap、pcl、など）も指定する

- `catkin_package`
  catkinビルドシステムに関連した設定を行う。

- `include_directories`
  このプロジェクトでビルドするソースファイルで使われるヘッダーファイルがある場所。

- `add_executable`
  作成する実行ファイルの名前とそれを生成するソースファイルを指定する。

- `add_library`
  作成するライブラリの名前とそれを作成するソースファイルを指定する。

- `target_link_libraries`
  作成する実行ファイル、またはライブラリにリンクする外部ライブラリを指定する。



#### ビルドする

```bash
# $HOME 以外の場所にワークスペースを作っている場合は適宜変更する
cd ~/catkin_ws
catkin_make
```

このコマンドを実行すると、CMakeLists.txtを読み込んでワークスペース中のすべてのパッケージの依存関係をチェックした後、コンパイル、リンクを行う。

メッセージが大量に流れるが、エラーが出る場合はその内容をきちんと読もう。



#### 実行してみる

```bash
# terminal 1
# ROSのマスターノードを起動
roscore

# terminal 2
# publishするノードを起動
# rosrun [package name] [executable_name]
rosrun ros_tutorial ros_tutorial_image_publisher


# terminal 3
# subscribeするノードを起動
# rosrun [package name] [executable_name]
rosrun ros_tutorial ros_tutorial_image_subscriber
```

ここでのexecutable_nameは、`CMakeLists.txt`で指定したターゲットの名前になる。



## 2 launchファイルの記述

ROSではノードを複数起動することが多い。launchファイルは、ROSのノードをまとめて起動するための設定ファイルであり、xml形式で記述される。複数のノードをまとめて起動する機能の他に、ROSシステムでのパラメータの設定、引数処理、インクルード処理、簡単な条件分岐などを行うことも可能である。

ここではファイルの中身は示さないので、これも書き写して学ぶこと。



1. `basis.launch`

```bash
# 同時に2つのノードを起動するlaunchファイル basis.launch を起動する
# roslaunch [package name] [launch file name]
roslaunch ros_tutorial basis.launch
```

ファイル内について簡単に説明する。node要素に示された各属性値の意味は以下の通りである。

pkg属性...パッケージ名

type属性...実行ファイル名

name属性...マスタに登録するノードの名前

launchファイルで起動する場合、ROSのマスタを起動する必要はない。



2. `argument_parameter.launch`

arg要素を用いることで、引数を与えて実行させることができる。

```bash
# arg1に引数hogehogeを与える例
roslaunch argument_parameter.launch arg1:=[hogehoge]
```

param要素を用いることで、launchファイルを実行する際にROSマスタにパラメータを登録することができる。

nodeタグの中にparamを指定することもできる。この場合、パラメータの名前は`[ノード名]/[パラメータ名]`になる。namespaceの概念と同様に考えるとわかりやすい。このlaunchファイルを実行した後、以下のようなコマンドを実行することで確認することができる。

```bash
# マスタに登録されているパラメータを確認するコマンドの例
rosparam list
rosparam get [paramter name]
rosparam set [aparameter name] [value]
```



パラメータをROSプログラムから取得する方法については、`src/get_parameter.cpp`にサンプルがある。



3. `include.launch`

includeタグを用いることで、他のlaunchファイルの内容を取り込んで実行することができる。launchファイルを内容ごとに分割し、再利用しやすくする仕組みの一つである。



---



以上を踏まえた上で、`orb_slam2_ros`のパッケージのlaunchファイルを見ると理解が深まるだろう。



## 3 応用：orb_slam2_rosパッケージ

応用編として、[orb_slam2_ros](https://github.com/appliedAI-Initiative/orb_slam_2_ros)パッケージの中で今回の内容がどのように使われているかについて見たあと、今回作成したプログラムで画像が与えられていることを確認する。

#### CMakeLists

まず、`CMakeLists.txt`を見てみる。

以下の部分では、orb_slam2の共有ライブラリを作成している。ライブラリなので以下の部分ではmain関数を持たず、他のプログラムから呼び出される形で利用される。

```cmake
# L96（96行目）
add_library(${PROJECT_NAME} SHARED
orb_slam2/src/System.cc
orb_slam2/src/Tracking.cc
orb_slam2/src/LocalMapping.cc
orb_slam2/src/LoopClosing.cc
orb_slam2/src/ORBextractor.cc
orb_slam2/src/ORBmatcher.cc
orb_slam2/src/FrameDrawer.cc
orb_slam2/src/Converter.cc
orb_slam2/src/MapPoint.cc
orb_slam2/src/KeyFrame.cc
orb_slam2/src/Map.cc
orb_slam2/src/Optimizer.cc
orb_slam2/src/PnPsolver.cc
orb_slam2/src/Frame.cc
orb_slam2/src/KeyFrameDatabase.cc
orb_slam2/src/Sim3Solver.cc
orb_slam2/src/Initializer.cc
)
```

以下の部分では、ライブラリのリンクと依存関係の設定を行っている。`${}`は、変数を示している。

```cmake
# L55
set(LIBS_ORBSLAM
${OpenCV_LIBS}
${EIGEN3_LIBS}
${PROJECT_SOURCE_DIR}/orb_slam2/Thirdparty/DBoW2/lib/libDBoW2.so
${PROJECT_SOURCE_DIR}/orb_slam2/Thirdparty/g2o/lib/libg2o.so
)


## ~~~~~~~~~~~~~~~~~~~~

# L115
add_dependencies (${PROJECT_NAME} g2o DBoW2)

# L117
target_link_libraries(${PROJECT_NAME}
${LIBS_ORBSLAM}
)
```

以下でrgbdの場合の実行ファイルと依存関係の設定、リンクをしている。

```cmake
# L149
add_executable (${PROJECT_NAME}_rgbd
ros/src/RGBDNode.cc
ros/src/Node.cc
)
add_dependencies (${PROJECT_NAME}_rgbd ${PROJECT_NAME} ${PROJECT_NAME}_gencfg)

target_link_libraries(${PROJECT_NAME}_rgbd
${LIBS_ROS}
)
```



#### ソースコード

rgbdを用いたSLAMの場合、main関数は`orb_slam_2_ros/ros/src/RGBDNode.cc`にある。またここでは画像をサブスクラブしたときのコールバック関数`ImageCallback()`が実装されており。サブスクライブした画像の処理を行っていることが確認できる。

コールバック関数の登録を行っているコードは`RGBDNode::RGBDNode()`関数にあるが、rgb画像とdepth画像を同期させてサブスクライブする必要があるため多少複雑になっている。また、正確に言えばこの関数は関数のなかでもクラスのコンストラクタと言われるものであり、17行目で変数が生成される際に自動的に呼び出される処理になっている。このあたりの処理の流れについてはC++でのクラスの実装とオブジェクト指向について勉強してほしい。



#### 画像を与えてみる

orb_slam2_testの、トピック名のみを変えることで、今回作成したプログラムから画像を与えられることが確認できる。

`orb_slam2_ros/ros/launch/orb_slam2_d435_rgbd.launch`の以下の部分を、今回作成したノードがpublishするトピック名に揃えてみる。

```xml
       <remap from="/camera/rgb/image_raw" to="/camera/color/image_rect_color" />
       <remap from="/camera/depth_registered/image_raw" to="/camera/depth/image_rect_raw" />
```

本当はカメラパラメータなども変えないといけないが、この部分だけ変えればorb_slamが動くことは確認できる。

```sh
roslaunch orb_slam2_ros orb_slam2_d435_rgbd.launch
```



## 4 補足など

### 4-1 プログラミング基礎用語

ROSとは関係のない一般的な用語についての簡単なリファレンス。筆者の理解に基づくものなので、正確性は保証しない。

- ビルド
  プリプロセス→コンパイル→リンク によって、ソースコードから実行ファイルやライブラリを作ること
- プリプロセス
  C++のビルドの手順の1つで、#include命令や#define命令を展開する（置き換える）処理などを指す
- コンパイル
  ソースファイルを1ファイルずつ対応させて機械語で書かれたオブジェクトファイルにする処理
- リンク
  1つずつのオブジェクトファイルをまとめ、実行ファイルやライブラリを作る
- 実行ファイル
  プログラムがそのまま実行できる形式（機械語：バイナリ形式）のファイル
- ライブラリ
  他の実行ファイルから呼び出されて実行されるファイル
- 依存関係
  あるプログラムの実行のために、別のプログラムの存在を必要としていること、そのことを前提としてプログラムが作られていること。
- cmake
  ビルド設定ツールの名称。cmake自体はROSに依存しないツールであり、クロスプラットフォーム（windowsでもlinuxでも使用可能）である。ソースコードの場所、ヘッダファイルの場所、ライブラリの場所、インストールする場合の場所など、プログラムのビルドには多くの設定ファイルを必要とする。cmakeを使用するとそれらをOSに依存しない形で記述することができる。記述にはcmake独自の文法を用いる。
- xmlファイル
  ファイル形式の1つ。構造化データをタグと属性を用い記述する。
- yamlファイル
  ファイル形式の1つ。xmlと同じように構造化データを記述することができる。



### 4-2 ROSの用語

- マスタ
  1つのROSシステム全体を管理するプログラム。`roscore`コマンドで起動するか、launchファイルを用いて自動的に起動させる。具体的に行っている処理の例としては、どの名前のどのノードがシステムで動いて動いているか、どのトピック、サービスが存在しているか、ROSシステムで登録されているパラメータは何がありどんな値なのか、シミュレーション時間か実時間かの管理、などが挙げられる。
- トピック
- ノード
- パブリッシュ
- サブスクライブ
- メッセージ
  



### 4-3 参考資料

- [ROSロボットプログラミングバイブル、オーム社](https://www.ohmsha.co.jp/book/9784274221965/)
- ROS公式HPの記事
  - [ROSパッケージを作る](http://wiki.ros.org/ja/ROS/Tutorials/CreatingPackage)
  - [publisher subscriberを書く](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29)
  - [image_transportでpublishする](http://wiki.ros.org/image_transport/Tutorials/PublishingImages)
  - [cv_bridge](http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages)
  - [CMakeLists.txt](http://wiki.ros.org/ja/catkin/CMakeLists.txt)
  - [roslaunch](http://wiki.ros.org/roslaunch)
  - [sensor_msgs::Image](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html)
  
- [orb_slam2_ros](https://github.com/appliedAI-Initiative/orb_slam_2_ros)



### さいごに

ここまで挙げてきた機能はほんの一部である。上記の参考資料や公式リファレンス、信頼性の高い公開パッケージなどで調べることでより便利なツールを学ぶことができる。