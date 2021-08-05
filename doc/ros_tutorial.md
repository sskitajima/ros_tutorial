# ROSチュートリアル第1回

- 前提条件

  - ROSのインストール

  - 基本的なlinuxコマンドや操作

    



- 目的

実際の開発の中でよく使われるであろうROS開発知識を、**ROSの概念やツールの包括的な説明を放棄することにより短時間で**、そして**実際に手を動かすことで確実に**身につけること。



- 内容

1. ROSのパブリッシャとサブスクライバを書く
   - パッケージを作る
   - 画像を読み込んでpublish
   - 画像をsubscribeして保存
2. launchファイルを書く
   - 基本と、いくつかのタグの説明
3. orb_slam2_rosに画像を与えてみる



### 1.簡単なROSノードの作成

1. **パッケージを作る**

```sh
# catkin_create_pkg [package name] [dependent packages]
catkin_create_pkg ros_tutorial roscpp rospy image_transport cv_bridge
```

パッケージを作成した後でも、依存関係は追加することができる。



- ROSのパッケージの意義

ROSでは、パッケージと呼ばれる単位でプログラムを管理する。パッケージで区別してプログラムを区切ると、プログラムの再利用がしやすくなる。簡単に言えば、処理Aを行うプログラムと処理Bを行うプログラムを合わせてモジュールXを最初に作成し、その後、処理Aと処理Cを合わせたモジュールYを作成したくなったとき、処理Aと処理Bが密結合したコードになっていると、プログラムの再利用がしにくい。処理を分けて、その間のインタフェース（データ通信、依存関係の記述など）の仕組みが整えられていることがROSの特徴である。



- 自動的に作られるファイルについて

パッケージを作成すると、フォルダと、`package.xml`、`CMakeLists.txt`が作成される。

`packege.xml`は、パッケージ名、パッケージの作者、ライセンス、パッケージの依存関係、メタデータを記述することができるファイルである。パッケージ名は、フォルダの名前で決まるのではなく、このxmlファイルの中身で決まる。`rospack find [package name]`や`roscd [package name]`でこれらを確かめられる。

`CMakeLists.txt`は、ビルドについての設定ファイルであり、cmakeというC++のビルドで使われるツールで使われるファイルでもある。どのソースコードを使ってどのバイナリ（PCが直接実行できるファイル）を作るか、どの外部ライブラリを使うか、PC中にインストールするならどの場所にインストールするのか、などを記述する。このファイル自体はROS以外のC++プログラムでも使われるものだが、ROSでは一般的に使われているcmakeの文法に加えて、独自の命令も使う。



2. **画像を用意する**

```bash
roscd ros_tutorial
mkdir image

# image配下に画像を置く
```

今回のチュートリアルで使う画像を配置する。



3. **ソースファイルをコーディングする**

画像ファイルのパスは変更する。

具体的な内容についてはソースコード中のコメントを参照。



3. **CMakeLists.txtを更新する**

cmakeの文法については様々なものがある。インストールしてプログラムを公開する際にはきちんと書かなければならないが、最低限以下が使えれば何かしらのブログラムは実行できるだろう。

`cmake_minimum_required`

`project`

`find_package`

`catkin_package`

`include_directories`

`add_executable`

`add_library`

`target_link_libraries`

3. **ビルドする**

```bash
# $HOME 以外の場所にワークスペースを作っている場合は適宜変更する
cd ~/catkin_ws
catkin_make
```

CMakeLists.txtを読み込んでワークスペース中のすべてのパッケージの依存関係をチェックした後、コンパイル、リンクを行う。

メッセージが大量に流れるが、エラーが出る場合はその内容をきちんと読もう。



5. **実行してみる**

```bash
# terminal 1
roscore

# terminal 2
rosrun ros_tutorial ros_tutorial_image_publisher


# terminal 3
rosrun ros_tutorial ros_tutorial_image_subscriber
```



## 2.launchファイルの記述

ROSではノードを複数起動することが多い。launchファイルは、ROSのノードをまとめて起動するための設定ファイルであり、xml形式で記述される。複数のノードをまとめて起動する機能の他に、ROSシステムでのパラメータの設定、引数処理、インクルード処理、簡単な条件分岐などを行うことも可能である。

1. `basis.launch`

```bash
# roslaunch [package name] [launch file name]
roslaunch ros_tutorial argument_parameter.launch
```

node要素に示された各属性値の意味は以下の通りである。

pkg属性...パッケージ名

type属性...実行ファイル名

name属性...マスタに登録するノードの名前



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



3. `include.launch`

includeタグを用いることで、他のlaunchファイルの内容を取り込んで実行することができる。launchファイルを内容ごとに分割し、再利用しやすくする仕組みの一つである。

---

以上を踏まえた上で、`orb_slam2_ros`のパッケージのlaunchファイルを見ると理解が深まるだろう。

## 3.orb_slam2_rosでの実行

orb_slam2_testの、トピック名のみを変えることで、今回作成したプログラムから画像を与えられることが確認できる。



### プログラミング基礎用語

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

- xmlファイル
- yamlファイル

### ROSの用語

- マスタ
  1つのROSシステム全体を管理するプログラム。`roscore`コマンドで起動するか、launchファイルを用いて自動的に起動させる。具体的に行っている処理の例としては、どの名前のどのノードがシステムで動いて動いているか、どのトピック、サービスが存在しているか、ROSシステムで登録されているパラメータは何がありどんな値なのか、シミュレーション時間か実時間かの管理、などが挙げられる。
- トピック
- ノード
- パブリッシュ
- サブスクライブ

### 参考資料

- ROSロボットプログラミングバイブル、オーム社

- ROS公式HPのチュートリアル
  - [publisher subscriberを書く](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29)
  - [image_transportでpublishする](http://wiki.ros.org/image_transport/Tutorials/PublishingImages)
  - [cv_bridge](http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages)
  - [roslaunch](http://wiki.ros.org/roslaunch)

## 最後に

ここまで挙げてきた機能はほんの一部である。上記の参考資料や公式リファレンス、信頼性の高い公開パッケージなどで調べることでより便利なツールを学ぶことができる。