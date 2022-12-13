# mypkg
## 概要
![test](https://github.com/18C1054-S-K/mypkg/actions/workflows/test.yml/badge.svg)<br>
ロボットシステム学の課題で作成したROS2パッケージです。<br>
授業で作成したtalker、listenerの他、整数を2つの平方数の和に分解する機能が入っています。


## 動作確認済み環境
Ubuntu20.04<br>
ROS2 foxy


## 利用にあたって
このパッケージでは[mypkg_msgs](https://github.com/18C1054-S-K/mypkg_msgs)で定義されたメッセージ、サービスを用いています。<br>
そのため、予めmypkg_msgsがcloneされている必要があります。
#### cloneの方法
<ros2_ws>/srcに移動した上で
```
git clone https://github.com/18C1054-S-K/mypkg_msgs
```
(ただし<ros2_ws>はROS2ワークスペースです。)


## ノード
- talker
- listener
  - 省略
- prime_factorizer
  - /countupトピック(Int16型)で受け取った整数を素因数分解して、<br>
    結果を/primesトピック(Primes型)で渡します。
- result
  - /primesトピック(Primes型)で受け取った素因数分解の結果から、<br>
    その整数が2つの平方数の和として表せるかを判定し、表せないならばその旨をログに出力、<br>
    表せるならば/calc_sq_sumサービス(CalcSqSum型)に計算させて、結果をログに出力します。
- sqsum_calculator
  - /calc_sq_sumサービス(CalcSqSum型)のサーバーです。<br>
    受け取った素因数分解の結果を元に2つの平方数の和に分解して返します。


## launchファイル
- talk_listen
  - 省略
- sqsum
  - talker、prime_factorizer、sqsum_calculator、resultを起動します。<br>
    一定時間ごとに0から昇順に整数を2つの平方数の和に分解し、結果をログに出力します。


## メッセージ、サービス
- mypkg_msgs/msg/Primes
  - 整数、素因数の配列、対応する指数の配列からなるメッセージです。<br>
    下記が定義です。
    ```
    int16 original
    int16[] primes
    int16[] indexs
    ```
    例えばoriginal=20なら20=2×2×5なのでprimes=[2,5]、indexs=[2,1]となるようにします。
- mypkg_msgs/srv/CalcSqSum
  - 整数の素因数分解を受け取って、2つの平方数の和に分解するサービスです。<br>
    リクエストは素因数の配列、対応する指数の配列、レスポンスは計算結果の2つの整数です。<br>
    下記が定義です。
    ```
    int16[] primes
    int16[] indexs
    ---
    int16 x
    int16 y
    ```
    x > yになります。
    例えばprimes=[2,5]、indexs=[2,1]のとき、このリクエストは2×2×5=20を表していて、<br>
    この数は20=4^2+2^2と分解できるのでレスポンスはx=4、y=2になります。


## 背景にある背景
二平方和定理によると2より大きな素数pについて、2つの平方数の和で表せる⇔p=1(mod4)　が成り立ちます。<br>
また2つの平方数の和で表せる整数x、yに対し、その積xyも2つの平方数の和で表せます。<br>
(x=a^2+b^2、y=c^2+d^2 ⇒ xy=(ac-bd)^2+(ad+bc)^2)<br>
また1、2は2つの平方数の和として表せます。<br>
(1=1^2+0^2、2=1^2+1^2)<br>
よって　自然数nが2つの平方数の和として表せる⇔素因数分解したときに3mod4の素因数の指数が偶数　が成り立ちます。<br>
resultノードはこれを利用しています。


## 参考
- pythonファイルを書く際に参考にしました。
  - [note.nkmk.me](https://note.nkmk.me/python/)
- README.mdを書く際に参考にしました。
  - [Qiita Markdown記法一覧](https://qiita.com/oreo/items/82183bfbaac69971917f)


## 著作権
このパッケージのコードは、下記のスライド(CC-BY-SA 4.0 by Ryuichi Ueda)のものを、本人の許可を得て自身の著作としたものです。<br>
[ryuichiueda/my_slides_robosys_2022](https://github.com/ryuichiueda/my_slides_robosys_2022)


## ライセンス
MITライセンス<br>
LICENSEをお読みください。
