# joycon_probe_control
プローブ走査機構の制御をするプログラム

## 起動方法
### アクセス権限の設定
なし

### ノードの起動
roslaunch parallel_link_control parallel_link_control.launch

## 入力モード名
### automatic
* 自動僧帽弁探索モード <br>

### joy
* ジョイスティックで操作するモード <br>
#### 動作方法
* 動作切り替えの際は毎回後ろのボタン(button1)を押す <br>
* レバーの操作でプローブのxy位置を操作 <br>
* button1を押しながらレバーを操作することでパラレルリンクの姿勢を調整可能 <br>

### manual
*  手動でプローブ走査機構を制御(x, y, roll, pitch, yaw) <br>
