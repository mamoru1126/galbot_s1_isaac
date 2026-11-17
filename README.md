# galbot_s1_isaac

[Galbot S1](https://github.com/GalaxyGeneralRobotics/galbot_s1_description) を **NVIDIA Isaac Sim 6.x** で動かすための環境です。
走行・腕・グリッパ・胴・頭の操作と「机の上の物を掴んでトレイに置く」デモを **Web ブラウザから** 実行できます。
Gazebo 版 [mamoru1126/galbot_s1_gazebo](https://github.com/mamoru1126/galbot_s1_gazebo) と同じ ROS 2 トピック・同じ commander・同じ Web UI を使い、シミュレータだけを Isaac Sim に置き換えています。

> [!IMPORTANT]
> **検証状況** — このリポジトリを作った環境には GPU が無く、**Isaac Sim 本体での起動はまだ確認できていません**。
>
> | 確認済み | 未確認 (RTX マシンでの確認待ち) |
> |---|---|
> | Isaac Sim 6.2.0 のソースと API 名・引数を照合 (URDF インポータ、Articulation / RigidPrim、SimulationManager、Replicator) | Isaac Sim 上での起動、URDF→USD 変換結果 |
> | CI: Isaac 用の代替バックエンド (kinematic) で ROS 2 スタック全体の E2E (走行・ジョグ・グリッパ・赤い缶のデモ・左腕で箱の把持と戻し・リセット) | PhysX の挙動 (ゲイン、把持中の追従) |
> | CI: USD シーン生成 (テーブル・物体・カメラの向き) を usd-core で検査 | RTX カメラ画像、RTX 3090 での速度 |
>
> RTX 3090 のマシンで [`./scripts/self_test.sh`](#自己テスト-rtx-マシンで) を実行し、出力フォルダ (`selftest/…`) のログを共有してもらえれば、それを見て調整します。

```
ブラウザ (PC / スマホ)
   │  http://<host>:8000         ws://<host>:9090
   ▼                                  ▼
galbot_s1_web (静的ページ) ──── rosbridge ────┐
                                              │ /galbot/command, /galbot/status, /cmd_vel, カメラ (JPEG)
                                              ▼
galbot_s1_control  commander (IK・把持・デモ) / base_controller (全方向移動) / camera_relay
                                              │ /<group>_controller/commands, /galbot/base/cmd_vel,
                                              │ /galbot/sim/attach|detach|set_model_pose  (Gazebo 版と同じ)
                                              ▼
galbot_s1_isaac    isaac/run_sim.py  ── Isaac Sim 6.x (PhysX + RTX)  /clock, /joint_states, odom, 画像
                                     └─ (--backend kinematic: GPU なしの代替。CI 用)
galbot_s1_description  公式 URDF から変換したモデル (メッシュ・慣性・関節)
```

## パッケージ

| パッケージ | 内容 |
|---|---|
| `galbot_s1_description` | ロボットモデル (xacro + COLLADA/STL)。Gazebo 版と同じもの |
| `galbot_s1_control` | commander / base_controller / camera_relay。Gazebo 版と同じコード |
| `galbot_s1_web` | ブラウザ用コンソール (HTML/JS のみ) と静的 Web サーバ |
| `galbot_s1_isaac` | Isaac Sim アプリ (`isaac/run_sim.py`)、シーン・設定 (`config/`)、Isaac 用 URDF (`urdf/`)、launch |

## 動作環境

| | |
|---|---|
| GPU | RTX 3090 (24 GB) を想定。**Isaac Sim 6.2 の公式要件表の最小は RTX 4080 で、3090 は表に載っていません** (RT コア付き・VRAM 24 GB なので動く見込みですが公式サポート外)。 |
| ドライバ | Isaac Sim 6.2 のドキュメントの推奨は 595.58.03。コンテナ自体の最低チェックは 570.169 |
| OS | Ubuntu 22.04 / 24.04 (Docker)。ネイティブ実行は Ubuntu 24.04 + ROS 2 Jazzy |
| ROS | **ROS 2 Jazzy のみ**。Isaac Sim 6.x は ROS 1 をサポートしていないので、ROS 1 で使う場合は Gazebo 版を使ってください |
| Isaac Sim | 6.2.0 (コンテナ `nvcr.io/nvidia/isaac-sim:6.2.0`)。ほかの 6.x も `--isaac-tag` で指定可 |
| その他 | Docker + NVIDIA Container Toolkit、数十 GB のディスク空き (Isaac Sim のイメージが大きい)、イメージ作成時のインターネット接続 |

## Docker で動かす (おすすめ)

イメージを作ります (Isaac Sim 6.2.0 の上に ROS 2 Jazzy とこのワークスペースを入れます。初回は Isaac Sim イメージのダウンロードで時間がかかります)。

```bash
git clone https://github.com/mamoru1126/galbot_s1_isaac.git
cd galbot_s1_isaac
./scripts/docker_build.sh
```

Isaac Sim のコンテナは `ACCEPT_EULA=Y` (イメージのライセンスへの同意) が必要です。
[Container Installation](https://docs.isaacsim.omniverse.nvidia.com/6.2.0/installation/install_container.html) のライセンスを確認してから設定してください。

```bash
export ACCEPT_EULA=Y
./scripts/docker_run.sh
```

ブラウザで **http://localhost:8000** を開きます (同じ LAN のスマホからは `http://<PC の IP>:8000`)。

- デスクトップ環境 (`DISPLAY` あり) では Isaac Sim のウィンドウも開きます。ウィンドウなしは `./scripts/docker_run.sh --headless`
- **初回起動は時間がかかります**: URDF→USD 変換と RTX のシェーダコンパイルで数分〜十数分。
  キャッシュは `~/docker/isaac-sim` (変数 `ISAAC_SIM_DATA` で変更可) に残るので 2 回目からは速くなります
- コンテナ内の Isaac Sim はユーザー uid 1234 で動くため、キャッシュフォルダの所有者を最初に 1234 に変更します (スクリプトが自動で行います)

その他のオプション:

```bash
./scripts/docker_run.sh --headless
./scripts/docker_run.sh -- sim_args:=--rebuild-usd
./scripts/docker_run.sh -- cameras:=false
./scripts/docker_run.sh --shell
```

`sim_args:=` には `isaac/run_sim.py` の引数 (`--rebuild-usd`, `--no-cameras`, `--realtime-factor 0`, `--physics-device cuda:0`, `--attach-mode joint` など) を渡せます。

### GPU なしで UI だけ試す

Isaac Sim の代わりに kinematic バックエンド (関節が目標に追従するだけの代替。物理なし・カメラは仮画像) で、Web UI と commander の動作を確認できます。CI と同じ構成です。

```bash
./scripts/docker_build.sh --ci
./scripts/docker_run.sh --kinematic
```

## ネイティブで動かす (Ubuntu 24.04 + ROS 2 Jazzy)

Isaac Sim 6.x の Python は 3.12 で ROS 2 Jazzy と同じなので、ROS 2 を source した状態で Isaac Sim の Python からそのまま rclpy を使います。

```bash
sudo apt install ros-jazzy-ros-base ros-jazzy-rosbridge-server ros-jazzy-robot-state-publisher ros-jazzy-xacro python3-colcon-common-extensions python3-opencv
mkdir -p ~/galbot_ws/src
cd ~/galbot_ws/src
git clone https://github.com/mamoru1126/galbot_s1_isaac.git
cd ~/galbot_ws
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

Isaac Sim の場所を教えて起動します (ワークステーション版は `python.sh` のあるフォルダ、pip 版は venv の python)。

```bash
export ISAACSIM_PATH=$HOME/isaacsim
ros2 launch galbot_s1_isaac sim.launch.py
```

```bash
export ISAACSIM_PYTHON=$HOME/isaac-venv/bin/python
ros2 launch galbot_s1_isaac sim.launch.py headless:=true
```

シミュレータと ROS 側を別々に起動することもできます。

```bash
$ISAACSIM_PATH/python.sh ~/galbot_ws/install/galbot_s1_isaac/share/galbot_s1_isaac/isaac/run_sim.py
```

```bash
ros2 launch galbot_s1_isaac bringup.launch.py
```

### 起動引数 (`sim.launch.py`)

| 引数 | 既定値 | 内容 |
|---|---|---|
| `backend` | `isaac` | `isaac` / `kinematic` (GPU なしの代替) |
| `headless` | `false` | Isaac Sim のウィンドウを出さない |
| `cameras` | `true` | 頭・手首・俯瞰カメラの画像を描画・配信する |
| `web` / `web_port` / `rosbridge_port` | `true` / `8000` / `9090` | Web UI |
| `rviz` | `false` | RViz2 |
| `scene_file` | `config/scene.yaml` | シーン |
| `sim_args` | | `run_sim.py` への追加引数 |

## Web UI

`http://<host>:8000` を開くと、カメラ映像 (俯瞰 / 頭 / 右手首 / 左手首)、上から見た図、操作パネルが表示されます。右上の **停止** ボタン (またはスペースキー) はいつでもすべての動作を止めます。

- **デモ** : 物体 (赤い缶 / 緑のボトル / 青い箱) を選び、「▶ 全自動デモ」で *テーブルへ移動 → 掴んで持ち上げる → トレイに置く → スタート位置へ戻る* を実行します。各ステップを個別に実行するボタンと「シーンをリセット」もあります。腕は物体の位置から自動で選びます (左右の指定も可)。
- **走行** : ジョイスティックで全方向移動、⟲⟳ で旋回。キーボードは W/S 前後・A/D 左右・Q/E 旋回。
- **アーム** : 手先を前後・左右・上下・回転 (押し続けで連続)、グリッパの開閉、7 関節のスライダ、姿勢プリセット。「閉じる/掴む」は指の間に物体があればそれを掴みます。
- **胴・頭** : リフト (0〜0.735 m) と首の左右・上下。

commander への JSON コマンド (`/galbot/command`, 例: `{"type": "task", "name": "demo", "object": "red_can"}`) や状態 (`/galbot/status`) は Gazebo 版と同じです (一覧は [Gazebo 版 README](https://github.com/mamoru1126/galbot_s1_gazebo) の「commander への JSON コマンド」)。

## Isaac Sim 側の作り

| | |
|---|---|
| ロボット | `urdf/galbot_s1_isaac.urdf` を Isaac Sim の URDF インポータで USD に変換 (キャッシュ)。公式の USD は ArticulationRoot が無くグリッパのミミック関節の結合も無いため、Gazebo 版で修正済みの URDF から作っています |
| ベース | ルートを固定し、`base_x_joint` / `base_y_joint` (直動) と `base_yaw_joint` (回転) の 3 関節を硬い位置ドライブで動かす平面ベース。`/galbot/base/cmd_vel` を物理ステップごとに積分して目標にします (Gazebo 版の平面ベースと同じ考え方)。車輪はスワーブの見た目用で、衝突形状なし |
| 関節 | 腕・胴・頭・グリッパ・ステアは位置ドライブ、駆動輪は速度ドライブ。ゲインは `config/isaac_sim.yaml` の `robot.drives`。commander の目標の変化から速度フィードフォワードも与えます |
| 重力 | ロボットのリンクは重力オフ (= 理想的な重力補償。実機と同様に姿勢保持のためのゲインを小さくできる) |
| 把持 | `attach_mode: follow` (既定): 掴んだ物体を毎物理ステップ手首リンクに合わせて置く。`joint`: PhysX の固定ジョイントを実行時に作成 |
| カメラ | 頭・左右手首のカメラはリンクの子として配置、俯瞰カメラは固定。Replicator の render product + `rgb` annotator で取得し、購読者がいるときだけ配信 |
| シーン | `config/scene.yaml` から USD で生成 (テーブル・トレイ・物体・ライト・地面)。Gazebo 版と同じファイル |
| トピック | `/clock` `/joint_states` `/galbot/ground_truth/odom` `/galbot/objects/<name>/odom` `/galbot/<camera>/image_raw` などを rclpy で直接配信 (OmniGraph 不使用) |

### RTX 3090 での負荷を下げる

- `--headless` (ウィンドウの描画が無くなる)
- `config/isaac_sim.yaml` の `cameras` で解像度・レート (`rate`) を下げる、または `cameras:=false`
- レンダラは既定で軽い `RaytracedLighting` (`--renderer RealTimePathTracing` で Isaac Sim 既定の描画)
- 物理は CPU (1 台なら CPU の方が軽い)。`--physics-device cuda:0` で GPU に変更可
- シミュレーションが実時間より遅い場合でも、commander などは `/clock` (シミュレーション時間) で動くので動作は正しく、ゆっくりになるだけです

## 自己テスト (RTX マシンで)

Isaac Sim をヘッドレスで起動し、CI と同じ E2E テスト (起動 → 走行 → アームのジョグ → グリッパ → 赤い缶のデモ → 左腕で青い箱 → リセット) を実行します。

```bash
export ACCEPT_EULA=Y
./scripts/self_test.sh
```

結果は `selftest/<日時>_isaac/` に出ます: `result.json` (各項目の合否)、`launch.log` (Isaac Sim と ROS ノードのログ)、`client.log`、`kit.log` (Isaac Sim の詳細ログ)、`diagnostics.log` (`/clock` やカメラのレート)、`system.txt` (`nvidia-smi`)、`frames/*.jpg` (途中のカメラ画像)。
うまく動かないときはこのフォルダをそのまま共有してください。

GPU なしでテスト自体を確認するには `./scripts/self_test.sh --kinematic` (イメージは `./scripts/docker_build.sh --ci`)。

## 設定を変える

- `galbot_s1_isaac/config/isaac_sim.yaml` : 物理 (刻み・CPU/GPU)、実時間係数、ゲイン、把持方式、カメラ
- `galbot_s1_isaac/config/scene.yaml` : テーブル・トレイ・物体の位置や大きさ、俯瞰カメラ (commander も同じファイルを読みます)
- ロボットモデルを変えたとき (`galbot_s1_description` の xacro) は Isaac 用 URDF を作り直します (ROS 2 を source した状態で):

```bash
python3 galbot_s1_isaac/tools/generate_urdf.py
```

URDF やゲインを変えると次回起動時に USD が自動で作り直されます (キャッシュのキーに含まれます)。

## テスト (CI)

GitHub Actions には RTX GPU が無いため、Isaac Sim 自体は動かせません。CI では

- `ci/unit_tests.sh` : コミット済み URDF が xacro と一致しているか + pytest (運動学・Isaac 用 URDF・ゲイン設定・シミュレーションループ)
- `ci/run_sim_test.sh` : kinematic バックエンドで ROS 2 スタック全体の E2E (`ci/sim_test_client.py`)
- usd-core で USD シーン生成のテスト (`galbot_s1_isaac/test/test_usd_scene.py`)

を実行しています。kinematic バックエンドは Isaac Sim バックエンドと同じシミュレーションループ (`isaac/galbot_isaac/sim_core.py`) とトピックを使います。

## トラブルシューティング

- **`ACCEPT_EULA is not Y`** : 上の「Docker で動かす」を参照
- **キャッシュフォルダの権限エラー** : `~/docker/isaac-sim` の所有者が uid 1234 になっているか確認 (`docker_run.sh` が自動で変更します)
- **ロボットが出ない / USD の変換エラー** : `-- sim_args:=--rebuild-usd` で作り直し。`launch.log` の `[galbot_isaac]` の行を確認
- **Isaac Sim のウィンドウが開かない** : コンテナの GUI はローカルのデスクトップが必要です (リモートデスクトップ・VNC は非対応)。`--headless` で Web UI だけ使えます
- **Isaac Sim の GUI で Stop を押した** : 物理が止まるので、アプリを再起動してください
- **ホストの ROS 2 からトピックが見えない** : `--network=host` で動かしているので、`ROS_DOMAIN_ID` をそろえてください (既定 0)

## ライセンス

ロボットモデル (メッシュ・URDF) は [GalaxyGeneralRobotics/galbot_s1_description](https://github.com/GalaxyGeneralRobotics/galbot_s1_description) (Apache License 2.0) からの派生です。変更点は `galbot_s1_description/NOTICE` を参照してください。
このリポジトリのその他のコードも Apache License 2.0 としています。Isaac Sim 本体とコンテナイメージは NVIDIA のライセンスに従います。
