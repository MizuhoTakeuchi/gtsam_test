# PoseEstimatorNode コード構成

## ファイル一覧

| ファイル | 役割 |
|---------|------|
| `src/pose_estimator_node.cpp` | メインノード実装（ファクターグラフ構築・最適化・パブリッシュ） |
| `launch/pose_estimator.launch.py` | ノード起動設定（パラメータ・リマッピング） |

---

## `src/pose_estimator_node.cpp`

### 全体構造

```
L1-3      ファイルヘッダ（概要コメント）
L5-23     インクルード（ROS2 / GTSAM / STL）
L25       シンボル定義: X() = Pose3変数
L27-88    ユーティリティ関数（ROS ↔ GTSAM変換）
L90-99    StampedMeasurement構造体
L101-375  PoseEstimatorNodeクラス
L377-387  main関数
```

### ユーティリティ関数

| 関数 | 行 | 入力 | 出力 | 説明 |
|------|-----|------|------|------|
| `rosPoseToGtsam()` | L35-43 | `geometry_msgs::Pose` | `gtsam::Pose3` | ROS Pose → GTSAM Pose3 変換 |
| `rosCovarianceToGtsamNoise()` | L45-66 | `std::array<double,36>` | `gtsam::SharedNoiseModel` | ROS共分散 [tx,ty,tz,rx,ry,rz] → GTSAM [rx,ry,rz,tx,ty,tz] 並び替え |
| `gtsamPoseToRos()` | L68-88 | `gtsam::Pose3`, `Time`, `frame_id` | `PoseStamped` | GTSAM Pose3 → ROS PoseStamped 変換 |

### `StampedMeasurement` 構造体（L94-99）

GPS/LiDAR測定値の統一表現。タイムスタンプ順ソートでイベント駆動ノード作成を実現する。

```cpp
struct StampedMeasurement {
  rclcpp::Time stamp;                        // 測定タイムスタンプ
  enum class Source { GPS, LIDAR } source;   // センサ種別
  gtsam::Pose3 pose;                         // 測定ポーズ
  gtsam::SharedNoiseModel noise;             // センサ共分散からのノイズモデル
};
```

### `PoseEstimatorNode` クラス

#### コンストラクタ（L108-164）

```
パラメータ宣言
  ├── optimization_period_ms (default: 500)      タイマー周期 [ms]
  ├── between_noise_translation (default: 1.0)   並進ノイズ密度 [sigma/√s]
  ├── between_noise_rotation (default: 0.1)      回転ノイズ密度 [sigma/√s]
  └── min_dt_for_new_node (default: 0.001)       新ノード作成の最小時間間隔 [s]

ISAM2初期化
  ├── relinearizeThreshold = 0.1
  └── relinearizeSkip = 1

Subscriber (best-effort QoS, depth=10)
  ├── /gps/pose_with_covariance   → gps_buffer_
  └── /lidar/pose_with_covariance → lidar_buffer_

Publisher
  ├── /estimated_pose  (PoseStamped)
  └── /estimated_path  (Path)

TF broadcaster: map → base_link
Timer: optimization_period_ms周期でoptimizationCallback呼び出し
```

#### `optimizationCallback()` メインロジック（L170-341）

```
1. バッファ取得（L172-183）
   mutex lockでGPS/LiDARバッファをswap取得
   両方空なら即return

2. StampedMeasurementへ変換・ソート（L185-209）
   GPS測定値 → StampedMeasurement (Source::GPS)
   LiDAR測定値 → StampedMeasurement (Source::LIDAR)
   タイムスタンプ昇順にstd::sort

3. ファクターグラフ構築ループ（L211-293）
   各測定値について:
   ├── ノード作成判定
   │   ├── have_node == false → 初回ノード必須作成
   │   ├── dt < 0 → 警告ログ出力、スキップ
   │   ├── dt >= min_dt_for_new_node_ → 新ノード作成
   │   └── dt < min_dt_for_new_node_ → 既存ノードに付与
   │
   ├── 新ノード作成時
   │   ├── X(pose_index_) に初期値挿入
   │   ├── 初回: PriorFactor（アンカー）
   │   └── 2回目以降: BetweenFactor
   │       ├── delta = prev_estimate.between(meas.pose)
   │       └── noise = sigma * sqrt(dt)  ← ランダムウォークモデル
   │
   └── PriorFactor（測定値）を現在ノードに追加

4. ISAM2更新（L300-302）
   isam2_->update(new_factors, new_values)  バッチ更新
   isam2_->update()                         追加イテレーション

5. 結果取得・パブリッシュ（L304-341）
   ├── latest_pose_ 更新
   ├── /estimated_pose パブリッシュ（測定タイムスタンプ使用）
   ├── /estimated_path に新規ノード全て追加
   ├── TF (map → base_link) ブロードキャスト
   └── ログ出力: total_nodes, 座標, GPS/LiDAR数, 新規ノード数
```

#### メンバ変数（L343-374）

| カテゴリ | 変数 | 型 | 説明 |
|---------|------|-----|------|
| 排他制御 | `mtx_` | `std::mutex` | バッファアクセス用mutex |
| バッファ | `gps_buffer_` | `vector<PoseWithCovarianceStamped>` | GPS測定値バッファ |
| バッファ | `lidar_buffer_` | `vector<PoseWithCovarianceStamped>` | LiDAR測定値バッファ |
| GTSAM | `isam2_` | `unique_ptr<ISAM2>` | 増分ソルバ |
| GTSAM | `pose_index_` | `size_t` | 次に作成するノードのインデックス |
| GTSAM | `latest_pose_` | `Pose3` | 最新ノードの最適化済みポーズ |
| GTSAM | `prev_stamp_` | `rclcpp::Time` | 最後に作成したノードのタイムスタンプ |
| パラメータ | `between_noise_trans_` | `double` | 並進ノイズ密度 [sigma/√s] |
| パラメータ | `between_noise_rot_` | `double` | 回転ノイズ密度 [sigma/√s] |
| パラメータ | `min_dt_for_new_node_` | `double` | 新ノード作成閾値 [s] |
| ROS | `gps_sub_`, `lidar_sub_` | `Subscription` | センササブスクライバ |
| ROS | `pose_pub_` | `Publisher<PoseStamped>` | 推定ポーズ出力 |
| ROS | `path_pub_` | `Publisher<Path>` | 推定軌跡出力 |
| ROS | `tf_broadcaster_` | `TransformBroadcaster` | TFブロードキャスタ |
| ROS | `timer_` | `TimerBase` | 最適化タイマー |
| 蓄積 | `path_` | `nav_msgs::Path` | 軌跡蓄積用 |

---

## `launch/pose_estimator.launch.py`

```python
Node(
    package='gtsam_test',
    executable='pose_estimator_node',
    parameters=[{
        'optimization_period_ms': 500,       # タイマー周期
        'between_noise_translation': 1.0,    # 並進ノイズ密度
        'between_noise_rotation': 0.1,       # 回転ノイズ密度
        'min_dt_for_new_node': 0.001,        # 新ノード作成閾値
    }],
    remappings=[
        ('/gps/pose_with_covariance', '/gps/pose_with_covariance'),
        ('/lidar/pose_with_covariance', '/lidar/pose_with_covariance'),
    ],
)
```

---

## ファクターグラフの構造

### 変更前（タイマーベース）

```
X(0) ----BetweenFactor---- X(1) ----BetweenFactor---- X(2)
 |                           |                           |
 ├── PriorFactor(GPS@0.0s)  ├── PriorFactor(GPS@0.5s)  ├── ...
 ├── PriorFactor(LiDAR@0.05s)├── PriorFactor(LiDAR@0.55s)
 ├── PriorFactor(LiDAR@0.15s)├── PriorFactor(LiDAR@0.65s)
 ├── PriorFactor(LiDAR@0.25s)├── ...
 ├── PriorFactor(LiDAR@0.35s)
 └── PriorFactor(LiDAR@0.45s)

問題: 異なる時刻の測定値が同一ノードを拘束
      500ms内の軌跡情報が失われる
      BetweenFactorのノイズが時間に依存しない
```

### 変更後（イベント駆動ノード）

```
X(0)--BF--X(1)--BF--X(2)--BF--X(3)--BF--X(4)--BF--X(5)--BF--X(6)
 |         |         |         |         |         |         |
GPS      LiDAR    LiDAR    LiDAR    LiDAR    GPS+LiDAR   LiDAR
@0.0s    @0.1s    @0.2s    @0.3s    @0.4s    @0.5s       @0.6s
                                              (同一ノード)

BF = BetweenFactor（ノイズ ∝ √dt）
各測定値が正確な時刻のポーズノードを拘束
GPS+LiDARが同一タイムスタンプなら同一ノードに付与
```

### BetweenFactorノイズモデル

```
σ_between = σ_density × √dt

σ_density: パラメータで設定するノイズ密度 [sigma/√s]
dt: ノード間の時間差 [s]

理論的背景: ランダムウォークモデルでは共分散がdtに比例
            → 標準偏差が√dtに比例
```

---

## ROSインターフェース

### Subscribe

| トピック | 型 | QoS | レート目安 |
|---------|-----|-----|-----------|
| `/gps/pose_with_covariance` | `PoseWithCovarianceStamped` | best-effort, depth=10 | 1 Hz |
| `/lidar/pose_with_covariance` | `PoseWithCovarianceStamped` | best-effort, depth=10 | 10 Hz |

### Publish

| トピック | 型 | 説明 |
|---------|-----|------|
| `/estimated_pose` | `PoseStamped` | 最新推定ポーズ（測定タイムスタンプ） |
| `/estimated_path` | `Path` | 全ノードの推定軌跡 |

### TF

| 親フレーム | 子フレーム | 説明 |
|-----------|-----------|------|
| `map` | `base_link` | 最新推定ポーズ |

---

## エッジケース処理

| ケース | 処理 | 該当行 |
|--------|------|--------|
| 同一タイムスタンプ (dt < min_dt) | 同一ノードにPriorFactor付与 | L245-247 |
| タイムスタンプ逆転 (dt < 0) | 警告ログ + スキップ | L240-244 |
| センサドロップアウト | ノード作成レートが自然に変化 | 構造的に対応済み |
| 全測定値スキップ | early return | L296-298 |
| 初回測定値 | アンカーPriorFactor自動付与 | L254-256 |
