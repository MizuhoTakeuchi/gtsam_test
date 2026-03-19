# 因子グラフ最適化: 選択的因子除去と再最適化の実装

## アーキテクチャ: 2モード最適化

### 通常モード（500ms周期）
- ISAM2による増分最適化（高速）
- GPS/LiDARの測定値をPriorFactorとして追加
- ノード間はBetweenFactor（sqrt(dt)スケールのランダムウォークモデル）で接続

### 因子除去モード
- `/remove_factors`トピックに`"GPS"`または`"LIDAR"`をpublishすると発動
- 該当する全測定因子をshadow graphからフィルタリング
- LevenbergMarquardtでバッチ再最適化
- 再最適化結果でISAM2をリセットし、通常モードに復帰

## 主な追加要素

### Shadow Graph
- `shadow_graph_` + `factor_records_`で全因子のメタデータ（種別・タイムスタンプ・関連キー）を保持
- ISAM2とは別に完全なグラフ履歴を管理し、因子除去時のバッチ再構築を可能にする

### FactorRecord
各因子を以下の4種に分類:

| 種別 | 説明 | 除去対象 |
|------|------|----------|
| `ANCHOR_PRIOR` | 最初のノードのアンカー | No |
| `BETWEEN` | 連続ポーズ間の接続 | No |
| `GPS_MEASUREMENT` | GPS PriorFactor | Yes |
| `LIDAR_MEASUREMENT` | LiDAR PriorFactor | Yes |

### handleFactorRemoval()
因子除去のフロー:
1. 指定ソース（GPS/LiDAR）の因子をshadow graphからフィルタ
2. 残りの因子グラフでLevenbergMarquardt最適化
3. 最適化結果でISAM2を新規作成・初期化
4. shadow graph・factor records・パスを更新

### /remove_factors トピック
- 型: `std_msgs/msg/String`
- 外部からの除去トリガー
- ペイロード: `"GPS"` または `"LIDAR"`

### retention_duration_s パラメータ
- グラフ保持時間（デフォルト300秒）
- 数百秒間のグラフ履歴を保持可能

## 変更ファイル

| ファイル | 変更内容 |
|----------|----------|
| `src/pose_estimator_node.cpp` | shadow graph・因子除去・バッチ再最適化のロジック追加 |
| `CMakeLists.txt` | `std_msgs`依存追加 |
| `package.xml` | `std_msgs`依存追加 |
| `launch/pose_estimator.launch.py` | `retention_duration_s`パラメータ追加 |

## パラメータ一覧

| パラメータ | デフォルト値 | 説明 |
|-----------|-------------|------|
| `optimization_period_ms` | 500 | 最適化周期 (ms) |
| `between_noise_translation` | 1.0 | BetweenFactorの並進ノイズ密度 (sigma/sqrt(s)) |
| `between_noise_rotation` | 0.1 | BetweenFactorの回転ノイズ密度 (sigma/sqrt(s)) |
| `min_dt_for_new_node` | 0.001 | 新規ノード作成の最小時間差 (s) |
| `retention_duration_s` | 300.0 | グラフ保持時間 (s) |

## トピック一覧

### Subscribe
| トピック | 型 | 説明 |
|---------|-----|------|
| `/gps/pose_with_covariance` | `PoseWithCovarianceStamped` | GPS測定値 |
| `/lidar/pose_with_covariance` | `PoseWithCovarianceStamped` | LiDAR測定値 |
| `/remove_factors` | `String` | 因子除去トリガー |

### Publish
| トピック | 型 | 説明 |
|---------|-----|------|
| `/estimated_pose` | `PoseStamped` | 最新の推定姿勢 |
| `/estimated_path` | `Path` | 全軌跡 |

### TF
| parent | child | 説明 |
|--------|-------|------|
| `map` | `base_link` | 推定姿勢のTFブロードキャスト |

## 使い方

```bash
# ノード起動
ros2 launch gtsam_test pose_estimator.launch.py

# GPS因子を全除去して再最適化
ros2 topic pub --once /remove_factors std_msgs/msg/String "data: 'GPS'"

# LiDAR因子を全除去して再最適化
ros2 topic pub --once /remove_factors std_msgs/msg/String "data: 'LIDAR'"
```
