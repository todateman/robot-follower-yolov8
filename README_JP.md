# YOLOv8を使用したロボットフォロワー

このプロジェクトは、ROS 2 (Humble) とYOLOv8を使用して人間検出を行うロボットフォロワーシステムを実装しています。リアルタイムで人を検出・追跡し、深度カメラデータを使用して距離を計算し、近接度に基づいてロボットの動作を制御します。精密な速度制御、滑らかな動作、RVizでの追跡可視化機能を備えています。

## 機能

- **リアルタイム人間検出** - YOLOv8による高精度・高速検出（推論時間約8-12ms）
- **距離ベースの追跡** - 設定可能な追跡範囲（0.5m - 2.0m）
- **滑らかな動作制御** - 指数平滑化による安定した移動
- **RViz可視化** - ターゲット追跡用マーカー表示
- **複数カメラサポート** - RealSense D435iおよびその他のRGB-Dカメラに対応
- **ROS 2統合** - 標準トピックによる簡単な統合

## ハードウェア要件

- RGB-Dカメラ（Intel RealSense D435iでテスト済み）
- 差動駆動対応ロボットプラットフォーム
- CUDA対応GPU搭載コンピュータ（最適なパフォーマンスのため推奨）

## ソフトウェア依存関係

- **Python 3.10+**
- **ROS 2 Humble**
- **YOLOv8 (ultralytics)**
- **OpenCV**
- **NumPy 1.24.2**（ROS 2互換性のため）
- **cv_bridge**
- **RealSense SDK**（Intel RealSenseカメラ用）

## 依存関係ファイル

このプロジェクトには、異なる用途のために複数の依存関係ファイルが含まれています：

- `requirements.txt` - 完全な環境再現のための正確なバージョンを含む完全な凍結依存関係
- `requirements-minimal.txt` - 最小限の依存関係（メインパッケージのみ、依存関係は自動解決）
- `pyproject.toml` - uvネイティブ依存関係管理を備えた現代的なPythonパッケージング

## インストール

### 1. リポジトリのクローン

```bash
git clone https://github.com/ahmadidrisyakubu/robot-follower-yolov8.git
cd robot-follower-yolov8
```

### 2. uvによる仮想環境セットアップ

```bash
# uvがインストールされていない場合はインストール
curl -LsSf https://astral.sh/uv/install.sh | sh

# Python 3.10仮想環境の作成
uv venv --python 3.10 robot-follower-env-310

# オプションA: 最小依存関係のインストール（推奨）
uv pip install --python robot-follower-env-310/bin/python -r requirements-minimal.txt

# オプションB: 完全な依存関係のインストール（環境の完全な再現）
uv pip install --python robot-follower-env-310/bin/python -r requirements.txt

# オプションC: pyproject.tomlを使用したインストール（uvネイティブ）
uv sync
```

### 3. RealSenseカメラのセットアップ（Intel RealSense D435i使用の場合）

```bash
# RealSense SDKのインストール
sudo apt update
sudo apt install ros-humble-realsense2-*

# RealSenseカメラの起動
ros2 launch realsense2_camera rs_launch.py
```

## 使用方法

### 1. RealSenseカメラの起動

```bash
# ターミナル1
ros2 launch realsense2_camera rs_launch.py
```

### 2. ロボットフォロワーの実行

```bash
# ターミナル2
source /opt/ros/humble/setup.bash
robot-follower-env-310/bin/python Robot_follower.py
```

### 3. システムの監視

様々なROS 2ツールを使用してロボットの動作を監視できます：

#### 人間検出の確認

```bash
# ログをフィルタリングして人間検出を確認
ros2 run robot-follower-yolov8 Robot_follower.py 2>&1 | grep "person"
```

#### 速度コマンドの監視

```bash
# ロボットの速度コマンドを確認
ros2 topic echo /cmd_vel
```

#### RVizでの可視化

```bash
# 可視化用RVizの起動
rviz2
# マーカートピックを追加：/visualization_marker
```

#### カメラトピックの確認

```bash
# 利用可能なカメラトピックのリスト表示
ros2 topic list | grep camera

# カメラデータの監視
ros2 topic hz /camera/camera/color/image_raw
ros2 topic hz /camera/camera/depth/image_rect_raw
```

## 設定

`Robot_follower.py`内のパラメータを変更することで、ロボットフォロワーを設定できます：

### 追跡パラメータ

```python
self.min_follow_distance = 0.5  # 追跡開始距離（メートル）
self.max_follow_distance = 2.0  # 追跡停止距離（メートル）
```

### 速度制御

```python
self.linear_velocity_scale = 0.1   # 直進速度スケール係数
self.angular_velocity_scale = 2.0  # 角速度スケール係数
```

### 検出パラメータ

```python
self.confidence_threshold = 0.1      # YOLO検出の最小信頼度
self.distance_threshold = 50         # バウンディングボックス関連付けの最大距離（ピクセル）
```

### 平滑化パラメータ

```python
self.omega = 0.3  # 指数平滑化のフィルタ係数（0-1）
```

## ROS 2トピック

### 購読トピック

- `/camera/camera/color/image_raw` (sensor_msgs/Image) - RGBカメラフィード
- `/camera/camera/depth/image_rect_raw` (sensor_msgs/Image) - 深度カメラフィード
- `/camera/camera/color/camera_info` (sensor_msgs/CameraInfo) - カメラキャリブレーションデータ

### 発行トピック

- `/cmd_vel` (geometry_msgs/Twist) - ロボット速度コマンド
- `/visualization_marker` (visualization_msgs/Marker) - RViz可視化マーカー

## トラブルシューティング

### 人は検出されるがロボットが動かない

1. 人が追跡範囲内（0.5m - 2.0m）にいるか確認
2. 深度データが有効か確認：`ros2 topic echo /camera/camera/depth/image_rect_raw`
3. 速度コマンドを確認：`ros2 topic echo /cmd_vel`

### カメラトピックが見つからない

1. RealSenseカメラが接続され、起動されているか確認
2. 利用可能なトピックを確認：`ros2 topic list | grep camera`
3. カメラの権限とUSB接続を確認

### YOLOモデルのダウンロード問題

YOLOv8モデル（`yolov8m-seg.pt`）は初回実行時に自動ダウンロードされます。ダウンロードに失敗した場合：

1. インターネット接続を確認
2. 手動ダウンロード：<https://github.com/ultralytics/assets/releases/download/v8.3.0/yolov8m-seg.pt>

### NumPy互換性問題

NumPyバージョンの競合が発生した場合：

```bash
uv pip install --python robot-follower-env-310/bin/python "numpy==1.24.2" --force-reinstall
```

## パフォーマンス最適化

### より良いパフォーマンスのために

1. **GPU加速の使用** - YOLOv8 GPU推論のためのCUDAの適切なインストール
2. **画像解像度の調整** - より高速な処理のためのカメラ解像度低下
3. **信頼度閾値の調整** - 高い閾値は偽陽性を減らしますが検出を見逃す可能性があります
4. **フィルタリングの最適化** - より滑らかだが潜在的により遅い応答のための`omega`パラメータ調整

## 安全に関する考慮事項

1. **安全な環境でのテスト** - 常に清潔で安全なエリアでテストしてください
2. **緊急停止** - ロボットを停止するための手動制御を利用可能にしておいてください
3. **障害物回避** - このシステムは人を追跡するのみです - 実際の配備では障害物回避を追加してください
4. **速度制限** - 安全な動作のために速度スケールを低く保ってください

## ライセンス

このプロジェクトはBSD 3-Clause Licenseの下でライセンスされています - 詳細はLICENSEファイルを参照してください。

## コントリビューション

1. リポジトリをフォーク
2. 機能ブランチを作成
3. 変更をコミット
4. ブランチにプッシュ
5. プルリクエストを作成

## サポート

問題や質問については：

1. 上記のトラブルシューティングセクションを確認
2. ROS 2およびYOLOv8のドキュメントを確認
3. GitHubリポジトリでissueを作成

## 参考リンク

- [ROS 2 Humble ドキュメント](https://docs.ros.org/en/humble/)
- [YOLOv8 (Ultralytics) ドキュメント](https://docs.ultralytics.com/)
- [Intel RealSense SDK](https://github.com/IntelRealSense/librealsense)
- [元のプロジェクト](https://github.com/ahmadidrisyakubu/robot-follower-yolov8)
