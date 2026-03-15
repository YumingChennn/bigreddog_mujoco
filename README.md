# BigRedDog MuJoCo RL Lab

這是一個基於 MuJoCo 的四足機器人強化學習實驗項目，支援 BigRedDog 和 Go2 兩種機器人模型。

## 環境需求

- Python 3.10+
- PyTorch >= 2.1.0
- MuJoCo >= 3.2.0
- NumPy >= 1.24.0（< 2.0.0）
- PyYAML >= 6.0.0
- Matplotlib >= 3.7.0
- Pygame >= 2.6.0

## 環境設置

### 1. 建立 Conda 虛擬環境

```bash
# 建立名為 rlmujoco310 的虛擬環境
conda create -n rlmujoco310 python=3.10

# 啟動環境
conda activate rlmujoco310
```

### 2. 安裝依賴套件

#### 方法一：使用 requirements.txt（推薦）

```bash
pip install -r requirements.txt
```

#### 方法二：手動安裝

```bash
# 安裝 PyTorch (根據你的 CUDA 版本選擇)
# CPU 版本
pip install torch>=2.1.0

# GPU 版本 (CUDA 11.8)
pip install torch>=2.1.0 --index-url https://download.pytorch.org/whl/cu118

# 安裝其他依賴
pip install mujoco>=3.2.0
pip install "numpy>=1.24.0,<2.0.0"
pip install pyyaml>=6.0.0
pip install matplotlib>=3.7.0
pip install pygame>=2.6.0
```

啟動環境後，你的終端提示符應該會顯示如下：
```
(rlmujoco310) ray@ray-15Z980-G-AA75C2:~/bigreddog_mujoco$
```

## 專案結構

```
.
├── requirements.txt             # 依賴（repo root）
├── bigreddog/                   # BigRedDog / Go2 主程式與資源
│   ├── mujoco_rl_lab_big_reddog.py
│   ├── mujoco_rl_lab_go2.py
│   ├── keyboard_controller.py
│   ├── config/
│   │   ├── big_reddog_lab.yaml
│   │   └── go2_lab.yaml
│   ├── pre_train/
│   ├── urdf/
│   └── xml/
└── bigreddog_hieghtscan/        # BigRedDog heightscan（raycaster）
	├── mujoco_rl_him_big_reddog_heightscan.py
	├── config/
	│   └── big_reddog_him.yaml
	├── pre_train/
	└── xml/
```

## 使用方法

### 運行 BigRedDog 機器人模擬

```bash
cd bigreddog
python3 mujoco_rl_lab_big_reddog.py config/big_reddog_lab.yaml
```

### 運行 Go2 機器人模擬

```bash
cd bigreddog
python3 mujoco_rl_lab_go2.py config/go2_lab.yaml
```

### 運行 BigRedDog heightscan（raycaster）

```bash
cd bigreddog_hieghtscan
python3 mujoco_rl_him_big_reddog_heightscan.py config/big_reddog_him.yaml
```

在運行 heightscan 之前，請先看過並依照這個專案把 raycaster plugin 準備好（編譯/安裝）：
https://github.com/Albusgive/mujoco_ray_caster

heightscan 會在程式內呼叫：
`mujoco.mj_loadPluginLibrary('/home/ray/mujoco/plugin/mujoco_ray_caster/lib/libsensor_raycaster.so')`

請確認該 `.so` 檔案存在且路徑正確（若你的路徑不同，請修改 `bigreddog_hieghtscan/mujoco_rl_him_big_reddog_heightscan.py` 內的路徑）。

## 配置說明

配置文件 (YAML) 包含以下主要參數：

- `policy_path`: 訓練好的策略模型路徑
- `xml_path`: MuJoCo 場景 XML 文件路徑
- `simulation_duration`: 模擬總時長（秒）
- `simulation_dt`: 模擬時間步長
- `control_decimation`: 控制器更新頻率除數
- `kps`: PD 控制器的比例增益
- `kds`: PD 控制器的微分增益
- `default_angles`: 關節默認角度
- `action_scale`: 動作縮放因子
- `cmd_init`: 初始命令 [線速度x, 線速度y, 角速度z]

## 鍵盤控制

運行模擬時，可以使用鍵盤控制機器人移動：

- **W**: 向前移動
- **S**: 向後移動
- **A**: 向左移動
- **D**: 向右移動
- **Q**: 逆時針旋轉
- **E**: 順時針旋轉
- **Space**: 停止

## 注意事項

1. 確保所有路徑（特別是 `policy_path`）在配置文件中正確設置
2. 首次運行前，請確認預訓練模型文件存在
3. 如果遇到渲染問題，請確保已正確安裝 MuJoCo 及其依賴
4. 控制頻率為 50Hz (simulation_dt * control_decimation = 0.02s)
