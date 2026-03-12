# BigRedDog MuJoCo RL Lab

這是一個基於 MuJoCo 的四足機器人強化學習實驗項目，支援 BigRedDog 和 Go2 兩種機器人模型。

## 環境需求

- Python 3.8+
- PyTorch >= 1.10.0
- MuJoCo >= 3.2.0
- NumPy >= 1.24.0 (< 2.0.0)
- PyYAML >= 6.0.0
- Matplotlib >= 3.7.0
- Pygame >= 2.6.0

## 環境設置

### 1. 建立 Conda 虛擬環境

```bash
# 建立名為 rlmujoco 的虛擬環境
conda create -n rlmujoco python=3.8

# 啟動環境
conda activate rlmujoco
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
pip install torch>=1.10.0

# GPU 版本 (CUDA 11.8)
pip install torch>=1.10.0 --index-url https://download.pytorch.org/whl/cu118

# 安裝其他依賴
pip install mujoco>=3.2.0
pip install "numpy>=1.24.0,<2.0.0"
pip install pyyaml>=6.0.0
pip install matplotlib>=3.7.0
pip install pygame>=2.6.0
```

啟動環境後，你的終端提示符應該會顯示如下：
```
(rlmujoco) ray@ray-15Z980-G-AA75C2:~/bigreddog_mujoco$
```

## 專案結構

```
.
├── config/                      # 配置文件目錄
│   ├── big_reddog_lab.yaml     # BigRedDog 機器人配置
│   └── go2_lab.yaml            # Go2 機器人配置
├── pre_train/                   # 預訓練模型目錄
│   └── robot_lab/
│       ├── big_reddog/
│       │   └── 0209_1201/
│       │       └── policy.pt   # BigRedDog 訓練好的策略模型
│       └── go2/
│           └── 0120_1655/
│               └── policy.pt   # Go2 訓練好的策略模型
├── xml/                         # MuJoCo XML 場景文件
│   ├── scene_big_reddog.xml
│   ├── scene_go2.xml
│   ├── big_reddog_new.xml
│   └── go2.xml
├── urdf/                        # URDF 模型文件
│   └── bigreddog.urdf
├── mujoco_rl_lab_big_reddog.py # BigRedDog 主程式
├── mujoco_rl_lab_go2.py        # Go2 主程式
└── keyboard_controller.py       # 鍵盤控制器
```

## 使用方法

### 運行 BigRedDog 機器人模擬

```bash
python3 mujoco_rl_lab_big_reddog.py config/big_reddog_lab.yaml
```

### 運行 Go2 機器人模擬

```bash
python3 mujoco_rl_lab_go2.py config/go2_lab.yaml
```

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
