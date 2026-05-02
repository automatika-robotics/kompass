<div align="center">

<picture>
  <source media="(prefers-color-scheme: dark)" srcset="_static/Kompass_dark.png">
  <source media="(prefers-color-scheme: light)" srcset="_static/Kompass_light.png">
  <img alt="Kompass Logo" src="_static/Kompass_light.png" width="600">
</picture>

<br/>

[EMOS](https://github.com/automatika-robotics/emos) エコシステムの一部

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Python](https://img.shields.io/badge/python-3.8+-blue.svg)](https://www.python.org/downloads/)
[![ROS2](https://img.shields.io/badge/ROS2-Foxy%2B-green)](https://docs.ros.org/en/humble/index.html)
[![Discord](https://img.shields.io/badge/Discord-%235865F2.svg?logo=discord&logoColor=white)](https://discord.gg/B9ZU6qjzND)

**高性能・イベント駆動型 ROS 2 ナビゲーションスタック**

[**EMOS ドキュメント**](https://emos.automatikarobotics.com) | [**開発者ドキュメント**](https://automatika-robotics.github.io/kompass/) | [**Discord**](https://discord.gg/B9ZU6qjzND)

🇬🇧 [English](../README.md) | 🇨🇳 [简体中文](README.zh.md)

</div>

---

## Kompass とは？

**Kompass** は [EMOS](https://github.com/automatika-robotics/emos)（Embodied Operating System）のナビゲーションレイヤーです。Kompass は自律移動ロボット向けの**堅牢**で**イベント駆動型**のナビゲーションスタックを構築するためのフレームワークです。カスタマイズ可能、拡張可能、ハードウェア非依存に設計されており、統合・拡張・適応が容易な直感的な Python API を提供します。

Kompass には**高度に最適化された GPU 対応の C++ ナビゲーションアルゴリズム**が含まれており、利用可能なハードウェアリソースを最大限に活用します。**CPU マルチスレッド実行**をサポートし、**任意の GPU**（Nvidia、AMD など）上で動作可能で、ベンダーロックインがありません。最も重要なのは、Kompass により**単一の Python スクリプト内**で高度なナビゲーション機能を簡単に作成・デプロイできることです。パフォーマンスや柔軟性を犠牲にすることはありません。

完全なドキュメント、チュートリアル、レシピは [emos.automatikarobotics.com](https://emos.automatikarobotics.com) をご覧ください。

- [**主な機能**](#主な機能)
- [**コンポーネント**](#コンポーネント)
- [**インストール**](#インストール)
- [**ベンチマーク結果**](#ベンチマーク結果)
- [**動的 Web UI**](#動的-web-ui)
- [**EMOS エコシステム**](#emos-エコシステムの一部)

---

## 主な機能

- **適応型イベント駆動設計**：現実世界のイベント、ロボットの状態変化、タスク更新に応答します。イベント-アクションペアを定義してランタイムにナビゲーションスタックを再構成したり、環境コンテキストに基づいて計画・制御戦略をシームレスに切り替えたりできます。

- **速度のために設計**：すべてのコアアルゴリズムはモダン C++ で記述されています（[kompass-core](https://github.com/automatika-robotics/kompass-core)）。GPGPU による GPU ベースの実行を明示的にサポートした初のナビゲーションフレームワークで、ベンダーロックインなしであらゆるハードウェアで高性能を発揮します。

- **ML モデルをファーストクラスシチズンとして**：外部イベントはセンサーデータやユーザーコマンドを解釈する ML モデルの出力によって駆動できます。スタック全体が ML 推論に基づいて再構成可能になり、従来のビジュアルナビゲーションシナリオを超えます。

- **Pythonic API とネイティブスピード**：重い処理は最適化された C++ が行い、Kompass は直感的な Python API を提供します。素早くプロトタイプを作成し、コードを書き直すことなく高性能システムをデプロイできます。

---

## コンポーネント

Kompass は複数の相互作用するコンポーネントに分かれており、それぞれがナビゲーションのサブタスクを担当します：

<div align="center">
<picture>
  <source media="(prefers-color-scheme: dark)" srcset="_static/images/diagrams/system_components_dark.png">
  <source media="(prefers-color-scheme: light)" srcset="_static/images/diagrams/system_components_light.png">
  <img alt="Kompass コンポーネント" src="_static/images/diagrams/system_components_dark.png" width="100%">
</picture>
</div>

各コンポーネントは ROS 2 ライフサイクルノードとして動作し、トピック、サービス、またはアクションサーバーを使って通信します：

<div align="center">
<picture>
  <source media="(prefers-color-scheme: dark)" srcset="_static/images/diagrams/system_graph_dark.png">
  <source media="(prefers-color-scheme: light)" srcset="_static/images/diagrams/system_graph_light.png">
  <img alt="システム図" src="_static/images/diagrams/system_graph_dark.png" width="100%">
</picture>
</div>

各コンポーネントの詳細は [EMOS ドキュメント](https://emos.automatikarobotics.com) でご確認ください：
[プランナー](https://emos.automatikarobotics.com/kompass/navigation/path_planning.html) |
[コントローラー](https://emos.automatikarobotics.com/kompass/navigation/control.html) |
[ローカルマッパー](https://emos.automatikarobotics.com/kompass/navigation/mapper.html) |
[マップサーバー](https://emos.automatikarobotics.com/kompass/navigation/map_server.html) |
[ドライブマネージャー](https://emos.automatikarobotics.com/kompass/navigation/driver.html) |
[モーションサーバー](https://emos.automatikarobotics.com/kompass/navigation/motion_server.html)

---

## インストール

詳細なインストール手順は [EMOS ドキュメント](https://emos.automatikarobotics.com) をご参照ください。

### kompass-core のインストール

**GPU サポート付き（推奨）：**

```bash
curl -sSL https://raw.githubusercontent.com/automatika-robotics/kompass-core/refs/heads/main/build_dependencies/install_gpu.sh | bash
```

**pip でインストール（CPU のみ）：**

```bash
sudo apt-get install -y libompl-dev libfcl-dev && pip install kompass-core
```

### Kompass のインストール

**クイックインストール（Ubuntu/Debian、ROS 2 Humble+ または Jazzy+）：**

```bash
sudo apt install ros-$ROS_DISTRO-kompass
```

**ソースからビルド（開発者向け）：**

```bash
mkdir -p kompass_ws/src && cd kompass_ws/src
git clone https://github.com/automatika-robotics/sugarcoat
git clone https://github.com/automatika-robotics/kompass
rosdep update && rosdep install -y --from-paths . --ignore-src
cd .. && colcon build
```

---

## ベンチマーク結果

以下のプロットはナビゲーションコンポーネント全体の CPU と GPU のパフォーマンスを比較しています。手法の詳細は[ベンチマーク詳細](https://github.com/automatika-robotics/kompass-core/blob/main/src/kompass_cpp/benchmarks/README.md)をご覧ください。

### 対数スケール（CPU vs GPU）

<div align="center">
<picture>
  <source media="(prefers-color-scheme: dark)" srcset="https://raw.githubusercontent.com/automatika-robotics/kompass-core/main/docs/benchmark_log_dark.png">
  <source media="(prefers-color-scheme: light)" srcset="https://raw.githubusercontent.com/automatika-robotics/kompass-core/main/docs/benchmark_log_light.png">
  <img alt="対数ベンチマーク結果" src="https://raw.githubusercontent.com/automatika-robotics/kompass-core/main/docs/benchmark_log_light.png" width="60%">
</picture>
</div>

### 消費電力と効率

_効率 = ジュールあたりの操作数（スループット / ワット）。高いほど良い。_

<div align="center">
<picture>
  <source media="(prefers-color-scheme: dark)" srcset="https://raw.githubusercontent.com/automatika-robotics/kompass-core/main/docs/benchmark_power_dark.png">
  <source media="(prefers-color-scheme: light)" srcset="https://raw.githubusercontent.com/automatika-robotics/kompass-core/main/docs/benchmark_power_light.png">
  <img alt="電力ベンチマーク結果" src="https://raw.githubusercontent.com/automatika-robotics/kompass-core/main/docs/benchmark_power_light.png" width="60%">
</picture>
</div>

---

## 動的 Web UI

すべての Kompass レシピは動的 Web UI を自動生成します。[Sugarcoat](https://github.com/automatika-robotics/sugarcoat) フレームワーク上の FastHTML で構築され、フロントエンドコードを書くことなく即座に制御と可視化を提供します。

<div align="center">
<picture>
  <img alt="Kompass UI" src="_static/gif/ui_navigation.gif" width="60%">
</picture>
</div>

---

## EMOS エコシステムの一部

Kompass は [EMOS](https://github.com/automatika-robotics/emos)（Embodied Operating System）の 3 つのコアオープンソースコンポーネントの 1 つです。Physical AI のための統合オーケストレーションレイヤー：

- **[EmbodiedAgents](https://github.com/automatika-robotics/embodied-agents)**：知能と操作。セマンティックメモリと適応的再構成を備えた ML モデルグラフ。
- **[Kompass](https://github.com/automatika-robotics/kompass)**：ナビゲーション。GPU 加速のプランニングと制御。
- **[Sugarcoat](https://github.com/automatika-robotics/sugarcoat)**：堅牢なイベント駆動型 ROS 2 システム設計。

レシピを一度書けば、任意のロボットにデプロイ。コード変更は不要です。

---

## リソース

- [EMOS ドキュメント](https://emos.automatikarobotics.com)：チュートリアル、レシピ、使用ガイド
- [開発者ドキュメント](https://automatika-robotics.github.io/kompass/)：拡張、カスタムコンポーネント、API リファレンス
- [Discord](https://discord.gg/B9ZU6qjzND)：コミュニティとサポート

## 著作権と貢献

**Kompass** は [Automatika Robotics](https://automatikarobotics.com/) と [Inria](https://inria.fr/) の共同開発です。

コードは **MIT ライセンス** の下で提供されています。詳細は [LICENSE](../LICENSE) をご覧ください。
Copyright (c) 2024 Automatika Robotics（特に明記されている場合を除く）。
