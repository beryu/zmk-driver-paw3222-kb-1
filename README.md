# ZMK PAW3222 driver for kb-1

[kb-1](https://github.com/beryu/kb-1)で使用するPAW3222トラックボール用のZMKドライバです。
Seeed Studio XIAO nRF52840 Plusの`spi2`と、PAW3222の3線式SPI（MOSI/MISO共通）に対応しています。

このリポジトリはkb-1専用です。torabo-tsuki-lpでは
[`beryu/zmk-driver-paw3222`](https://github.com/beryu/zmk-driver-paw3222)を使用してください。

## 対応構成

- SoC: Nordic nRF52840
- SPIコントローラー: `spi0`〜`spi3`（kb-1は`spi2`）
- 通信方式: PAW3222 3線式SPI
- モーション検出: Active Low割り込み
- センサー電源制御とZephyr Device PM
- ZMKレイヤー連動スマートスクロール

## kb-1での使用

ファームウェア側の`config/west.yml`から、検証済みコミットをSHAで指定します。

```yaml
- name: zmk-driver-paw3222
  url: https://github.com/beryu/zmk-driver-paw3222-kb-1
  revision: <commit-sha>
```

kb-1でのピン割り当ては次のとおりです。

| 信号 | XIAO | nRF52840 GPIO |
| --- | --- | --- |
| センサー電源 | D6 | P1.11 |
| CS | D7 | P1.12 |
| SCLK | D8 | P1.13 |
| MOTION | D9 | P1.14 |
| SDIO | D10 | P1.15 |

実際のdevicetree設定とビルド構成は
[`beryu/zmk-keyboard-kb-1`](https://github.com/beryu/zmk-keyboard-kb-1)を参照してください。

## Devicetreeプロパティ

- `irq-gpios`: MOTION端子（必須、Active Low）
- `power-gpios`: センサー電源制御端子（任意）
- `res-cpi`: CPI解像度（任意）
- `force-awake`: センサーの省電力モードを無効化（任意）

## 検証

ドライバ変更時は、ファームウェアリポジトリで以下の5構成をビルドして確認します。

- 左Central（トラックボールあり）
- 右Central（トラックボールあり）
- 左Peripheral
- 右Peripheral
- Settings Reset

ビルド成功だけでは実機上のSPI通信、移動方向、スリープ復帰、消費電流は確認できません。
