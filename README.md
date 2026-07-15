[![Build Status](https://github.com/RoboCup-SSL/grSim/workflows/Build/badge.svg)](https://github.com/RoboCup-SSL/grSim/actions?query=workflow%3ABuild+branch%3Amaster) [![CodeFactor](https://www.codefactor.io/repository/github/robocup-ssl/grsim/badge/master)](https://www.codefactor.io/repository/github/robocup-ssl/grsim/overview/master)

grSim
=======================

[RoboCup Small Size League](https://ssl.robocup.org/) シミュレータ。

![grSim on Ubuntu](docs/img/screenshot01.jpg?raw=true "grSim on Ubuntu")

- [インストール手順](INSTALL.md)
- [作者](AUTHORS.md)
- [変更履歴](CHANGELOG.md)
- ライセンス: [GNU General Public License (GPLv3)](LICENSE.md)

システム要件
-----------------------

grSim は、そこそこのグラフィックカードを積んだ最近のデュアルコア PC であればおおむね動作する。典型的な構成は以下の通り:

- デュアルコア CPU(2.0 GHz 以上)
- 1GB の RAM
- 256MB の nVidia または ATI グラフィックカード

より低スペックな環境でも動作する可能性はあるが、快適な性能は保証されない。


ソフトウェア要件
---------------------

grSim は Linux(Ubuntu および Arch Linux 系のみで動作確認済み)と Mac OS 上でビルドできる。以下のライブラリに依存する:

- [CMake](https://cmake.org/) バージョン 3.5 以上
- [pkg-config](https://freedesktop.org/wiki/Software/pkg-config/)
- [OpenGL](https://www.opengl.org)
- [Qt5 Development Libraries](https://www.qt.io)
- [Open Dynamics Engine (ODE)](http://www.ode.org)
- [VarTypes Library](https://github.com/jpfeltracco/vartypes)([Szi's Vartypes](https://github.com/szi/vartypes) からのフォーク)
- [Google Protobuf](https://github.com/google/protobuf)
- [Boost development libraries](http://www.boost.org/)(VarTypes が必要とする)

詳細は [インストール手順](INSTALL.md) を参照。

使い方
-----

grSim からデータを受信する方法は、[Google Protobuf](https://github.com/google/protobuf) ライブラリを使って [SSL-Vision](https://github.com/RoboCup-SSL/ssl-vision) からデータを受信する場合と同様である。
Google Protobuf を使ってシミュレータへデータを送信することも可能。サンプルクライアントは [clients](./clients) フォルダに含まれている。*Qt ベース* と *Java ベース* の2種類のクライアントが利用可能。ネイティブクライアントは grSim のコンパイル時に一緒にビルドされる。Java クライアントをコンパイルするには、対応する `README` ファイルを参照すること。

シミュレータとデータを送受信するための Qt の[サンプルプロジェクト](https://github.com/robocin/ssl-client)。

Star History
------
[![Star History Chart](https://api.star-history.com/svg?repos=robocup-ssl/grsim&type=Date)](https://star-history.com/#robocup-ssl/grsim&Date)

引用
------

研究で本ソフトウェアを利用する場合は、オリジナルの論文を引用すること:
```
@inproceedings{Monajjemi2011grSimR,
  title={grSim - RoboCup Small Size Robot Soccer Simulator},
  author={Valiallah Monajjemi and A. Koochakzadeh and S. S. Ghidary},
  booktitle={RoboCup},
  year={2011}
}
```

本リポジトリとその改変部分を特に引用したい場合は、以下を引用すること:

```
@misc{grsim2021,
  author = {Mohammad Mahdi Rahimi and Jan Segre and Valiallah Monajjemi and A. Koochakzadeh and Sepehr MohaimenianPour and Nicolai Ommer and  Avatar
Kazunori Kimura and Jeremy Feltracco and Kenta Sato and Atousa Ahsani},
  title = {GRSIM},
  year = {2021},
  publisher = {GitHub},
  note = {GitHub repository},
  howpublished = {\url{https://github.com/RoboCup-SSL/grSim/}}
}
```
