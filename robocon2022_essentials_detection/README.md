# robocon2022_essentials_detection

NuiTrackから得られた情報を処理し、メッセージをパブリッシュする

## Logic

NuiTrakから得られる以下のメッセージを使用する。

|  Message          |  Description            |
|-------------------|-------------------------|
| BodyTrackerArray  | スケルトンの２次元座標を使う |
| Gestures          | ジェスチャーの識別         |

タイムスタンプとUserIDから双方のメッセージを紐付けする。
Bodyの２次元座標が中心に近いものかつ、信用度(confident)が高いものを優先的に処理する。  
処理したデータを以下のメッセージでパブリッシュする。
```
/robocon2022_essentials_detection/UserAction
```

## Dependency

- [TNCT-Mechatech/nuitrack_body_tracker](https://github.com/TNCT-Mechatech/nuitrack_body_tracker)
- [TNCT-Mechatech/body_tracker_msgs](https://github.com/TNCT-Mechatech/body_tracker_msgs)