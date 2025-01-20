# CC1120のプログラム
[通信機の動作試験](https://yui-p.esa.io/posts/1274)

## src
### CC1120_TRX_class
- メインボードに実際に使うクラスのプログラム
- 工事中🚜
### test_RX
- RXのテストプログラム
- SIDLEからSRXに入る→RX_FIFOに入っているデータが15個以上になるまで待つ→RX_FIFOを読み出す→データが0個になったらSIDLEに戻るという動作。
### test_TX
- TXのテストプログラム
- SIDLEからSRXに入る→TX_FIFOにデータを入れる→SIDLEに戻るという動作。
### memo.md
- src内に関する気づき、注意点まとめ