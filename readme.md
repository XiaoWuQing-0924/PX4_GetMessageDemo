how to use?
```bash
xl@jxl-computer:~/note/PX4_GetMessageDemo$ make
g++ -g -w -I ./mavlink -o PX4_GetMessageDemo main.c
jxl@jxl-computer:~/note/PX4_GetMessageDemo$ ./PX4_GetMessageDemo 
xacc yacc zacc: -1391205209: -0.057532	 -0.171265	 -9.730990
xacc yacc zacc: -1391182645: -0.056959	 -0.164590	 -9.782205
xacc yacc zacc: -1391160170: -0.084184	 -0.128646	 -9.841348
xacc yacc zacc: -1391133148: -0.060414	 -0.162370	 -9.771601
xacc yacc zacc: -1391110584: -0.062588	 -0.130401	 -9.809486
xacc yacc zacc: -1391083560: -0.057659	 -0.147555	 -9.815854
xacc yacc zacc: -1298620685: -0.044296	 -0.129360	 -9.802094
xacc yacc zacc: -1298395495: -0.045490	 -0.143112	 -9.788814
xacc yacc zacc: -1298165846: -0.075727	 -0.152035	 -9.828282
xacc yacc zacc: -1297940656: -0.065174	 -0.149878	 -9.803563
xacc yacc zacc: -1297710916: -0.070294	 -0.132845	 -9.741825
xacc yacc zacc: -1297620840: -0.090882	 -0.122823	 -9.771341
xacc yacc zacc: -1297602825: -0.073047	 -0.131591	 -9.786416
xacc yacc zacc: -1297580352: -0.074203	 -0.147583	 -9.759832
xacc yacc zacc: -1297562336: -0.087753	 -0.111884	 -9.857010
xacc yacc zacc: -1297539771: -0.057879	 -0.155707	 -9.844431
...
```


1、px4设备被识别为：/dev/ttyACM0
2、打开串口，并设置属性
3、向px4发送心跳包（否则px4不会发送mavlink消息）
4、循环接受px4发送过来的mavlink消息，并对感兴趣的消息解码
