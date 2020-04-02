# RPi-nodeIO

1. 节点采用MQTT协议与上位机通讯。

2. MQTT的主题为：

    节点发布主题：node/01/status，用于读取输入/输出点状态。

    节点订阅主题：node/01/cmd，用于接收上位机发出的命令。

    上面主题里的“01”为节点编号，默认为“01”，可配置。

3. 节点配置4进4出共8个IO点，引脚分布见下表

    | 引脚功能 | BCM引脚 | 物理引脚 | 物理引脚 | BCM引脚 | 引脚功能 |
    | :------: | :-----: | :------: | :------: | :-----: | :------: |
    | 3.3V     |         | 1        | 2        |         | 5V       |
    |          | 2       | 3        | 4        |         | 5V       |
    |          | 3       | 5        | 6        |         | GND      |
    | 输出点1  | 4       | 7        | 8        | 14      |          |
    | GND      |         | 9        | 10       | 15      |          |
    | 输出点2  | 17      | 11       | 12       | 18      |          |
    | 输出点3  | 27      | 13       | 14       |         | GND      |
    | 输出点   | 22      | 15       | 16       | 23      |          |
    | 3.3V     |         | 17       | 18       | 24      |          |
    |          | 10      | 19       | 20       |         | GND      |
    |          | 9       | 21       | 22       | 25      |          |
    |          | 11      | 23       | 24       | 8       |          |
    | GND      |         | 25       | 26       | 7       |          |
    |          | 0       | 27       | 28       | 1       |          |
    | 输入点1  | 5       | 29       | 30       |         | GND      |
    | 输入点2  | 6       | 31       | 32       | 12      |          |
    | 输入点3  | 13      | 33       | 34       |         | GND      |
    |          | 19      | 35       | 36       | 16      |          |
    | 输入点4  | 26      | 37       | 38       | 20      |          |
    | GND      |         | 39       | 40       | 21      |          |

4. “node/01/status”主题发布的消息为UTF-8编码格式的文本，内容为被触发的输入点编号，默认为下降沿触发，可配置。

    例如“1”表示：输入点1被触发。

    注：当节点首次连上MQTT代理服务器时会在这个主题下发送“conneted”消息。

5. “node/01/cmd”主题发布的消息为UTF-8编码格式的文本，内容为想要设置输出点的状态。使用2个字符，高位字符表示输出点编号，低位字符表示欲设置的状态，“0”表示设为低电平，“1”表示设为高电平。

    例如“21”表示：将输出点2置高电平。

6. 若想要读取该节点所有输出点状态的话，先在“node/01/cmd”主题内发布消息“OP”，节点收到该消息后会在“node/01/status”主题内返回所有输出点的状态，以4个字符表示，从左到右依次表示从小到大的编号输出点状态，“0”表示低电平，“1”表示高电平。

    例如“0101”表示：输出点1为低电平，输出点2为高电平，输出点3为低电平，输出点4为高电平。

7. 若想要读取该节点所有输入点状态的话，先在“node/01/cmd”主题内发布消息“IP”，节点收到该消息后会在“node/01/status”主题内返回所有输入点的状态，以4个字符表示，从左到右依次表示从小到大的编号输入点状态，“0”表示低电平，“1”表示高电平。

    例如“0101”表示：输入点1为低电平，输入点2为高电平，输入点3为低电平，输入点4为高电平。

8. 配置文件路径：~/nodeIO/config.py。
