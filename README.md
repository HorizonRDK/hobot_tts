# Hobot TTS

**hobot_tts**提供将文本转化为语音播放功能，订阅文本消息，然后调用TTS软件接口，将文本转化为PCM数据，最后调用ALSA接口播放。

首次运行需要下载模型文件解压，详细命令如下：

```bash
wget http://archive.sunrisepi.tech//tts-model/tts_model.tar.gz
sudo tar -xf tts_model.tar.gz -C /opt/tros/lib/hobot_tts/
```

环境搭建：

运行hobot_tts，需要搭配地平线专用音频驱动板，或修改代码中打开的音频设备。使用地平线专用音频驱动板板子上电后需要运行如下命令加载音频驱动：

```bash
echo 8 >/proc/sys/kernel/printk
echo 112 >/sys/class/gpio/export
echo out >/sys/class/gpio/gpio112/direction
echo 1 >/sys/class/gpio/gpio112/value
echo 118 >/sys/class/gpio/export
echo out >/sys/class/gpio/gpio118/direction
echo 1 >/sys/class/gpio/gpio118/value


modprobe -r es7210
modprobe -r es8156
modprobe -r hobot-i2s-dma
modprobe -r hobot-cpudai
modprobe -r hobot-snd-7210


modprobe es7210
modprobe es8156
modprobe hobot-i2s-dma
modprobe hobot-cpudai
modprobe hobot-snd-7210 snd_card=5
```

运行上述命令后使用如下命令可检查是否加载成功：

```bash
root@ubuntu:~# ls /dev/snd/
by-path  controlC0  pcmC0D0c  pcmC0D1p  timer
```

如果出现`pcmC0D1p`设备则表示加载成功。

运行方式：

```bash
source /opt/tros/setup.bash

# 屏蔽调式打印信息
export GLOG_minloglevel=1

ros2 run hobot_tts hobot_tts
```

运行成功后，程序订阅topic "/tts_text"（消息类型为std_msgs/msg/String），然后转化为语音信号播放。

参数列表：

| 参数名          | 解释            | 类型        | 是否必须 | 默认值      |
| --------------- | -------------- | ----------- | --------| ----------- |
| topic_sub       | 订阅的文本topic | std::string | 否      | "/tts_text" |
