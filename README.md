# Hobot TTS

该Node提供将文本转化为语音播放功能，定于文本消息，然后调用TTS软件接口，将文本转化为PCM数据，最后调用ALSA接口播放。

首次运行需要下载模型文件解压，详细命令如下：

```bash
wget http://archive.sunrisepi.tech//tts-model/tts_model.tar.gz
sudo tar -xf tts_model.tar.gz -C /opt/tros/lib/hobot_tts/
```

运行方式：

```bash
source /opt/tros/setup.bash

ros2 run hobot_tts hobot_tts
```
