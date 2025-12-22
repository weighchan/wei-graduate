vnc远程控制（==必须在同一个局域网下，否则需要内网穿透==）

1. 安装并启用SSH服务；

   ```txt
   $ sudo apt install openssh-server
   $ sudo systemctl status ssh.service 
   $ 确认是否开机自启动
   $ sudo systemctl is-enabled ssh.service 
   ```

2. related works；

   ```txt
   $ sudo apt-get install x11vnc
   
   $ 安装lightdm ————>lightdm
   $ sudo apt-get install lightdm
   
   $ 设置密码
   $ x11vnc -storepasswd
   
   ```

3. 配置VNC服务器

   ```txt
   $ 配置开机自启服务
   $ sudo vim /lib/systemd/system/x11vnc.service
   ---
   [Unit]
   Description=Start x11vnc at startup.
   After=multi-user.target display-manager.service
   Wants=display-manager.service
   
   [Service]
   Type=simple
   User=yourname
   Environment=DISPLAY=:0
   Environment=XAUTHORITY=/home/yourname/.Xauthority
   ExecStart=/usr/bin/x11vnc -display :0 -auth /home/yourname/.Xauthority -forever -loop -noxdamage -repeat -rfbauth /home/yourname/.vnc/passwd -rfbport 5900 -shared

   Restart=always
   RestartSec=5
   
   [Install]
   WantedBy=graphical.target
   ---
   $ sudo systemctl enable x11vnc.service
   $ sudo systemctl start x11vnc.service
   $ 查看一下服务器状态
   $ sudo systemctl status x11vnc.service
   ```
