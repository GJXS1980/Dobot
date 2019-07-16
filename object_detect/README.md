## 安装nuitka
稳定版
```bash
CODENAME=`grep UBUNTU_CODENAME /etc/os-release | cut -d= -f2`
if ["$CODENAME"] = ""]
then
   CODENAME=`lsb_release -c -s`
fi;
wget -O - http://nuitka.net/deb/archive.key.gpg | apt-key add -
echo >/etc/apt/sources.list.d/nuitka.list "deb http://nuitka.net/deb/stable/$CODENAME $CODENAME main"
apt-get update
apt-get install nuitka

```
开发版
```bash
CODENAME=`grep UBUNTU_CODENAME /etc/os-release | cut -d= -f2`
if ["$CODENAME"] = ""]
then
   CODENAME=`lsb_release -c -s`
fi;
wget -O - http://nuitka.net/deb/archive.key.gpg | apt-key add -
echo >/etc/apt/sources.list.d/nuitka.list "deb http://nuitka.net/deb/develop/$CODENAME $CODENAME main"
apt-get update
apt-get install nuitka
```
[参考网站](https://nuitka.net/pages/download.html)

## 打包程序
```bash
pyinstaller -F test.py
```