# scancontext
```sh
pip3 install open3d
sudo pip install matplotlib --upgrade

#pip3安装(/home/lyu/.local/lib/python3.6/site-packages)失败,pip安装成功
#pip安装到了/home/lyu/miniconda3/lib/python3.8/site-packages
pip install Cython
pip install open3d
sudo cp /home/lyu/miniconda3/lib/python3.8/site-packages/open3d* /home/lyu/.local/lib/python3.6/site-packages/ -r
sudo cp /home/lyu/miniconda3/lib/python3.8/site-packages/Cython* /home/lyu/.local/lib/python3.6/site-packages/ -r
#拷贝到/home/lyu/.local/lib/python3.6/site-packages后也可使用
```