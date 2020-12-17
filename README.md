# catkin_slam

### create submodule
>e.g sc-series
```sh
#step1. init submodule
git clone <main-repo-url>
cd <path>/src/sc-series
git submodule add <submodule-repo1>
git submodule add <submodule-repo2>
git submodule add <submodule-repo3>
git submodule add <submodule-repo4> #本地repo里出现.gitmodules 里面含有各个submodules信息
#e.g git submodule add https://github.com/gisbi-kim/PyICP-SLAM.git
#e.g git submodule add https://github.com/gisbi-kim/SC-LIO-SAM.git

git status 
git commit -m"add submodule repos" 
git push  #远程仓库将出现.gitmodules且对于目录有submodule-repo(直接链接到sub-repo的地址)

#step2. 拉取repo
#2-1
git clone <main-repo-url> #sub-repo为空文件夹
git submodule init
git submodule update
#2-2
git clone <main-repo-url> --recurse-submodules #拉取main-repo 以及 sub-repo
#2-3 update sub-repo
git submodule foreach 'git pull origin master' #等同于在各个sub-repo内单独执行"git pull origin master"
```