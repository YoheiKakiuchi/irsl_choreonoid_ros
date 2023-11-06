# Openlab2023で使用したデモスクリプト

## 動かし方
1. 事前準備
```
cd ~
git clone https://github.com/IRSL-tut/irsl_docker_irsl_system
docker pull irslrepo/irsl_system:noetic
```

1. 実際の動作
    1. Terminal Aを開いて以下コマンドを実行する．`/path/to/irsl_choreonoid_ros/sample/openlab2023`には自分のsampleへのpathを入れる．
        ```
        cd ~/irsl_docker_irsl_system
        ./run.sh -U -w /path/to/irsl_choreonoid_ros/sample/openlab2023 jupyter
        ```

    1. Terminal Bを開いて以下コマンドを実行する．
        ```
        cd ~/irsl_docker_irsl_system
        ./exec.sh
        ```
        ```
        roslaunch run_sim_robot.launch
        ```
    
    1. ブラウザを開き http://localhost:8888 にアクセスし，demo_prog.ipynbを開きセルを実行していく．