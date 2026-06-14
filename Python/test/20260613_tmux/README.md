在深度学习或日常开发中，我们经常需要在服务器上运行耗时很长的训练脚本。如果直接在终端运行，一旦网络断开或关闭终端，训练就会中断。`tmux`（Terminal Multiplexer）就是解决这个问题的神器，它可以让你在后台运行终端，即使断开连接也能继续运行。

下面我将分两部分为你解答：首先创建一个模拟的深度学习训练脚本，然后详细演示如何使用 `tmux` 来运行它。

---

### 第一部分：创建一个模拟训练脚本

我们先写一个简单的 Python 脚本来模拟模型训练的过程（包含 Epoch、Loss 打印和耗时等待）。

新建一个文件叫 `train.py`，写入以下代码：

```python
import time
import random

def train_model(total_epochs=50):
    print("="*40)
    print("🚀 模型开始训练...")
    print("="*40)
    
    for epoch in range(1, total_epochs + 1):
        # 模拟训练时间（每个epoch耗时2秒）
        time.sleep(2)
        
        # 模拟Loss逐渐下降
        base_loss = 1.0 / epoch
        loss = base_loss + random.uniform(-0.05, 0.05)
        
        print(f"Epoch: {epoch}/{total_epochs} | Loss: {loss:.4f}")
        
    print("="*40)
    print("🎉 训练完成！")

if __name__ == "__main__":
    train_model(50)
```

---

### 第二部分：`tmux` 使用教程与实战

`tmux` 的核心概念是 **会话**。你可以把会话理解为后台运行的虚拟终端。

#### 1. 核心操作前缀键
在 `tmux` 中，所有的快捷键都需要先按**前缀键** `Ctrl+b`，松开后再按对应的命令键。

#### 2. 完整工作流示例（结合上面的训练脚本）

##### 步骤一：创建并进入一个新的 tmux 会话
登录服务器后，输入以下命令创建一个名为 `train_job` 的会话（起名字是为了方便以后找回来）：
```bash
tmux new -s train_job  # -s : session 会话
```
*执行后，你会进入一个新的终端界面，看起来和刚才没什么区别，但其实已经在 tmux 环境中了。*

##### 步骤二：在会话中运行训练脚本
在新终端中运行我们的脚本：
```bash
python train.py
```
此时脚本开始运行，不断打印 Epoch 和 Loss。

##### 步骤三：分离会话（让脚本在后台跑）
这时候你想去做别的事，或者直接关掉电脑下班。你需要把当前会话挂起（分离）：
1. 按下 `Ctrl + b`
2. 松开手，然后按 `d` (代表 detach)

*此时你会退回到最初的普通终端界面，但是 `train.py` 依然在服务器后台欢快地跑着！*

##### 步骤四：查看当前所有的 tmux 会话
下班回家后，你重新连上服务器，想看看后台有哪些任务在跑：
```bash
tmux ls
```
*输出类似：`train_job: 1 windows (created Mon May 20 10:00:00 2024)`*

##### 步骤五：重新连接到会话（查看训练进度）
你想看看 Loss 降到哪里了，重新接入刚才的会话：
```bash
tmux attach -t train_job # -t : target 会话名
```
*( `-t` 后面跟的是会话名，也可以简写为 `tmux a -t train_job`)*
*接入后，你会看到终端上依然在实时打印 Epoch 和 Loss，仿佛你从未离开过。*

##### 步骤六：翻页查看历史日志
如果 Loss 打印得太快，你想往上翻页查看之前的输出：
1. 按下 `Ctrl + b`
2. 松开手，然后按 `[` （进入复制/滚动模式）
3. 此时可以使用键盘的 **上下方向键** 或者 `PageUp` / `PageDown` 翻页查看。
4. 按 `q` 退出滚动模式。

##### 步骤七：彻底结束会话
训练完成后，你不需要这个会话了：
1. 在会话内部直接输入 `exit` 回车，或者按 `Ctrl + d`，会话会被关闭并退出。
2. 如果你在普通终端，想直接杀掉后台的会话：
```bash
tmux kill-session -t train_job
```

---

### 💡 进阶技巧：把日志同时输出到屏幕和文件

虽然 `tmux` 可以翻页，但它的缓冲区有限。在实际训练中，我们通常会把日志保存到文件里，方便后续分析。

你可以结合 `tee` 命令在 `tmux` 中运行脚本：

```bash
python train.py 2>&1 | tee train_log.txt
```
* `2>&1`：把错误输出也重定向到标准输出。
* `| tee train_log.txt`：把输出同时显示在屏幕上，并写入到 `train_log.txt` 文件中。

这样，即使 `tmux` 的历史缓冲区被清空，你依然可以通过 `cat train_log.txt` 查看完整的训练记录！