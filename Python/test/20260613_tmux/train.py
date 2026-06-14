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